"""Batch GTSAM 4.2 SE(3), component alignment and explicit odometry noise."""
from collections import defaultdict, deque
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import pose, inv, constraint, canonical_edge, TANGENT_ORDER


def delta(a,b):
    T=inv(a)@b
    return np.linalg.norm(T[:3,3]),Rotation.from_matrix(T[:3,:3]).magnitude()


def active_components(robots,edges):
    adjacency=defaultdict(set)
    for edge in edges:
        a,b=edge['i'][0],edge['j'][0]
        adjacency[a].add(b);adjacency[b].add(a)
    result={}
    for robot in sorted(robots):
        if robot in result:continue
        result[robot]=robot;queue=deque([robot])
        while queue:
            for other in sorted(adjacency[queue.popleft()]):
                if other not in result:result[other]=robot;queue.append(other)
    return result


def align_robots(rows,loops,cfg):
    by_id={(r['robot_id'],r['keyframe_id']):r for r in rows}
    robots=sorted({r['robot_id'] for r in rows}); hypotheses=defaultdict(list)
    for original in loops:
        edge=canonical_edge(original);i,j=tuple(edge['i']),tuple(edge['j'])
        if i[0]==j[0]:continue
        A=pose(by_id[i]['T_world_body'])@pose(edge['T_i_j'])@inv(pose(by_id[j]['T_world_body']))
        hypotheses[(i[0],j[0])].append((A,i,j))
    adjacency=defaultdict(list);reports=[]
    for (a,b),hs in sorted(hypotheses.items()):
        clusters=[]
        for seed in hs:
            cluster=[seed]
            for h in hs:
                if h is seed:continue
                if all(delta(h[0],member[0])[0]<=cfg['alignment_translation_m'] and
                       delta(h[0],member[0])[1]<=np.deg2rad(cfg['alignment_rotation_deg']) for member in cluster):
                    cluster.append(h)
            clusters.append(cluster)
        cluster=max(clusters,key=len)
        # Distinct endpoints required: repeated constraints at one location do not
        # manufacture support for an unknown robot alignment.
        support=min(len({h[1] for h in cluster}),len({h[2] for h in cluster}))
        ok=support>=cfg['alignment_min_support']
        reports.append(dict(robots=[a,b],hypotheses=len(hs),consistent=len(cluster),support=support,accepted=ok))
        if not ok:continue
        A=np.eye(4);A[:3,3]=np.mean([h[0][:3,3] for h in cluster],axis=0)
        A[:3,:3]=Rotation.from_matrix(np.stack([h[0][:3,:3] for h in cluster])).mean().as_matrix()
        adjacency[a].append((b,A));adjacency[b].append((a,inv(A)))
    alignment={};membership={}
    for robot in robots:
        if robot in alignment:continue
        alignment[robot]=np.eye(4);membership[robot]=robot;q=deque([robot])
        while q:
            a=q.popleft()
            for b,A in sorted(adjacency[a],key=lambda x:x[0]):
                if b in alignment:continue
                alignment[b]=alignment[a]@A;membership[b]=robot;q.append(b)
    return alignment,membership,reports


def optimize(rows,loops,cfg):
    import gtsam
    rows=sorted(rows,key=lambda r:(r['robot_id'],r['keyframe_id']))
    by_id={(r['robot_id'],r['keyframe_id']):r for r in rows}
    unique={};rejected=[]
    for e in loops:
        if not e.get('accepted'):continue
        if e.get('tangent_order')!=TANGENT_ORDER:raise ValueError('Unsupported constraint tangent ordering')
        e=constraint(e['i'],e['j'],e['T_i_j'],e['information'],**{k:v for k,v in e.items() if k not in
            ('i','j','T_i_j','information','tangent_order','perturbation','schema_version')})
        e=canonical_edge(e);i,j=tuple(e['i']),tuple(e['j'])
        if i not in by_id or j not in by_id:raise ValueError('Unknown constraint endpoint')
        if (i,j) in unique:rejected.append(dict(reason='duplicate_constraint',i=list(i),j=list(j)));continue
        unique[i,j]=e
    loops=list(unique.values())
    alignment,components,alignment_report=align_robots(rows,loops,cfg)
    graph=gtsam.NonlinearFactorGraph();initial=gtsam.Values();keys={endpoint:k for k,endpoint in enumerate(by_id)}
    factors=[];known=[]
    for endpoint,row in by_id.items():
        initial.insert(keys[endpoint],gtsam.Pose3(alignment[endpoint[0]]@pose(row['T_world_body'])))
    sigmas=np.array([np.deg2rad(cfg['odometry_rotation_sigma_deg'])]*3+[cfg['odometry_translation_sigma_m']]*3)
    odom_noise=gtsam.noiseModel.Diagonal.Sigmas(sigmas)
    by_robot=defaultdict(list)
    for row in rows:by_robot[row['robot_id']].append(row)
    for robot,track in by_robot.items():
        for a,b in zip(track,track[1:]):
            i=(robot,a['keyframe_id']);j=(robot,b['keyframe_id'])
            measured=inv(pose(a['T_world_body']))@pose(b['T_world_body'])
            known.append(graph.size());graph.add(gtsam.BetweenFactorPose3(keys[i],keys[j],gtsam.Pose3(measured),odom_noise))
            factors.append(dict(kind='odometry',i=list(i),j=list(j),T_i_j=measured.tolist(),
                noise_model='independent configured rotation/translation sigma; filter marginals unused',sigmas=sigmas.tolist()))
    # Recomputed for each frozen constraint set: one gauge prior per current component.
    for component in sorted(set(components.values())):
        endpoint=next(e for e in keys if e[0]==component)
        known.append(graph.size());graph.add(gtsam.PriorFactorPose3(keys[endpoint],initial.atPose3(keys[endpoint]),
            gtsam.noiseModel.Diagonal.Sigmas(np.full(6,1e-6))))
        factors.append(dict(kind='anchor',i=list(endpoint),component=component))
    for edge in loops:
        i,j=tuple(edge['i']),tuple(edge['j'])
        if components[i[0]]!=components[j[0]]:
            rejected.append(dict(reason='insufficient_alignment_support',i=list(i),j=list(j)));continue
        graph.add(gtsam.BetweenFactorPose3(keys[i],keys[j],gtsam.Pose3(pose(edge['T_i_j'])),
                    gtsam.noiseModel.Gaussian.Information(np.asarray(edge['information']))))
        factors.append(edge)
    original_graph=graph;initial_components=dict(components);gnc_result=None
    params=gtsam.GncLMParams();params.setLossType(gtsam.GncLossType.TLS);params.setKnownInliers(known)
    optimizer=gtsam.GncLMOptimizer(graph,initial,params)
    optimizer.setInlierCostThresholdsAtProbability(cfg.get('gnc_probability',0.99))
    result=optimizer.optimize();weights=np.asarray(optimizer.getWeights())
    gnc_result=result
    threshold=cfg.get('gnc_inlier_weight_min',.5)
    if not 0<threshold<=1:raise ValueError('GNC inlier weight threshold must be in (0, 1]')
    for index,factor in enumerate(factors):
        factor['gnc_weight']=float(weights[index])
        factor['gnc_squared_whitened_residual']=float(graph.at(index).error(gnc_result)*2)
    inliers=[f for f in factors if f['kind']=='loop' and f['gnc_weight']>=threshold]
    components=active_components(by_robot,inliers)
    # GNC can suppress every bridge between two robots. Refit its selected
    # inliers with independent gauges for the resulting components; otherwise
    # rejected bridges still determine reported connectivity and alignment.
    refit=gtsam.NonlinearFactorGraph();selected=[];omitted=[];known=[]
    for index,factor in enumerate(factors):
        if factor['kind']=='odometry':
            known.append(refit.size());refit.add(graph.at(index));selected.append(factor)
    for component in sorted(set(components.values())):
        endpoint=next(e for e in keys if e[0]==component)
        known.append(refit.size());refit.add(gtsam.PriorFactorPose3(keys[endpoint],gnc_result.atPose3(keys[endpoint]),
            gtsam.noiseModel.Diagonal.Sigmas(np.full(6,1e-6))))
        selected.append(dict(kind='anchor',i=list(endpoint),component=component,gnc_weight=1.,
            anchor_provenance='fixed at the GNC result; rebuilt for selected-inlier connectivity'))
    for index,factor in enumerate(factors):
        if factor['kind']!='loop':continue
        if factor['gnc_weight']>=threshold:refit.add(graph.at(index));selected.append(factor)
        else:omitted.append(factor)
    result=gtsam.LevenbergMarquardtOptimizer(refit,gnc_result).optimize()
    graph=refit;factors=selected+omitted
    weights=np.array([f['gnc_weight'] for f in factors])
    corrected=[]
    for endpoint,row in by_id.items():
        corrected.append(dict(robot_id=endpoint[0],keyframe_id=endpoint[1],stamp_ns=row['stamp_ns'],
            component=components[endpoint[0]],T_world_body=result.atPose3(keys[endpoint]).matrix().tolist(),
            T_initial_body=initial.atPose3(keys[endpoint]).matrix().tolist()))
        if gnc_result is not None:corrected[-1]['T_gnc_body']=gnc_result.atPose3(keys[endpoint]).matrix().tolist()
    for index,factor in enumerate(factors):
        factor['factor_index']=index;factor['robust_weight']=float(weights[index]);factor['known_inlier']=index in known
        factor['solver_weight']=1. if index<graph.size() else 0.
        if index<graph.size():error_factor=graph.at(index)
        else:
            error_factor=gtsam.BetweenFactorPose3(keys[tuple(factor['i'])],keys[tuple(factor['j'])],
                gtsam.Pose3(pose(factor['T_i_j'])),gtsam.noiseModel.Gaussian.Information(np.asarray(factor['information'])))
        factor['squared_whitened_residual']=float(error_factor.error(result)*2)
        if 'j' in factor:
            predicted=result.atPose3(keys[tuple(factor['i'])]).between(result.atPose3(keys[tuple(factor['j'])]))
            factor['residual_rotation_translation']=gtsam.Pose3.Logmap(gtsam.Pose3(pose(factor['T_i_j'])).between(predicted)).tolist()
            factor['residual_has_shared_gauge']=components[factor['i'][0]]==components[factor['j'][0]]
    return dict(poses=corrected,factors=factors,rejected=rejected,components=components,
        initial_components=initial_components,alignment=alignment_report,
        optimization_method='GNC-TLS followed by selected-inlier LM refit and rebuilt component anchors',
        gnc_inlier_weight_min=cfg.get('gnc_inlier_weight_min',.5),
        initial_error=float(original_graph.error(initial)),final_unrobust_error=float(original_graph.error(result)),
        final_solver_error=float(graph.error(result)),gtsam_version='4.2',tangent_order=TANGENT_ORDER)
