"""Independent MegaLoc and native MapClosures retrieval, with shared GICP acceptance."""
import numpy as np
from .backends import Backend,pack_array,unpack_array
from .registration import refine


def native_result(value):
    return {k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in value.items()}


class MegaLocMapClosures(Backend):
    name='megaloc_mapclosures'

    def __init__(self,cfg):
        super().__init__(cfg)
        import s3e_mapclosures_native as native
        if native.upstream_commit!=cfg['mapclosures']['upstream_commit']:
            raise ValueError('MapClosures binary/source revision mismatch')
        self.visual_enabled=cfg['name']=='megaloc_mapclosures'
        self.name=cfg['name']
        self.options=cfg['mapclosures'];self.visual_min=cfg.get('fusion',{}).get('visual_min_similarity',.5)
        if not 0<self.visual_min<1 or self.options['inliers_threshold']<3:
            raise ValueError('Explicit retrieval thresholds are required')
        self.engine=native.MapClosures(self.options['density_map_resolution'],
            self.options['density_threshold'],self.options['hamming_distance_threshold'])
        self.combined=self.options.get('retrieval_channels','legacy')=='fullheight_and_layers'
        if self.options.get('retrieval_channels','legacy') not in ('legacy','fullheight_and_layers'):
            raise ValueError('Unknown MapClosures retrieval channel policy')
        self.cached={}
        self.multilayer=None
        if self.options.get('multilayer',{}).get('enabled',False):
            from .multilayer_mapclosures import MultilayerMatcher
            self.multilayer=MultilayerMatcher(self.options)

    def describe(self,row,cloud,image):
        xyz=np.asarray(cloud[:,:3],dtype=np.float64)
        xyz=xyz[np.isfinite(xyz).all(1)&(np.linalg.norm(xyz,axis=1)<=self.options['max_range_m'])]
        result=self.engine.describe(xyz)
        return dict(mapclosures={k:pack_array(result[k]) for k in ('ground','xy','bits')},
                    mapclosures_features=len(result['xy']))

    def decode(self,desc):
        expected=self.options.get('representation')
        if expected is not None and desc.get('representation')!=expected:
            raise ValueError('Descriptor BEV representation differs from the configured pipeline')
        if self.cfg.get('ellipsoid_only',False) and (desc.get('representation')!='ellipsoid' or
                desc.get('renderer')!=self.options.get('renderer','sampled_surface')):
            raise ValueError('Descriptor renderer/evidence type mismatch')
        vector=None
        if self.visual_enabled:
            vector=unpack_array(desc['visual']).astype(np.float64).ravel()
            if not np.isfinite(vector).all() or abs(np.linalg.norm(vector)-1)>.001:
                raise ValueError('Invalid normalized MegaLoc vector')
        elif 'visual' in desc:
            raise ValueError('Visual descriptors are not permitted in MapClosures-only mode')
        return vector,{k:unpack_array(v) for k,v in desc['mapclosures'].items()}

    def native_pass(self,h):
        return h['valid_pose'] and h['inliers']>self.options['inliers_threshold']

    def retrieve(self,query,index,top_k):
        if not index:return []
        qv,qf=self.decode(query)
        for key in sorted(index):
            if key not in self.cached:
                self.cached[key]=self.decode(index[key])
                if self.multilayer is not None:self.multilayer.add(key,index[key])
                if self.multilayer is None or self.combined:self.engine.add(key,self.cached[key][1])
        keys=sorted(index)
        visual_scores=dict(zip(keys,map(float,np.stack([self.cached[k][0] for k in keys])@qv))) if self.visual_enabled else dict.fromkeys(keys,0.)
        ranked={}
        def item(key):
            if key not in ranked:
                visual=visual_scores[key]
                ranked[key]=dict(keyframe_id=key,visual_similarity=visual if self.visual_enabled else None,score=max(0.,visual),
                    eligible=False,sources=[],branch_scores={},rejection_reason='retrieval_threshold')
            return ranked[key]
        for key in (sorted(keys,key=lambda k:(-visual_scores[k],k))[:top_k] if self.visual_enabled else []):
            r=item(key)
            if r['visual_similarity']>=self.visual_min:
                r['sources'].append('megaloc');r['branch_scores']['megaloc']=r['visual_similarity'];r['eligible']=True
        channels={}
        if self.multilayer is not None:channels['layers']=self.multilayer.query(query,keys)
        if self.multilayer is None or self.combined:
            channels['fullheight']=self.engine.query(qf,keys,self.options['max_hypotheses_per_query'])
        for channel,native in channels.items():
            for h in native:
                h=native_result(h);h['channel']=channel;r=item(h['keyframe_id'])
                r.setdefault('mapclosures_hypotheses',[]).append(h)
                previous=r.get('mapclosures_hypothesis')
                if previous is None or h['inliers']>previous['inliers']:r['mapclosures_hypothesis']=h
                if self.native_pass(h):
                    if 'mapclosures' not in r['sources']:r['sources'].append('mapclosures')
                    r['branch_scores']['mapclosures']=max(r['branch_scores'].get('mapclosures',0),h['inliers'])
                    if self.combined:r.setdefault('channel_scores',{})[channel]=h['inliers']
                    r['eligible']=True
                    r['score']=max(r['score'],h['inliers']/(h['inliers']+self.options['inliers_threshold']))
        return sorted(ranked.values(),key=lambda r:(-r['score'],r['keyframe_id']))

    def verify(self,query,candidate,proposal=None):
        proposal=proposal or {}
        hypotheses=proposal.get('mapclosures_hypotheses',[])
        if not self.combined:return self._verify_once(query,candidate,proposal)
        if not hypotheses:
            _,qf=self.decode(query['descriptor']);_,cf=self.decode(candidate['descriptor'])
            hypotheses=[dict(native_result(self.engine.pair(qf,cf)),channel='fullheight')]
            if self.multilayer is not None:
                hypotheses.append(dict(self.multilayer.pair(query['descriptor'],candidate['descriptor']),channel='layers'))
        eligible=sorted((h for h in hypotheses if self.native_pass(h)),key=lambda h:(-h['inliers'],h['channel']))
        if not eligible:return self._verify_once(query,candidate,proposal)
        attempts=[]
        for h in eligible:
            result=self._verify_once(query,candidate,dict(proposal,mapclosures_hypothesis=h))
            attempts.append(dict(channel=h['channel'],inliers=h['inliers'],accepted=result['accepted'],reason=result['reason']))
            if result['accepted']:break
        result['initialization_attempts']=attempts
        result['retrieval_channels']=[h['channel'] for h in hypotheses]
        return result

    def _verify_once(self,query,candidate,proposal=None):
        qv,qf=self.decode(query['descriptor']);cv,cf=self.decode(candidate['descriptor'])
        proposal=proposal or {};sources=proposal.get('sources',[])
        h=proposal.get('mapclosures_hypothesis')
        # Preserve the HBST database hypothesis. Visual-only proposals use a
        # two-map native HBST match, without a LiDAR retrieval gate on MegaLoc.
        if h is None or not self.native_pass(h):
            h=(self.multilayer.pair(query['descriptor'],candidate['descriptor']) if self.multilayer is not None else
               native_result(self.engine.pair(qf,cf)))
        diagnostics=dict(backend=self.name,visual_similarity=float(qv@cv) if self.visual_enabled else None,retrieval_sources=sources,
            selected_branches=proposal.get('selected_branches',[]),
            native=dict(method='MapClosures',hypothesis=h),pose_initializer='MapClosures density-map RANSAC')
        if self.multilayer is not None and h.get('channel','layers')=='layers':
            diagnostics['native']['method']='MapClosures multilayer HBST + joint SE(2) consensus'
            diagnostics['pose_initializer']='Multilayer joint SE(2) consensus'
        for name,payload in [('query',query),('candidate',candidate)]:
            if 'evidence_preprocessing' in payload:
                diagnostics[name+'_evidence']=payload['evidence_preprocessing']
        if self.cfg.get('ellipsoid_only',False):
            from .ellipsoid_backend import prepared_evidence
            for payload in (query,candidate):
                prepared_evidence(payload,payload['row'],self.cfg['registration'])
                if len(payload['cloud']):raise ValueError('Ellipsoid-only evidence contains point geometry')
            diagnostics['ellipsoid_endpoints']=[dict(robot=p['row']['robot_id'],key=p['row']['keyframe_id'],
                stamp_ns=p['row']['stamp_ns'],cloud_frame=p['row']['cloud_frame'],
                payload_sha256=p['row']['ellipsoid_evidence_sha256']) for p in (query,candidate)]
        method=self.cfg['registration'].get('method','point_gicp')
        if method not in ('point_gicp','ellipsoid'):
            raise ValueError('Unknown loop registration method')
        if method=='point_gicp' and self.cfg['registration'].get('sampling')=='fixed':
            resolution=self.cfg['registration']['voxel_m']
            for payload in (query,candidate):
                info=payload.get('evidence_preprocessing')
                if info is None or info['effective_voxel_m']>resolution+1e-9:
                    raise ValueError('Fixed-resolution verification requires evidence at the requested resolution or finer')
        if not self.native_pass(h):
            return dict(accepted=False,reason='mapclosures_no_pose',**diagnostics)
        initial=np.asarray(h['T_i_j'])
        vertical=self.options.get('vertical_initialization',{})
        if vertical.get('enabled',False):
            for payload in (query,candidate):
                if payload['descriptor'].get('projection_alignment',{}).get('projection')!='orthographic gravity-horizontal':
                    raise ValueError('Vertical initialization requires explicit gravity-horizontal descriptors')
            from .vertical_initialization import initialize_vertical
            diagnostics['bev_initial_T_i_j']=initial.tolist()
            geometry=(lambda p:p['ellipsoids'][:,:3]) if method=='ellipsoid' else (lambda p:p['cloud'])
            initial,height=initialize_vertical(geometry(query),geometry(candidate),initial,qf['ground'],cf['ground'],vertical,primitive_centers=self.cfg.get('ellipsoid_only',False))
            diagnostics.update(vertical_initialization=height,
                               pose_initializer=diagnostics['pose_initializer']+' plus geometric vertical initialization')
        if method=='ellipsoid':
            from .ellipsoid_registration import refine_ellipsoids
            if 'ellipsoids' not in query or 'ellipsoids' not in candidate:
                raise ValueError('Missing native ellipsoid verification evidence')
            result=refine_ellipsoids(query['ellipsoids'],candidate['ellipsoids'],initial,self.cfg['registration'])
        else:
            result=refine(query['cloud'],candidate['cloud'],initial,self.cfg['registration'],self.geometry_cache)
        result.update(initial_T_i_j=initial.tolist(),**diagnostics)
        return result
