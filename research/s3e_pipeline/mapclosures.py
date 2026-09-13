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
        self.options=cfg['mapclosures'];self.visual_min=cfg['fusion']['visual_min_similarity']
        if not 0<self.visual_min<1 or self.options['inliers_threshold']<3:
            raise ValueError('Explicit retrieval thresholds are required')
        self.engine=native.MapClosures(self.options['density_map_resolution'],
            self.options['density_threshold'],self.options['hamming_distance_threshold'])
        self.cached={}

    def describe(self,row,cloud,image):
        xyz=np.asarray(cloud[:,:3],dtype=np.float64)
        xyz=xyz[np.isfinite(xyz).all(1)&(np.linalg.norm(xyz,axis=1)<=self.options['max_range_m'])]
        result=self.engine.describe(xyz)
        return dict(mapclosures={k:pack_array(result[k]) for k in ('ground','xy','bits')},
                    mapclosures_features=len(result['xy']))

    @staticmethod
    def decode(desc):
        vector=unpack_array(desc['visual']).astype(np.float64).ravel()
        if not np.isfinite(vector).all() or abs(np.linalg.norm(vector)-1)>.001:
            raise ValueError('Invalid normalized MegaLoc vector')
        return vector,{k:unpack_array(v) for k,v in desc['mapclosures'].items()}

    def native_pass(self,h):
        return h['valid_pose'] and h['inliers']>self.options['inliers_threshold']

    def retrieve(self,query,index,top_k):
        if not index:return []
        qv,qf=self.decode(query)
        for key in sorted(index):
            if key not in self.cached:
                self.cached[key]=self.decode(index[key]);self.engine.add(key,self.cached[key][1])
        keys=sorted(index)
        scores=np.stack([self.cached[k][0] for k in keys])@qv
        visual_scores=dict(zip(keys,map(float,scores)))
        ranked={}
        def item(key):
            if key not in ranked:
                visual=visual_scores[key]
                ranked[key]=dict(keyframe_id=key,visual_similarity=visual,score=max(0.,visual),
                    eligible=False,sources=[],branch_scores={},rejection_reason='retrieval_threshold')
            return ranked[key]
        for key in sorted(keys,key=lambda k:(-visual_scores[k],k))[:top_k]:
            r=item(key)
            if r['visual_similarity']>=self.visual_min:
                r['sources'].append('megaloc');r['branch_scores']['megaloc']=r['visual_similarity'];r['eligible']=True
        native=self.engine.query(qf,keys,self.options['max_hypotheses_per_query'])
        for h in native:
            h=native_result(h);r=item(h['keyframe_id']);r['mapclosures_hypothesis']=h
            if self.native_pass(h):
                r['sources'].append('mapclosures');r['branch_scores']['mapclosures']=h['inliers'];r['eligible']=True
                # Display-only rank fusion. Branch selection uses its own raw score.
                r['score']=max(r['score'],h['inliers']/(h['inliers']+self.options['inliers_threshold']))
        return sorted(ranked.values(),key=lambda r:(-r['score'],r['keyframe_id']))

    def verify(self,query,candidate,proposal=None):
        qv,qf=self.decode(query['descriptor']);cv,cf=self.decode(candidate['descriptor'])
        proposal=proposal or {};sources=proposal.get('sources',[])
        h=proposal.get('mapclosures_hypothesis')
        # Preserve the HBST database hypothesis. Visual-only proposals use a
        # two-map native HBST match, without a LiDAR retrieval gate on MegaLoc.
        if h is None or not self.native_pass(h):h=native_result(self.engine.pair(qf,cf))
        diagnostics=dict(backend=self.name,visual_similarity=float(qv@cv),retrieval_sources=sources,
            selected_branches=proposal.get('selected_branches',[]),
            native=dict(method='MapClosures',hypothesis=h),pose_initializer='MapClosures density-map RANSAC')
        if not self.native_pass(h):
            return dict(accepted=False,reason='mapclosures_no_pose',**diagnostics)
        initial=np.asarray(h['T_i_j'])
        result=refine(query['cloud'],candidate['cloud'],initial,self.cfg['registration'],self.geometry_cache)
        result.update(initial_T_i_j=initial.tolist(),**diagnostics)
        return result
