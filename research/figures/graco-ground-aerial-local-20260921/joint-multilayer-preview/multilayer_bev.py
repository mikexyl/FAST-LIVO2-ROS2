"""Independent terrain-relative height slicing for an offline BEV preview.

The ground model is a supported low-surface plane, not a semantic ground label.
It never receives another robot's data, a loop pose, or a flight-height prior.
"""
import numpy as np


BANDS = (
    ('near', 'Near terrain: −0.5–2 m', -.5, 2.),
    ('low', 'Low structures: 2–5 m', 2., 5.),
    ('middle', 'Middle structures: 5–10 m', 5., 10.),
    ('high', 'High structures: 10–20 m', 10., 20.),
    ('upper', 'Upper structures: ≥20 m', 20., None),
)
TERRAIN_SETTINGS = dict(cell_m=4., minimum_cell_points=20, quantile=.10,
    trials=600, seed=0, tolerance_m=.35, max_slope_deg=12., minimum_support=30,
    minimum_fraction=.15, minimum_minor_spread_m=8.)


def fit_terrain(points, settings=None):
    """Fit z=a*x+b*y+c to low quantiles with equal spatial-cell weighting.

    Uniform spatial weighting prevents a dense tree/roof patch from dominating
    solely through its point count. Confidence describes plane support; it does
    not prove the fitted surface is terrain when the real ground is occluded.
    """
    cfg=dict(TERRAIN_SETTINGS)
    if settings: cfg.update(settings)
    p=np.asarray(points,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all():
        raise ValueError('Terrain input must be finite Nx3 geometry')
    cells=np.floor(p[:,:2]/cfg['cell_m']).astype(np.int64)
    _,inverse=np.unique(cells,axis=0,return_inverse=True)
    order=np.argsort(inverse,kind='stable')
    cuts=np.r_[0,np.flatnonzero(np.diff(inverse[order]))+1,len(order)]
    lower=[]
    for a,b in zip(cuts[:-1],cuts[1:]):
        if b-a<cfg['minimum_cell_points']: continue
        patch=p[order[a:b]]
        z=np.quantile(patch[:,2],cfg['quantile'])
        chosen=patch[patch[:,2]<=z]
        lower.append([*np.mean(chosen[:,:2],axis=0),z])
    lower=np.asarray(lower,dtype=float).reshape(-1,3)
    if len(lower)<cfg['minimum_support']:
        raise ValueError('Insufficient spatial support for terrain estimate')
    X=np.c_[lower[:,:2],np.ones(len(lower))];z=lower[:,2]
    # Only low surfaces propose hypotheses; all cells assess their support.
    proposal=np.flatnonzero(z<=np.quantile(z,.75))
    rng=np.random.default_rng(cfg['seed']);best=None;best_score=(-1,-np.inf)
    for _ in range(cfg['trials']):
        ids=rng.choice(proposal,3,replace=False)
        if abs(np.linalg.det(X[ids]))<1.: continue
        coefficients=np.linalg.solve(X[ids],z[ids])
        if np.linalg.norm(coefficients[:2])>np.tan(np.deg2rad(cfg['max_slope_deg'])): continue
        residual=np.abs(z-X@coefficients);mask=residual<cfg['tolerance_m']
        score=(int(mask.sum()),-float(np.median(residual[mask])) if mask.any() else -np.inf)
        if score>best_score: best,best_score=mask,score
    if best is None or best.sum()<cfg['minimum_support']:
        raise ValueError('No supported low-surface plane')
    for _ in range(4):
        coefficients=np.linalg.lstsq(X[best],z[best],rcond=None)[0]
        best=np.abs(z-X@coefficients)<cfg['tolerance_m']
        if best.sum()<cfg['minimum_support']: raise ValueError('Unstable terrain support')
    coefficients=np.linalg.lstsq(X[best],z[best],rcond=None)[0]
    spread=np.sqrt(np.linalg.eigvalsh(np.cov(lower[best,:2].T)))
    residual=z-X@coefficients
    supported=(best.mean()>=cfg['minimum_fraction'] and spread[0]>=cfg['minimum_minor_spread_m']
        and np.linalg.norm(coefficients[:2])<=np.tan(np.deg2rad(cfg['max_slope_deg'])))
    if not supported: raise ValueError('Terrain plane support is narrow, sparse, or too steep')
    diagnostic=dict(method='spatially balanced low-quantile plane fit',settings=cfg,
        coefficients_z_equals_ax_by_c=coefficients.tolist(),candidate_cells=len(lower),
        support_cells=int(best.sum()),support_fraction=float(best.mean()),
        support_minor_major_spread_m=spread.tolist(),support_rmse_m=float(np.sqrt(np.mean(residual[best]**2))),
        sensor_height_above_fitted_plane_m=float(-coefficients[2]),
        slope_deg=float(np.degrees(np.arctan(np.linalg.norm(coefficients[:2])))),
        ground_truth_used=False,independent_per_submap=True,
        limitation='Dominant low surface may be a roof or another level if true terrain is not visible; planar terrain approximation.')
    return coefficients,diagnostic,lower,best


def relative_heights(level_points, coefficients):
    p=np.asarray(level_points,dtype=float);a,b,c=np.asarray(coefficients,dtype=float)
    return p[:,2]-(a*p[:,0]+b*p[:,1]+c)


def layer_masks(heights):
    h=np.asarray(heights,dtype=float)
    if not np.isfinite(h).all(): raise ValueError('Non-finite layer height')
    return {name:(h>=lo)&(True if hi is None else h<hi) for name,_,lo,hi in BANDS}
