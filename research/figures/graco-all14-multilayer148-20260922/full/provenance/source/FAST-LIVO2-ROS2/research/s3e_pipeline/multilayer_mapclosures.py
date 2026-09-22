"""Terrain-relative ORB packets and peer-local HBST retrieval for joint consensus."""
import numpy as np
from .backends import pack_array, unpack_array
from .multilayer_bev import BANDS, fit_terrain, relative_heights, layer_masks
from .joint_bev_ransac import pool_correspondences, fit_consensus
from .joint_bev_verification import lift_planar_pose

NAMES = tuple(b[0] for b in BANDS)
VERSION = 'terrain-relative-orb-v1'


def describe_layers(engine, points, surface, ground):
    """A failed terrain fit yields explicit unavailable layers, never a guessed height."""
    ground = np.asarray(ground, dtype=float)
    level = lambda p: np.asarray(p) @ ground[:3, :3].T + ground[:3, 3]
    empty = dict(ground=ground, xy=np.empty((0, 2)), bits=np.empty((0, 32), dtype=np.uint8))
    packets = {name: empty for name in NAMES}
    counts = dict.fromkeys(NAMES, 0)
    try:
        coefficients, terrain, _, _ = fit_terrain(level(points))
    except ValueError as error:
        terrain = dict(available=False, reason=str(error), ground_truth_used=False)
    else:
        terrain['available'] = True
        for name, mask in layer_masks(relative_heights(level(surface), coefficients)).items():
            counts[name] = int(mask.sum())
            if counts[name] >= 30:
                packets[name] = engine.describe(surface[mask], ground=ground)
    return dict(version=VERSION, terrain=terrain, surface_points=counts,
                feature_counts={name: len(p['xy']) for name, p in packets.items()},
                layers={name: {k: pack_array(p[k]) for k in ('ground', 'xy', 'bits')}
                        for name, p in packets.items()})


def decode_layers(descriptor):
    data = descriptor['mapclosures_multilayer']
    if data['version'] != VERSION or set(data['layers']) != set(NAMES):
        raise ValueError('Incompatible multilayer descriptor')
    ground = unpack_array(descriptor['mapclosures']['ground'])
    packets = {name: {k: unpack_array(v) for k, v in data['layers'][name].items()} for name in NAMES}
    for packet in packets.values():
        if not np.array_equal(packet['ground'], ground):
            raise ValueError('BEV layers must share the submap gravity frame')
        if not data['terrain']['available'] and len(packet['xy']):
            raise ValueError('Unavailable terrain cannot supply layered features')
    return packets


class MultilayerMatcher:
    def __init__(self, options):
        import s3e_mapclosures_native as native
        self.options = options
        self.engines = {name: native.MapClosures(options['density_map_resolution'],
            options['density_threshold'], options['hamming_distance_threshold']) for name in NAMES}
        if not all(hasattr(e, 'query_correspondences') for e in self.engines.values()):
            raise ValueError('Multilayer retrieval requires the raw-correspondence native adapter')
        self.grounds = {}

    def add(self, key, descriptor):
        packets = decode_layers(descriptor)
        for name in NAMES:
            self.engines[name].add(key, packets[name])
        self.grounds[key] = packets[NAMES[0]]['ground']

    def query(self, descriptor, keys):
        packets = decode_layers(descriptor)
        candidates = {}
        for name in NAMES:
            for packet in self.engines[name].query_correspondences(packets[name], keys):
                candidates.setdefault(packet['keyframe_id'], {})[name] = packet
        # Rank pooled distinct matches before bounded joint fitting. Full-height
        # features remain an inspection control and never enter this pool.
        pools = {key: pool_correspondences(parts, self.options['density_map_resolution'])
                 for key, parts in candidates.items()}
        ranked = sorted(pools, key=lambda key: (-len(pools[key]['hamming']), key))
        out = []
        for key in ranked[:self.options['max_hypotheses_per_query']]:
            h = fit_consensus(pools[key], {'inliers_threshold': self.options['inliers_threshold']})
            h.update(keyframe_id=key, method='multilayer HBST + joint SE(2) consensus')
            if h['valid_pose']:
                h['T_i_j'] = lift_planar_pose(h['T_query_candidate_level_2d'],
                    packets[NAMES[0]]['ground'], self.grounds[key]).tolist()
            out.append(h)
        return out

    def pair(self, query, candidate):
        local = MultilayerMatcher(self.options)
        local.add(0, candidate)
        hypotheses = local.query(query, [0])
        return hypotheses[0] if hypotheses else dict(keyframe_id=0, matches=0, inliers=0, valid_pose=False)
