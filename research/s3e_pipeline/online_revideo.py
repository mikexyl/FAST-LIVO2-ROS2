"""Re-render captured online arrivals without recomputing SLAM or reading GT.

Uses the original recorded frame schedule, including repeated frames. Corrections
are applied at their original display event, never before backend publication.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from .artifacts import read_json, read_jsonl
from .online_record import Recorder
from .online_video import group_video, video_fonts, group_panels
from .robot_colors import robot_colors


class ArrivalReplay(Recorder):
    def __init__(self, work):
        self.work = work
        self.robots = read_json(work/'START')['robots']
        self.colors = robot_colors(self.robots)
        self.fonts = video_fonts()
        self.rows = {r:[] for r in self.robots}
        self.poses = {r:[] for r in self.robots}
        self.chunks = {r:[] for r in self.robots}
        self.ids = {r:set() for r in self.robots}
        self.estimates = {r:{} for r in self.robots}
        self.components = {r:r for r in self.robots}
        self.revision = 0
        self.loops = []
        self.native_rows = {r:read_jsonl(work/f'prepared-{r}/store/keyframes.jsonl') for r in self.robots}

    def consume(self, event):
        kind = event['type']
        if kind == 'poses_received':
            self.poses[event['robot']].extend(event['poses'])
        elif kind == 'map_displayed':
            robot, key = event['robot'], event['keyframe_id']
            assert key == len(self.rows[robot])
            row = self.native_rows[robot][key]
            assert row['available_ns'] <= event['logged_wall_ns']
            path = self.work/f'frontends/{robot}/frontend/area_maps'/row['payload']
            with np.load(path, allow_pickle=False) as payload:
                ids = payload['point_ids']
                mask = np.fromiter((int(i) not in self.ids[robot] for i in ids), dtype=bool, count=len(ids))
                self.ids[robot].update(map(int, ids))
                points = payload['points'][mask]
            if len(points):
                _, selection = np.unique(np.floor(points).astype(np.int32), axis=0, return_index=True)
                points = points[selection].astype(np.float32)
            assert len(points) == event['display_points'], 'Display membership differs from original capture'
            T = np.asarray(row['T_world_body'])
            self.chunks[robot].append((key, row['stamp_ns'], points@T[:3,:3].T+T[:3,3]))
            self.rows[robot].append(row)
        elif kind == 'revision_displayed':
            assert event['published_wall_ns'] <= event['logged_wall_ns']
            self.revision = event['revision']
            epoch = self.work/f'epochs/{self.revision:03d}/dpgo'
            self.estimates = {r:{} for r in self.robots}
            for pose in read_jsonl(epoch/'poses.jsonl'):
                robot = pose['robot_id']
                self.estimates[robot][pose['keyframe_id']] = pose['T_world_body']
                self.components[robot] = pose['component']
            assert self.components == event['components']
            self.loops = read_jsonl(epoch/'constraints.jsonl')
        else:
            raise ValueError(f'Unknown arrival event: {kind}')


def render(work, out, previews_only=False):
    out.mkdir(exist_ok=True)
    replay = ArrivalReplay(work)
    events = read_jsonl(work/'demo/events.jsonl')
    frames = read_jsonl(work/'demo/frames.jsonl')
    assert all(a['logged_wall_ns']<=b['logged_wall_ns'] for a,b in zip(events,events[1:]))
    cursor = 0
    written = 0
    last = None
    saved_groups = set()
    transitions = []
    previous = None
    pipe = None if previews_only else (out/'video.rgb').open('wb', buffering=0)
    try:
        for frame in frames:
            while cursor<len(events) and events[cursor]['logged_wall_ns']<=frame['logged_wall_ns']:
                replay.consume(events[cursor])
                cursor += 1
            assert replay.revision == frame['revision']
            assert len(replay.loops) == frame['loops']
            groups = {g:[r for r in replay.robots if replay.components[r]==g] for g in set(replay.components.values())}
            assert max(map(len, groups.values())) == frame['largest_component']
            layout = tuple(sorted(replay.components.items()))
            if layout != previous:
                transitions.append(dict(elapsed_s=frame['elapsed_s'],frame=frame['frame'],
                    revision=replay.revision,groups=groups,panel_count=len(groups)))
                previous = layout
            # Preview representative populated maps for each actual layout.
            save = len(groups) not in saved_groups and frame['elapsed_s']>20
            if not previews_only or save:
                scene = replay.scene()
                image = group_video(replay, frame['elapsed_s'], scene, replay=True)
                if save:
                    image.save(out/f'groups-{len(groups):02d}.png')
                    saved_groups.add(len(groups))
                if pipe is not None:
                    data = image.tobytes()
                    while written<frame['frame']:
                        assert last is not None
                        pipe.write(last)
                        written += 1
                    pipe.write(data)
                    written += 1
                    last = data
                    if written%100<3:
                        print(f'{written}/{frames[-1]["frame"]+1} frames; {len(groups)} groups',file=sys.stderr,flush=True)
        final_scene = replay.scene()
        group_video(replay, frames[-1]['elapsed_s'], final_scene, replay=True).save(out/'final.png')
        assert len(final_scene[0])==1 and len(group_panels(final_scene[0]))==1
        summary = dict(frames=written,fps=5,duration_s=written/5,transitions=transitions,
            original_frame_schedule=True,all_display_memberships_verified=True,
            backend_revisions_applied_at_original_display_events=True,ground_truth_used=False,
            events_consumed=cursor,total_events=len(events),final_panels=1,
            final_robot_count=len(replay.robots),retained_loops=len(replay.loops),
            source_hashes={str(p.relative_to(work)):hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in [work/'START',work/'demo/events.jsonl',work/'demo/frames.jsonl']})
        assert cursor==len(events)
        (out/('preview-audit.json' if previews_only else 'render-audit.json')).write_text(json.dumps(summary,indent=2)+'\n')
    finally:
        if pipe is not None:
            pipe.close()


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--previews-only',action='store_true')
    args=parser.parse_args()
    render(args.work,args.out,args.previews_only)
