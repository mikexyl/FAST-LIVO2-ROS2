"""Launch independent CBS/ROS robot processes and collect their saved outputs."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from .artifacts import digest, file_hash, read_json, read_jsonl, write_json, write_jsonl


def native_provenance(source):
    result = {}
    for name in ('cbs', 'cbs_ros'):
        root = source/name
        git = lambda *args: subprocess.check_output(['git', '-C', str(root), *args]).decode().strip()
        files = git('ls-files', '--cached', '--others', '--exclude-standard').splitlines()
        result[name] = dict(branch=git('branch', '--show-current'), commit=git('rev-parse', 'HEAD'),
            sources={p: file_hash(root/p) for p in sorted(set(files)) if (root/p).is_file()})
    for relative in ('.ros2/dpgo-install/cbs/lib/libcbs.so',
                     '.ros2/dpgo-install/cbs_ros/lib/cbs_ros/cbs_ros_node',
                     'FAST-LIVO2-ROS2/scripts/run_dpgo_ros.sh'):
        result[relative] = file_hash(source/relative)
    underlay = Path(os.environ.get('CBS_UNDERLAY', '/home/mikexyl/workspaces/sb_slam_ros2_ws/install'))
    result['gtsam_library'] = dict(path=str((underlay/'gtsam/lib/libgtsam.so').resolve()),
                                  sha256=file_hash(underlay/'gtsam/lib/libgtsam.so'))
    return result


def run(cfg, artifacts, source, out):
    robots = cfg['robots']; settings = cfg['dpgo']; method = cfg['backend']['name']
    out = Path(out).resolve(); source = Path(source).resolve(); start = time.monotonic()
    overlay = source/'.ros2/dpgo-install'
    underlay = Path(os.environ.get('CBS_UNDERLAY', '/home/mikexyl/workspaces/sb_slam_ros2_ws/install'))
    wrapper = source/'FAST-LIVO2-ROS2/scripts/run_dpgo_ros.sh'
    native = overlay/'cbs_ros/lib/cbs_ros/cbs_ros_node'
    mode = settings.get('mode', 'peers')
    if mode not in ('peers', 'frozen'): raise ValueError('dpgo.mode must be peers or frozen')
    frozen = read_jsonl(Path(artifacts[f'loops.livo.{method}'])/'constraints.jsonl') if mode == 'frozen' else []
    processes = []; logs = []; observers = {}; peak_rss = {}
    env = dict(os.environ, ROS_DOMAIN_ID=str(settings['ros_domain_id']),
        PYTHONPATH=str(Path(__file__).resolve().parents[1])+os.pathsep+os.environ.get('PYTHONPATH', ''),
        PYTHONDONTWRITEBYTECODE='1', ROS_LOCALHOST_ONLY='1' if settings.get('localhost_only', True) else '0')
    def launch(name, command):
        log = (out/f'{name}.log').open('w'); logs.append(log)
        p = subprocess.Popen(['bash', str(wrapper), *command], env=env, stdout=log, stderr=subprocess.STDOUT,
                             start_new_session=True)
        processes.append((name, p)); return p
    try:
        for index, robot in enumerate(robots):
            local = out/robot; local.mkdir()
            params = dict(robot_id=index, robot_name=robot, num_robots=len(robots), pgo_formulation='pose3',
                pose_graph_topic=f'/{robot}/cbs/pose_graph', enable_visualization=False,
                synchronize_optimization_rounds=False, communication_topology_mode='dynamic_factors',
                max_iterations=-1, settle_iterations=settings['settle_iterations'], loop_rate=settings['loop_rate_hz'],
                belief_stage_switch_strategy='fixed_iteration', belief_stage_fixed_iterations=10,
                online=True, enable_gkcm=False, enable_soft_reset=False, enable_dcs=False,
                use_robust_noise_models=False, log_dir=str(local/'native'))
            params.update(settings.get('cbs_parameters', {}))
            import yaml
            param_path = local/'cbs.yaml'
            param_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
            launch(f'{robot}-cbs', [str(native), '--ros-args', '-r', f'__ns:=/{robot}/cbs',
                                  '--params-file', str(param_path)])
            incident = [e for e in frozen if robot in (e['i'][0], e['j'][0])]
            write_jsonl(local/'input-constraints.jsonl', incident)
            spec = dict(robot=robot, robots=robots, mode=mode, output=str(local),
                store=str(Path(artifacts[f'keyframes.livo.{robot}'])/'store'),
                descriptors=artifacts[f'descriptors.livo.{method}.{robot}'], backend=cfg['backend'],
                loops=cfg['loops'], pgo=cfg['pgo'], constraints=str(local/'input-constraints.jsonl'),
                ros_underlay=str(underlay), ros_overlay=str(overlay))
            write_json(local/'spec.json', spec)
            observers[robot] = launch(f'{robot}-frontend', [sys.executable, '-m', 's3e_pipeline.ros_dpgo_worker',
                                                          '--spec', str(local/'spec.json')])
        deadline = start+settings['timeout_s']; last_progress = 0
        while any(p.poll() is None for p in observers.values()):
            for name, p in processes:
                if p.poll() is not None and (p.returncode != 0 or name.endswith('-cbs')):
                    raise RuntimeError(f'{name} exited {p.returncode}; inspect {out/name}.log')
                try:
                    status_text = Path(f'/proc/{p.pid}/status').read_text()
                    rss = next(int(line.split()[1]) for line in status_text.splitlines() if line.startswith('VmRSS:'))
                    peak_rss[name] = max(peak_rss.get(name, 0), rss)
                except (FileNotFoundError, ProcessLookupError, StopIteration): pass
            now = time.monotonic()
            if now > deadline: raise TimeoutError(f'CBS run exceeded {settings["timeout_s"]} seconds')
            if now-last_progress > 10:
                status = {r: read_json(out/r/'progress.json') for r in robots if (out/r/'progress.json').exists()}
                write_json(out/'progress.json', dict(wall_s=now-start, robots=status))
                print(f'CBS/DDS {now-start:.1f}s: '+', '.join(f'{r} {s["observed"]}/{s["total"]} keys, '
                    f'{s["loops"]} incident loops, iteration {s["iteration"]}' for r, s in status.items()), flush=True)
                last_progress = now
            time.sleep(.2)
    finally:
        for _, p in processes:
            if p.poll() is None: os.killpg(p.pid, signal.SIGINT)
        for _, p in processes:
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL); p.wait()
        for log in logs: log.close()
    poses = []; unique = {}; summaries = {}
    for robot in robots:
        summaries[robot] = read_json(out/robot/'summary.json')
        poses += read_jsonl(out/robot/'poses.jsonl')
        for edge in read_jsonl(out/robot/'constraints.jsonl'):
            key = (tuple(edge['i']), tuple(edge['j']))
            if key in unique and digest(unique[key]) != digest(edge):
                raise ValueError('Loop endpoints disagree on final constraint')
            unique[key] = edge
    constraints = [unique[k] for k in sorted(unique)]
    write_jsonl(out/'poses.jsonl', poses); write_jsonl(out/'constraints.jsonl', constraints)
    write_json(out/'summary.json', dict(backend='CBS SE(3)', transport='ROS2 Fast DDS', mode=mode,
        wall_s=time.monotonic()-start, robots=summaries, keyframes=len(poses), loops=len(constraints),
        shared_reference=all(s['reference_available'] for s in summaries.values()),
        # Existing NodeStats counts the client side: requests sent + responses
        # received. Summing both counts each service message exactly once.
        cbs_network_cdr_bytes=sum(s['cbs_last_stats']['bandwidth_sent_bytes']+
            s['cbs_last_stats']['bandwidth_recv_bytes'] for s in summaries.values()),
        loop_network_cdr_bytes=sum(s['loop_network_cdr_bytes'] for s in summaries.values()),
        sampled_peak_rss_kib=peak_rss, memory_sampling_interval_s=.2,
        byte_scope='serialized CDR messages per directed recipient; excludes RTPS, discovery and retransmission',
        termination='fixed local settling budget after peer input completion; convergence reported separately'))
