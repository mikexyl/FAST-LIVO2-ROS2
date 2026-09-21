"""Launch independent CBS/ROS robot processes and collect their saved outputs."""
import os
import math
from pathlib import Path
from .frontends import frontend_name
import signal
import subprocess
import sys
import time
import uuid
from .artifacts import digest, file_hash, read_json, read_jsonl, write_json, write_jsonl
from .mixed_pgo import settings as registration_settings


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
    adapter = source/'FAST-LIVO2-ROS2/research/adapters/gtsam_points'
    result['registration_adapter_sources'] = {str(p.relative_to(adapter)):file_hash(p)
        for p in sorted(adapter.rglob('*')) if p.is_file()}
    result['registration_upstream_commit'] = '9d32e7dbecf6015560d84b4901d6b0a6f483ec46'
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
    frozen = read_jsonl(Path(artifacts[f'loops.{frontend_name(cfg)}.{method}'])/'constraints.jsonl') if mode == 'frozen' else []
    pcm = dict(settings.get('pcm', {}))
    pcm_enabled = pcm.get('enabled', False)
    if type(pcm_enabled) is not bool: raise ValueError('dpgo.pcm.enabled must be boolean')
    if set(pcm) - {'enabled', 'probability', 'minimum_clique_size', 'timeout_s'}:
        raise ValueError('Unknown dpgo.pcm option')
    if pcm_enabled:
        probability, minimum, timeout = pcm.get('probability', .99), pcm.get('minimum_clique_size', 2), pcm.get('timeout_s', 60.)
        if type(probability) not in (int, float) or not 0 < probability < 1:
            raise ValueError('PCM probability must be finite and strictly between 0 and 1')
        if type(minimum) is not int or minimum < 1:
            raise ValueError('PCM minimum_clique_size must be a positive integer')
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('PCM timeout_s must be finite and positive')
    pcm_session = uuid.uuid4().hex if pcm_enabled else ''
    registration = registration_settings(settings.get('registration_factors', {}))
    if registration['enabled'] and not pcm_enabled:
        raise ValueError('CBS registration factors require the immutable PCM gate')
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
            if any(k.startswith('pcm_') for k in settings.get('cbs_parameters', {})):
                raise ValueError('Configure PCM through dpgo.pcm, not cbs_parameters')
            params.update(pcm_enabled=pcm_enabled)
            if any(k.startswith('registration_') for k in settings.get('cbs_parameters', {})):
                raise ValueError('Configure registration through dpgo.registration_factors')
            params.update(registration_enabled=registration['enabled'])
            if registration['enabled']:
                import json
                params.update(registration_directory=str(local/'registration'),
                              registration_settings=json.dumps(registration, sort_keys=True))
            if pcm_enabled:
                params.update(pcm_session_id=pcm_session, pcm_probability=float(pcm.get('probability', .99)),
                    pcm_minimum_clique_size=pcm.get('minimum_clique_size', 2),
                    pcm_timeout_sec=float(pcm.get('timeout_s', 60.)))
            import yaml
            param_path = local/'cbs.yaml'
            param_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
            launch(f'{robot}-cbs', [str(native), '--ros-args', '-r', f'__ns:=/{robot}/cbs',
                                  '--params-file', str(param_path)])
            incident = [e for e in frozen if robot in (e['i'][0], e['j'][0])]
            write_jsonl(local/'input-constraints.jsonl', incident)
            spec = dict(robot=robot, robots=robots, mode=mode, output=str(local),
                store=str(Path(artifacts[f'keyframes.{frontend_name(cfg)}.{robot}'])/'store'),
                descriptors=artifacts.get(f'descriptors.{frontend_name(cfg)}.{method}.{robot}',
                                          str(Path(artifacts[f'keyframes.{frontend_name(cfg)}.{robot}'])/'store')), backend=cfg['backend'],
                loops=cfg['loops'], pgo=cfg['pgo'], constraints=str(local/'input-constraints.jsonl'),
                pcm_enabled=pcm_enabled, pcm_session_id=pcm_session,
                registration_factors=registration,
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
    poses = []; unique = {}; proposed = {}; summaries = {}; pcm_decisions = {}
    for robot in robots:
        summaries[robot] = read_json(out/robot/'summary.json')
        poses += read_jsonl(out/robot/'poses.jsonl')
        for edge in read_jsonl(out/robot/'constraints.jsonl'):
            key = (tuple(edge['i']), tuple(edge['j']))
            if key in unique and digest(unique[key]) != digest(edge):
                raise ValueError('Loop endpoints disagree on final constraint')
            unique[key] = edge
        for edge in read_jsonl(out/robot/'proposed-constraints.jsonl'):
            key = (tuple(edge['i']), tuple(edge['j']))
            if key in proposed and digest(proposed[key]) != digest(edge):
                raise ValueError('Loop endpoints disagree on proposed constraint')
            proposed[key] = edge
        if pcm_enabled:
            for decision in read_json(out/robot/'pcm.json')['verdicts']:
                key = (decision['robot_from'], decision['key_from'], decision['robot_to'], decision['key_to'])
                if key in pcm_decisions and pcm_decisions[key] != decision:
                    raise ValueError('PCM endpoint decisions disagree')
                pcm_decisions[key] = decision
    constraints = [unique[k] for k in sorted(unique)]
    if registration['enabled']:
        pairs = []; expected = {(tuple(e['i']), tuple(e['j'])) for e in unique.values()}
        for robot in robots:
            result = summaries[robot]['registration']
            if not result or not result['final'] or not result['success']: raise ValueError('Incomplete native registration result')
            for pair in result['pairs']:
                i, j = pair['i'], pair['j']
                if i[0] != robots.index(robot): raise ValueError('Registration factor assigned to wrong owner')
                pairs.append(((robots[i[0]], i[1]), (robots[j[0]], j[1])))
        if len(set(pairs)) != len(pairs) or set(pairs) != expected:
            raise ValueError('Native registration pair coverage/ownership mismatch')
        write_json(out/'registration.json', dict(settings=registration,
            robots={r:summaries[r]['registration'] for r in robots},
            registration_factor_count=sum(summaries[r]['registration']['registration_factor_count'] for r in robots),
            network_cdr_bytes=sum(s['registration_network_cdr_bytes'] for s in summaries.values()),
            ownership='one binary GICP factor per geometrically supported PCM-retained loop, on its canonical first robot'))
        # Exchanges are reproducible from original NPZs + the retained manifests.
        # Do not retain a second copy of the temporary transport clouds.
        if not settings.get('retain_registration_transport', False):
            for robot in robots:
                for payload in (out/robot/'registration').glob('*.bin'): payload.unlink()
    write_jsonl(out/'poses.jsonl', poses); write_jsonl(out/'constraints.jsonl', constraints)
    write_jsonl(out/'proposed-constraints.jsonl', [proposed[k] for k in sorted(proposed)])
    if pcm_enabled:
        write_json(out/'pcm.json', dict(session_id=pcm_session, settings=pcm,
            verdicts=[pcm_decisions[k] for k in sorted(pcm_decisions)],
            proposed_loops=len(proposed), retained_loops=len(constraints),
            excluded_loops=len(proposed)-len(constraints),
            scope='inter-robot measurement PCM; intra-robot loops pass geometric verification unchanged'))
    write_json(out/'summary.json', dict(backend='CBS SE(3)', transport='ROS2 Fast DDS', mode=mode,
        wall_s=time.monotonic()-start, robots=summaries, keyframes=len(poses), loops=len(constraints),
        proposed_loops=len(proposed), pcm_enabled=pcm_enabled,
        registration_enabled=registration['enabled'],
        registration_network_cdr_bytes=sum(s['registration_network_cdr_bytes'] for s in summaries.values()),
        pcm_network_cdr_bytes=sum(s['pcm_network_cdr_bytes'] for s in summaries.values()),
        shared_reference=all(s['reference_available'] for s in summaries.values()),
        # Existing NodeStats counts the client side: requests sent + responses
        # received. Summing both counts each service message exactly once.
        cbs_network_cdr_bytes=sum(s['cbs_last_stats']['bandwidth_sent_bytes']+
            s['cbs_last_stats']['bandwidth_recv_bytes'] for s in summaries.values()),
        loop_network_cdr_bytes=sum(s['loop_network_cdr_bytes'] for s in summaries.values()),
        sampled_peak_rss_kib=peak_rss, memory_sampling_interval_s=.2,
        byte_scope='serialized CDR messages per directed recipient; excludes RTPS, discovery and retransmission',
        termination='fixed local settling budget after peer input completion; convergence reported separately'))
