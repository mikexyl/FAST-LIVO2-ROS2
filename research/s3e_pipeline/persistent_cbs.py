"""Persistent CBS+ process lifecycle, independent of frontend capture/retrieval."""
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
import yaml
from .artifacts import digest, file_hash, read_json, write_jsonl
from .online_io import atomic_json
from .mixed_pgo import settings as registration_settings
from .dpgo import native_overlay


def parameters(config):
    cfg = copy.deepcopy(config['dpgo'])
    if not cfg.get('pcm', {}).get('enabled', False):
        raise ValueError('Persistent CBS requires distributed PCM')
    rate = cfg.get('persistent_loop_rate_hz', 1.)
    settle = cfg.get('persistent_settle_iterations', 10)
    if not math.isfinite(rate) or rate <= 0 or type(settle) is not int or settle < 1:
        raise ValueError('Invalid persistent iteration rate or final settling count')
    custom = dict(cfg.get('cbs_parameters', {}))
    reserved = {'persistent','loop_rate','max_iterations','settle_iterations','online',
                'synchronize_optimization_rounds','communication_topology_mode',
                'belief_stage_switch_strategy','enable_soft_reset','enable_dcs','enable_gkcm'}
    if reserved & custom.keys() or any(k.startswith(('pcm_','registration_')) for k in custom):
        raise ValueError('Persistent lifecycle/PCM/registration parameters cannot be overridden through cbs_parameters')
    custom.update(persistent=True,loop_rate=float(rate),settle_iterations=settle,max_iterations=-1,
        online=True,synchronize_optimization_rounds=False,communication_topology_mode='dynamic_factors',
        belief_stage_switch_strategy='random',enable_soft_reset=False,enable_dcs=False,enable_gkcm=False)
    return custom


class PersistentSession:
    """One native optimizer and one isolated geometry bridge per robot."""
    def __init__(self, config, source, output, stores):
        self.config = config; self.robots = config['robots']; self.source = Path(source)
        self.output = Path(output); self.output.mkdir(parents=True,exist_ok=False)
        self.session = uuid.uuid4().hex; self.revision = 0; self.final = False
        self.processes = []; self.logs = []; self.peak_rss_kib = 0
        self.started = time.monotonic(); self.submitted = None; self.last_rows = {}; self.last_edges = []
        settings = config['dpgo']; native_params = parameters(config)
        registration = registration_settings(settings.get('registration_factors', {}))
        if registration['enabled']:
            is_ellipsoid = config['backend'].get('registration',{}).get('method','point_gicp') == 'ellipsoid'
            if is_ellipsoid != (registration['factor']=='ellipsoid'):
                raise ValueError('Verification and CBS evidence types must match')
        self.overlay = native_overlay(source,settings)
        underlay = Path(os.environ.get('CBS_UNDERLAY','/workspace/.ros2/cbs-underlay'))
        wrapper = self.source/'FAST-LIVO2-ROS2/scripts/run_dpgo_ros.sh'
        native = self.overlay/'cbs_ros/lib/cbs_ros/cbs_ros_node'
        env = dict(os.environ,ROS_DOMAIN_ID=str(settings['ros_domain_id']),CBS_OVERLAY=str(self.overlay),
            CBS_UNDERLAY=str(underlay),PYTHONPATH=str(Path(__file__).resolve().parents[1])+os.pathsep+os.environ.get('PYTHONPATH',''),
            PYTHONDONTWRITEBYTECODE='1',ROS_LOCALHOST_ONLY='1' if settings.get('localhost_only',True) else '0')
        def launch(name, command):
            log = (self.output/f'{name}.log').open('w'); self.logs.append(log)
            process = subprocess.Popen(['bash',str(wrapper),*command],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            self.processes.append((name,process))
        try:
            for index,robot in enumerate(self.robots):
                local = self.output/robot; local.mkdir(); (local/'input').mkdir(); (local/'registration').mkdir()
                params = dict(native_params,robot_id=index,robot_name=robot,num_robots=len(self.robots),
                    pgo_formulation='pose3',pose_graph_topic=f'/{robot}/cbs/pose_graph',enable_visualization=False,
                    use_robust_noise_models=False,log_dir=str(local/'native'),pcm_enabled=True,pcm_session_id=self.session,
                    pcm_probability=float(settings['pcm'].get('probability',.99)),
                    pcm_minimum_clique_size=settings['pcm'].get('minimum_clique_size',1),
                    pcm_timeout_sec=float(settings['pcm'].get('timeout_s',60.)),registration_enabled=registration['enabled'])
                if registration['enabled']:
                    params.update(registration_directory=str(local/'registration'),registration_settings=json.dumps(registration,sort_keys=True))
                path = local/'cbs.yaml'; path.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
                launch(robot+'-cbs',[str(native),'--ros-args','-r',f'__ns:=/{robot}/cbs','--params-file',str(path)])
                spec = dict(robot=robot,robots=self.robots,store=str(stores[robot]),output=str(local),
                    session=self.session,pgo=config['pgo'],registration_factors=registration,
                    ros_underlay=str(underlay),ros_overlay=str(self.overlay))
                atomic_json(local/'spec.json',spec)
                launch(robot+'-bridge',[sys.executable,'-m','s3e_pipeline.persistent_worker','--spec',str(local/'spec.json')])
            atomic_json(self.output/'session.json',dict(session=self.session,mode='persistent',
                processes={name:p.pid for name,p in self.processes},config_sha256=digest(config),
                native_sha256=file_hash(native),library_sha256=file_hash(self.overlay/'cbs/lib/libcbs.so'),
                core_source_sha256=file_hash(self.source/'cbs/src/bpsam/bpsam.cpp'),
                node_source_sha256=file_hash(self.source/'cbs_ros/src/cbs_ros_node.cpp')))
        except BaseException:
            self.close(); raise

    def check(self):
        import psutil
        current = {}
        for name,process in self.processes:
            if process.poll() is not None:
                normal = name.endswith('-bridge') and process.returncode == 0 and self.final
                if not normal: raise RuntimeError(f'{name} exited {process.returncode}; see {self.output/name}.log')
            try:
                parent = psutil.Process(process.pid)
                for child in [parent,*parent.children(recursive=True)]:
                    try: current[child.pid] = child.memory_info().rss//1024
                    except (psutil.NoSuchProcess,psutil.AccessDenied): pass
            except (psutil.NoSuchProcess,psutil.AccessDenied): pass
        self.peak_rss_kib = max(self.peak_rss_kib,sum(current.values()))
        if self.submitted and time.monotonic()-self.submitted > self.config['dpgo'].get('timeout_s',600):
            raise TimeoutError('Persistent prefix was not applied/finished within its timeout')

    def ready(self):
        self.check()
        return all((self.output/r/'READY').exists() for r in self.robots)

    def submit(self, rows, edges, final=False):
        if self.final: raise ValueError('Input after final persistent seal')
        if self.submitted: raise ValueError('Previous persistent prefix not acknowledged')
        for robot in self.robots:
            if rows[robot][:len(self.last_rows.get(robot,[]))] != self.last_rows.get(robot,[]):
                raise ValueError('Persistent prefix regressed or changed')
        self.revision += 1; self.final = final; self.last_rows = copy.deepcopy(rows); self.last_edges = copy.deepcopy(edges)
        for robot in self.robots:
            incident = [e for e in edges if robot in (e['i'][0],e['j'][0])]
            atomic_json(self.output/robot/'input'/f'{self.revision}.json',
                        dict(revision=self.revision,final=final,rows=rows[robot],edges=incident))
        self.submitted = time.monotonic()

    def snapshot(self):
        self.check(); values = {}
        for robot in self.robots:
            path = self.output/robot/'estimate.json'
            if not path.exists(): return None
            value = read_json(path)
            if value['status']['revision'] != self.revision: return None
            values[robot] = value
        finished = self.final and all(v['status']['finished'] for v in values.values())
        if not self.final or finished: self.submitted = None
        poses = [p for v in values.values() for p in v['poses']]; constraints = {}
        for value in values.values():
            for edge in value['constraints']:
                key = (tuple(edge['i']),tuple(edge['j']))
                if key in constraints and digest(constraints[key]) != digest(edge):
                    raise ValueError('Persistent endpoint constraint mismatch')
                constraints[key] = edge
        if finished:
            frames = {r:values[r]['status']['component'] for r in self.robots}
            if any(frames[e['i'][0]] != frames[e['j'][0]] for e in constraints.values()):
                raise ValueError('Persistent CBS final common-frame contract failed')
        summary = dict(mode='persistent',backend='CBS+ SE(3)',transport='ROS2 Fast DDS',revision=self.revision,finished=finished,
            shared_reference=all(v['status']['reference_available'] for v in values.values()),
            pcm_enabled=True,registration_enabled=self.config['dpgo'].get('registration_factors',{}).get('enabled',False),
            iterations={r:v['status']['iteration'] for r,v in values.items()},
            keyframes=len(poses),loops=len(constraints),proposed_loops=len(self.last_edges),robots={r:v['status'] for r,v in values.items()},
            process_tree_peak_rss_kib=self.peak_rss_kib,wall_s=time.monotonic()-self.started,
            cbs_network_cdr_bytes=sum((v['status']['cbs_last_stats'] or {}).get('bandwidth_sent_bytes',0)+
                (v['status']['cbs_last_stats'] or {}).get('bandwidth_recv_bytes',0) for v in values.values()),
            registration_network_cdr_bytes=sum(v['status']['registration_network_cdr_bytes'] for v in values.values()),
            pcm_network_cdr_bytes=sum(v['status']['pcm_network_cdr_bytes'] for v in values.values()))
        return dict(summary=summary,poses=poses,constraints=list(constraints.values()),proposed=self.last_edges,
                    statistics={r:str(self.output/r/'stats.jsonl') for r in self.robots})

    def close(self):
        for _,p in self.processes:
            if p.poll() is None: os.killpg(p.pid,signal.SIGINT)
        for _,p in self.processes:
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired: os.killpg(p.pid,signal.SIGKILL); p.wait()
        for log in self.logs: log.close()


def save_snapshot(snapshot, output):
    output = Path(output); output.mkdir(parents=True,exist_ok=True)
    for name in ('poses','constraints'):
        temp = output/f'{name}.jsonl.tmp'; write_jsonl(temp,snapshot[name]); temp.replace(output/f'{name}.jsonl')
    atomic_json(output/'summary.json',snapshot['summary'])
    robots = snapshot['summary']['robots']
    atomic_json(output/'registration.json', dict(
        robots={r:v['registration'] for r,v in robots.items()},
        registration_factor_count=sum((v['registration'] or {}).get('registration_factor_count',0) for v in robots.values()),
        network_cdr_bytes=snapshot['summary']['registration_network_cdr_bytes']))
    verdicts = {}
    for status in robots.values():
        for decision in status['pcm']['verdicts']:
            key = tuple(decision[k] for k in ('robot_from','key_from','robot_to','key_to'))
            if key in verdicts and verdicts[key] != decision: raise ValueError('PCM verdict mismatch')
            verdicts[key] = decision
    atomic_json(output/'pcm.json',dict(robots={r:v['pcm'] for r,v in robots.items()},
        verdicts=list(verdicts.values()),proposed_loops=snapshot['summary']['proposed_loops'],
        retained_loops=snapshot['summary']['loops'],
        excluded_loops=snapshot['summary']['proposed_loops']-snapshot['summary']['loops'],
        network_cdr_bytes=snapshot['summary']['pcm_network_cdr_bytes']))
    write_jsonl(output/'proposed-constraints.jsonl',snapshot['proposed'])
    from .online_io import JsonlTail
    for robot, path in snapshot['statistics'].items():
        local = output/robot; local.mkdir(exist_ok=True)
        write_jsonl(local/'stats.jsonl',JsonlTail(path).read())
        write_jsonl(local/'events.jsonl',[])  # Retrieval runs in the separate live frontend.
        atomic_json(local/'summary.json',robots[robot])


def run_frozen(config, artifacts, source, out):
    """One sealed input to persistent CBS; growing capture uses persistent_online."""
    from .artifacts import read_jsonl
    from .frontends import frontend_name
    frontend=frontend_name(config); method=config['backend']['name']
    stores={r:Path(artifacts[f'keyframes.{frontend}.{r}'])/'store' for r in config['robots']}
    rows={r:read_jsonl(store/'keyframes.jsonl') for r,store in stores.items()}
    edges=read_jsonl(Path(artifacts[f'loops.{frontend}.{method}'])/'constraints.jsonl')
    session=PersistentSession(config,source,Path(out)/'persistent',stores)
    try:
        start=time.monotonic()
        while not session.ready():
            if time.monotonic()-start>config['dpgo'].get('timeout_s',600): raise TimeoutError('Persistent startup timeout')
            time.sleep(.1)
        session.submit(rows,edges,final=True)
        while True:
            snapshot=session.snapshot()
            if snapshot:
                save_snapshot(snapshot,out)
                if snapshot['summary']['finished']: return
            time.sleep(.2)
    finally: session.close()
