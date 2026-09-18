"""Serialized MegaLoc inference and robot evidence contracts."""
import base64
import io
import json
import os
import subprocess
import numpy as np
from .artifacts import canonical
from .registration import FeatureCache


class JsonWorker:
    def __init__(self,command,environment=None):
        if not command:
            raise ValueError('Backend requires an explicit pinned worker command')
        self.process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
            env=None if environment is None else dict(os.environ,**environment))

    def call(self,**request):
        self.process.stdin.write(canonical(request)+b'\n');self.process.stdin.flush()
        line=self.process.stdout.readline()
        if not line:
            raise RuntimeError(f'Model worker exited ({self.process.poll()})')
        result=json.loads(line)
        if 'error' in result:
            raise RuntimeError(result['error'])
        return result['result']

    def close(self):
        self.process.stdin.close()
        try:
            status=self.process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            self.process.terminate();self.process.wait(timeout=5)
            raise RuntimeError('Model worker failed to stop')
        if status:
            raise RuntimeError(f'Model worker failed with status {status}')


class Backend:
    def __init__(self,cfg):
        self.cfg=cfg
        self.geometry_cache=FeatureCache(cfg.get("registration",{}).get("feature_cache_mib",128))
    def close(self):pass


def pack_array(array):
    buffer=io.BytesIO();np.save(buffer,array,allow_pickle=False)
    return base64.b64encode(buffer.getvalue()).decode()


def unpack_array(value):
    return np.load(io.BytesIO(base64.b64decode(value,validate=True)),allow_pickle=False)


def pack_payload(payload):
    return dict(row=payload['row'],cloud=pack_array(payload['cloud']),
        image=base64.b64encode(payload['image']).decode(),descriptor=payload['descriptor'])


def unpack_payload(value):
    return dict(row=value['row'],cloud=unpack_array(value['cloud']),
        image=base64.b64decode(value['image'],validate=True),descriptor=value['descriptor'])


def create(cfg):
    if cfg.get('name') not in ('megaloc_mapclosures','mapclosures'):
        raise ValueError('Expected MegaLoc + MapClosures or MapClosures-only')
    from .mapclosures import MegaLocMapClosures
    return MegaLocMapClosures(cfg)
