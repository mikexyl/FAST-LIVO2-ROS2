#!/usr/bin/env python3
"""Pinned MegaLoc GPU descriptor inference in its isolated environment."""
import argparse
import base64
from contextlib import redirect_stdout
import fcntl
import importlib.util
import io
import json
from pathlib import Path
import sys
import numpy as np
import torch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.artifacts import canonical


def load_module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module);return module


def image_tensor(data,size,gray=False):
    import torchvision.transforms as tv
    from PIL import Image,ImageOps
    image=Image.open(io.BytesIO(data))
    if gray:image=ImageOps.equalize(image.convert('L')).convert('RGB')
    else:image=image.convert('RGB')
    if gray:
        return tv.Compose([tv.Resize((size,size)),tv.ToTensor(),
            tv.Normalize([.485,.456,.406],[.229,.224,.225])])(image)[None].cuda()
    return tv.Compose([tv.ToTensor(),tv.Normalize([.485,.456,.406],[.229,.224,.225]),
                       tv.Resize((size,size),antialias=True)])(image)[None].cuda()


class MegaLoc:
    def __init__(self,args):
        from safetensors.torch import load_file
        module=load_module(args.source/'megaloc_model.py','s3e_megaloc')
        self.model=module.MegaLoc();self.model.load_state_dict(load_file(args.checkpoint),strict=True)
        self.model=self.model.eval().cuda()
    def describe(self,request):
        with torch.inference_mode():v=self.model(image_tensor(base64.b64decode(request['image']),322))[0]
        return {'global':v.cpu().float().tolist()}


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--gpu-lock',type=Path,required=True)
    args=p.parse_args()
    if not torch.cuda.is_available():raise RuntimeError('MegaLoc inference requires CUDA')
    torch.set_num_threads(1);torch.manual_seed(0);np.random.seed(0)
    with args.gpu_lock.open('a+') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        with redirect_stdout(sys.stderr):model=MegaLoc(args)
        for line in sys.stdin.buffer:
            try:
                request=json.loads(line)
                with redirect_stdout(sys.stderr):
                    if request['op']=='describe':result=model.describe(request)
                    elif request['op']=='stats':
                        torch.cuda.synchronize()
                        result=dict(device=torch.cuda.get_device_name(),peak_cuda_reserved_bytes=torch.cuda.max_memory_reserved())
                    else:raise ValueError('Unsupported MegaLoc operation')
                response={'result':result}
            except Exception as exc:response={'error':str(exc)}
            sys.stdout.buffer.write(canonical(response)+b'\n');sys.stdout.buffer.flush()
        torch.cuda.synchronize()

if __name__=='__main__':main()
