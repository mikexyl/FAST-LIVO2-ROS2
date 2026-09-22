"""Linux Landlock restricts robot workers to their own artifact directories."""
import ctypes
import errno
import os
from pathlib import Path
import sys


def restrict_reads(allowed):
    # Landlock ABI 1: enforce filesystem read/execute capabilities at the kernel,
    # including native libraries and subprocesses. No GT/data-root capability.
    libc=ctypes.CDLL(None,use_errno=True)
    create,add,restrict=444,445,446
    READ=(1<<0)|(1<<2)|(1<<3)
    class Ruleset(ctypes.Structure):_fields_=[('handled_access_fs',ctypes.c_uint64)]
    class PathRule(ctypes.Structure):
        _pack_=1
        _fields_=[('allowed_access',ctypes.c_uint64),('parent_fd',ctypes.c_int32)]
    attr=Ruleset(READ)
    ruleset=libc.syscall(create,ctypes.byref(attr),ctypes.sizeof(attr),0)
    if ruleset<0:
        raise RuntimeError(f'Linux Landlock is required for worker filesystem isolation: {os.strerror(ctypes.get_errno())}')
    try:
        paths={Path(p).resolve() for p in allowed if Path(p).exists()}
        for path in sorted(paths):
            fd=os.open(path,os.O_PATH|os.O_CLOEXEC)
            try:
                access=READ if path.is_dir() else READ & ~(1<<3)
                rule=PathRule(access,fd)
                if libc.syscall(add,ruleset,1,ctypes.byref(rule),0)<0:
                    raise OSError(ctypes.get_errno(),f'Landlock rule: {path}')
            finally:os.close(fd)
        if libc.prctl(38,1,0,0,0) or libc.syscall(restrict,ruleset,0)<0:
            raise OSError(ctypes.get_errno(),'Landlock restriction failed')
    finally:os.close(ruleset)


def worker_paths(store, descriptors, backend):
    paths=['/usr','/lib','/lib64','/etc','/opt','/dev','/proc',sys.prefix,
           Path(__file__).resolve().parents[1],store,descriptors]
    # Commands and explicitly pinned model/config assets are capabilities.
    paths.extend(backend.get('read_paths',[]))
    for arg in backend.get('command',[]):
        p=Path(arg)
        if p.is_absolute() and p.exists():paths.append(p if p.is_file() else p)
    return paths
