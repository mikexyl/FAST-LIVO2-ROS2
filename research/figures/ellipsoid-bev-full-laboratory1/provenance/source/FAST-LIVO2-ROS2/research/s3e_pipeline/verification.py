"""Bounded robot-owned verification, independent of the causal retrieval index."""
from concurrent.futures import ThreadPoolExecutor
import json
import multiprocessing as mp
from pathlib import Path
import resource
import time

from .artifacts import canonical,digest,read_json,write_json
from .backends import create,unpack_payload


class Verifier:
    def __init__(self,config,backend=None):
        self.backend=backend if backend is not None else create(config)
        self.cache=Path(config['local_cache']) if config.get('local_cache') else None

    def verify(self,packet):
        start=time.monotonic();cached=False
        path=self.cache/f'{digest(packet)}.verification.json' if self.cache else None
        if path is not None and path.exists():
            entry=read_json(path)
            if digest(entry['result'])!=entry['sha256']:raise ValueError('Modified verification cache')
            result=entry['result'];cached=True
        else:
            extra=dict(proposal=packet['proposal']) if 'proposal' in packet else {}
            result=self.backend.verify(unpack_payload(packet['query']),unpack_payload(packet['candidate']),**extra)
            if path is not None:
                temp=path.with_suffix('.partial')
                write_json(temp,dict(result=result,sha256=digest(result)));temp.replace(path)
        return dict(result=result,cache_hit=cached,seconds=time.monotonic()-start,
                    max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)


def verifier_main(connection,config):
    # Landlock is inherited across fork/exec. This process receives serialized
    # evidence only and has no retrieval index or additional file capability.
    import ctypes,os,signal
    parent=os.getppid();ctypes.CDLL(None).prctl(1,signal.SIGTERM,0,0,0)
    if os.getppid()!=parent:return
    verifier=None
    try:
        verifier=Verifier(config);connection.send_bytes(canonical({'ready':True}))
        while True:
            request=json.loads(connection.recv_bytes())
            if request['op']=='close':break
            if request['op']!='verify':raise ValueError('Unknown verification operation')
            connection.send_bytes(canonical(verifier.verify(request['packet'])))
        verifier.backend.close();verifier=None
        connection.send_bytes(canonical({'closed':True}))
    except BaseException:
        import traceback
        try:connection.send_bytes(canonical({'error':traceback.format_exc()}))
        except (BrokenPipeError,EOFError):pass
    finally:
        if verifier:
            try:verifier.backend.close()
            except Exception:pass
        connection.close()


class VerificationProcess:
    def __init__(self,config,timeout):
        self.timeout=timeout;self.closed=False
        context=mp.get_context('spawn');self.connection,child=context.Pipe()
        self.process=context.Process(target=verifier_main,args=(child,config))
        self.process.start();child.close()
        try:self.receive()
        except BaseException:
            self.process.terminate();self.process.join();self.connection.close();raise
        # A single IPC thread avoids blocking the retrieval worker while a large
        # packet waits for the verifier. Computation itself runs in its own process.
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='verification-ipc')

    def receive(self):
        if not self.connection.poll(self.timeout):raise TimeoutError('Robot verifier timed out')
        response=json.loads(self.connection.recv_bytes())
        if 'error' in response:raise RuntimeError(response['error'])
        return response

    def call(self,packet):
        self.connection.send_bytes(canonical(dict(op='verify',packet=packet)))
        return self.receive()

    def submit(self,packet):return self.pool.submit(self.call,packet)

    def close(self):
        if self.closed:return
        self.closed=True
        try:
            self.pool.shutdown(wait=True,cancel_futures=True)
            if self.process.is_alive():
                self.connection.send_bytes(canonical({'op':'close'}));self.receive()
                self.process.join(timeout=10)
            if self.process.exitcode!=0:raise RuntimeError('Verifier failed to exit cleanly')
        finally:
            if self.process.is_alive():self.process.terminate();self.process.join(timeout=5)
            self.connection.close()
