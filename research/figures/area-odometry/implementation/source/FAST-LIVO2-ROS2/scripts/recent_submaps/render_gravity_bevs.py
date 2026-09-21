"""Reconstruct native ellipsoid density images and verify frozen ORB features."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import zlib

import cv2
import numpy as np
import yaml

ROOT = Path('/workspace')
parser = argparse.ArgumentParser()
parser.add_argument('--work', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--original-gallery', type=Path)
args = parser.parse_args()
WORK, OUT = args.work, args.output
trials = json.loads((WORK/'trials.json').read_text())
sys.path.insert(0, str(ROOT/'FAST-LIVO2-ROS2/research'))
from s3e_pipeline.backends import unpack_array
from s3e_pipeline.ellipsoid_bev import normalize_exported_basis
from s3e_pipeline.ellipsoid_cuda import SurfaceSampler
from s3e_pipeline.mapclosures_inspection import check_features, density_pixels
import s3e_mapclosures_inspection as inspection


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


started = time.monotonic()
OUT.mkdir(exist_ok=False)
(OUT/'images').mkdir()
cfg = yaml.safe_load((WORK/'config.yaml').read_text())
mc = cfg['backend']['mapclosures']
assert inspection.upstream_commit == mc['upstream_commit']
frozen = read(WORK/'source-hashes.json')
assert all(sha(Path(p)) == h for p, h in frozen.items())
sampler = SurfaceSampler()
inspector = inspection.Inspector(mc['density_map_resolution'], mc['density_threshold'], mc['hamming_distance_threshold'])
maps = []
try:
    for robot in cfg['robots']:
        keys = rows(WORK/f'prepared-{robot}/store/keyframes.jsonl')
        initial_ns = keys[0]['begin_ns']
        for key in keys:
            index = key['keyframe_id']
            payload = Path(trials[robot])/'frontend/submaps'/key['payload']
            assert sha(payload) == key['sha256']
            descriptor = WORK/f'prepared-{robot}/ellipsoid/{index:06d}.json.zlib'
            packet = json.loads(zlib.decompress(descriptor.read_bytes()))
            assert packet['member_scan_ids'] == key['member_scan_ids']
            assert packet['payload_sha256'] == key['sha256']
            features = {k: unpack_array(v) for k, v in packet['mapclosures'].items()}
            with np.load(payload, allow_pickle=False) as archive:
                e = archive['ellipsoids']
            assert len(e)
            basis, _ = normalize_exported_basis(e[:, 6:].reshape(-1, 3, 3), cfg['ellipsoid_basis_roundoff_tolerance'])
            points, sampling = sampler.render(e[:, :3], e[:, 3:6], basis)
            debug = inspector.density(points, features['ground'])
            check_features(debug, features)
            name = f'{robot}-{index:06d}'
            assert cv2.imwrite(str(OUT/'images'/f'{name}.png'), debug['image'])
            maps.append(dict(robot=robot, key=index, submap_id=key['submap_id'],
                start_s=(key['begin_ns']-initial_ns)/1e9, end_s=(key['end_ns']-initial_ns)/1e9,
                image=f'images/{name}.png', width=int(debug['image'].shape[1]), height=int(debug['image'].shape[0]),
                ellipsoids=len(e), features=len(features['xy']), member_scans=len(key['member_scan_ids']),
                orb_xy=density_pixels(features['xy'], debug['lower_bound']).tolist(),
                lower_bound=np.asarray(debug['lower_bound']).tolist(), ground=features['ground'].tolist(),
                payload_sha256=key['sha256'], descriptor_sha256=sha(descriptor), png_sha256=sha(OUT/'images'/f'{name}.png'),
                exact_cached_features=True, projection_alignment=packet.get('projection_alignment')))
            if len(maps) % 25 == 0:
                print(f'{len(maps)} native density images verified', flush=True)
finally:
    sampler.close()
assert len(maps) == sum(len(rows(WORK/f'prepared-{r}/store/keyframes.jsonl')) for r in cfg['robots'])
assert all(sha(Path(p)) == h for p, h in frozen.items())
loops = [dict(i=e['i'], j=e['j']) for e in rows(WORK/'dpgo/constraints.jsonl')]
summary = dict(complete=True, maps=maps, loops=loops, ground_truth_used=False,
    density_map_resolution_m=mc['density_map_resolution'], density_threshold=mc['density_threshold'],
    upstream_commit=inspection.upstream_commit, exact_cached_features=True, frozen_sources_verified=True,
    runtime_s=time.monotonic()-started, inspection_library_sha256=sha(Path(inspection.__file__)),
    renderer_sha256=sha(Path(__file__)))
(OUT/'manifest.json').write_text(json.dumps(summary, indent=2)+'\n')

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, axes = plt.subplots(2, 2, figsize=(11, 10), layout='constrained', facecolor='#f3f5f8')
for ax, robot in zip(axes.flat, cfg['robots']):
    item = next(m for m in maps if m['robot'] == robot and m['key'] == 8)
    im = cv2.imread(str(OUT/item['image']), cv2.IMREAD_GRAYSCALE)
    ax.imshow(im, cmap='gray', vmin=0, vmax=255, interpolation='nearest')
    ax.set_title(f'{robot.replace("aerial", "Aerial ")} · submap {item["submap_id"]}\n'
                 f'{item["start_s"]:.0f}–{item["end_s"]:.0f} s · {item["features"]} ORB features', fontsize=13, pad=9)
    ax.axis('off')
    length = 20/mc['density_map_resolution']
    x, y = 12, im.shape[0]-14
    ax.plot([x, x+length], [y, y], color='#ffd75e', lw=3)
    ax.text(x+length/2, y-5, '20 m', color='#ffd75e', ha='center', va='bottom', fontsize=10)
fig.suptitle('GRACO aerial · gravity-horizontal ellipsoid BEVs', fontsize=20)
fig.savefig(OUT/'overview.png', dpi=180)
plt.close(fig)

payload = json.dumps(dict(maps=maps, loops=loops, resolution=mc['density_map_resolution']))
html = '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>GRACO aerial — ellipsoid BEVs</title>
<style>body{font:16px system-ui,sans-serif;background:#eef1f5;color:#152033;margin:0}main{max-width:1400px;margin:auto;padding:28px}
h1{font-size:30px;margin:0 0 12px}p{line-height:1.5}.controls{display:flex;gap:16px;flex-wrap:wrap;align-items:center;background:white;padding:16px;border-radius:10px}
select,input,button{font:inherit}select,button{padding:6px}#slider{width:280px}.panels{display:grid;grid-template-columns:repeat(auto-fit,minmax(330px,1fr));gap:20px;margin-top:20px}
.panel{background:white;padding:16px;border-radius:10px}.panel h2{margin:0 0 10px;font-size:20px}.image{background:#000;display:flex;align-items:center;justify-content:center;min-height:360px}
canvas{max-width:100%;max-height:65vh;width:auto;height:auto;image-rendering:pixelated}.meta{font-size:14px;color:#4d5a70}a{color:#1648a6}</style>
<main><h1>GRACO aerial — ellipsoid BEVs</h1>
<p>Completed temporal submaps, 10 seconds each with 5 seconds overlap. Gravity-horizontal orthographic projection reconstructed from startup accelerometer measurements and saved IMU poses; the exact per-anchor filter gravity was not recorded in these historical captures. No ground truth was used. These are the native grayscale density images reconstructed from the experiment’s saved ellipsoids. Every image reproduces the cached ORB keypoints and descriptors exactly.</p>
<div class="controls"><label>Robot <select id="robot"></select></label><button id="previous">←</button><input id="slider" type="range" min="0"><button id="next">→</button><span id="position"></span><label><input type="checkbox" id="features"> Show retained ORB features</label></div>
<div class="panels" id="single"></div>
<h2>Accepted loop pairs</h2><p>Side-by-side views in their individual gravity-leveled submap frames. The selected pairs survived geometric verification and PCM.</p><select id="pair"></select><div class="panels" id="paired"></div>
<p class="meta">0.5 m per image pixel. Bright pixels show higher projected ellipsoid-surface density. Times are relative to each robot’s first initialized submap. Yellow circles mark retained ORB keypoints when enabled. The displayed maps retain their individual image orientation and extent.</p>
<p><a href="overview.png">Four-robot overview</a> · <a href="manifest.json">Image provenance and verification</a></p></main>
<script>const DATA=__DATA__;const el=id=>document.getElementById(id);const robots=[...new Set(DATA.maps.map(m=>m.robot))];
for(const r of robots)el('robot').add(new Option(r,r));for(let i=0;i<DATA.loops.length;i++){let l=DATA.loops[i];el('pair').add(new Option(`${l.i[0]} / ${l.i[1]} ↔ ${l.j[0]} / ${l.j[1]}`,i));}
function lookup(endpoint){return DATA.maps.find(m=>m.robot===endpoint[0]&&m.key===endpoint[1]);}
function panel(m){let div=document.createElement('div');div.className='panel';let title=document.createElement('h2');title.textContent=`${m.robot} · submap ${m.submap_id} · ${m.start_s.toFixed(0)}–${m.end_s.toFixed(0)} s`;div.append(title);
let holder=document.createElement('div');holder.className='image';let canvas=document.createElement('canvas');canvas.width=m.width;canvas.height=m.height;holder.append(canvas);div.append(holder);
let image=new Image();image.onload=()=>{let ctx=canvas.getContext('2d');ctx.drawImage(image,0,0);if(el('features').checked){ctx.strokeStyle='#ffe15a';ctx.lineWidth=.8;for(let [x,y] of m.orb_xy){ctx.beginPath();ctx.arc(x,y,2.4,0,2*Math.PI);ctx.stroke();}}};image.src=m.image;
let meta=document.createElement('p');meta.className='meta';meta.textContent=`${m.ellipsoids.toLocaleString()} ellipsoids · ${m.member_scans} member scans · ${m.features} ORB features · ${m.width} × ${m.height} pixels`;div.append(meta);
let link=document.createElement('a');link.href=m.image;link.textContent='Open original PNG';link.target='_blank';div.append(link);return div;}
function show(){let list=DATA.maps.filter(m=>m.robot===el('robot').value);el('slider').max=list.length-1;let i=Math.min(+el('slider').value,list.length-1);el('slider').value=i;el('position').textContent=`${i+1} / ${list.length}`;el('single').replaceChildren(panel(list[i]));let p=DATA.loops[+el('pair').value];el('paired').replaceChildren(...(p?[panel(lookup(p.i)),panel(lookup(p.j))]:[]));}
el('robot').onchange=()=>{el('slider').value=8;show()};el('slider').oninput=show;el('features').onchange=show;el('pair').onchange=show;
el('previous').onclick=()=>{el('slider').value=Math.max(0,+el('slider').value-1);show()};el('next').onclick=()=>{el('slider').value=Math.min(+el('slider').max,+el('slider').value+1);show()};
el('slider').value=8;el('pair').value=0;show();</script></html>'''
(OUT/'index.html').write_text(html.replace('__DATA__', payload))
print(json.dumps(dict(images=len(maps), exact_cached_features=True, runtime_s=time.monotonic()-started)), flush=True)

if args.original_gallery:
    fig, axes = plt.subplots(2, len(cfg['robots']), figsize=(18, 10), layout='constrained')
    old = read(args.original_gallery/'manifest.json')
    for col, robot in enumerate(cfg['robots']):
        for row, (gallery, entries, label) in enumerate([
                (args.original_gallery, old['maps'], 'Before: IMU XY (oblique)'),
                (OUT, maps, 'After: gravity-horizontal')]):
            item = next(m for m in entries if m['robot']==robot and m['key']==8)
            im = cv2.imread(str(gallery/item['image']), cv2.IMREAD_GRAYSCALE)
            axes[row,col].imshow(im,cmap='gray',vmin=0,vmax=255,interpolation='nearest')
            axes[row,col].set_title(f"{robot} · {label}\nSubmap 8 · {item['features']} ORB features")
            axes[row,col].axis('off')
            x,y,length=12,im.shape[0]-14,20/mc['density_map_resolution']
            axes[row,col].plot([x,x+length],[y,y],color='#ffd75e',lw=3)
            axes[row,col].text(x+length/2,y-5,'20 m',color='#ffd75e',ha='center',va='bottom')
    fig.suptitle('Same saved ellipsoids and member scans · projection correction only',fontsize=18)
    fig.savefig(OUT/'before-after.png',dpi=170)
    plt.close(fig)
