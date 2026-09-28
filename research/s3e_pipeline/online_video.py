"""Adaptive component panels shared by live capture and arrival-journal rendering."""
import math
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def group_panels(groups, box=(30, 206, 1860, 770), gap=12):
    """Partition the canvas by group size; every robot appears exactly once."""
    ordered = sorted(groups, key=lambda g: (-len(groups[g]), g))
    result = {}

    def split(keys, bounds):
        x, y, width, height = bounds
        if len(keys) == 1:
            result[keys[0]] = bounds
            return
        total = sum(len(groups[g]) for g in keys)
        cut = min(range(1, len(keys)),
                  key=lambda i: abs(sum(len(groups[g]) for g in keys[:i]) - total / 2))
        ratio = sum(len(groups[g]) for g in keys[:cut]) / total
        # Favor landscape tiles for oblique maps, including smaller components.
        if width >= height * 1.8:
            first = round((width - gap) * ratio)
            split(keys[:cut], (x, y, first, height))
            split(keys[cut:], (x + first + gap, y, width - first - gap, height))
        else:
            first = round((height - gap) * ratio)
            split(keys[:cut], (x, y, width, first))
            split(keys[cut:], (x, y + first + gap, width, height - first - gap))

    if ordered:
        split(ordered, box)
    return result


def group_video(recorder, elapsed, scene, replay=False):
    groups, main, tracks, clouds, _ = scene
    image = Image.new('RGB', (1920, 1080), (11, 18, 29))
    draw = ImageDraw.Draw(image)
    fonts = recorder.fonts

    def text(x, y, value, size=19, color=(216, 226, 238)):
        draw.text((x, y), value, font=fonts[size], fill=color)

    text(36, 20, 'GROUND + AIR', 38)
    text(398, 31, 'ONLINE MULTI-ROBOT SLAM', 28, (114, 194, 214))
    text(36, 74, f'{len(recorder.robots)} concurrent GRACO robots · estimated gravity-aligned maps · workstation 148', 19, (145, 164, 187))
    values = [('ONLINE ELAPSED', f'{elapsed:06.1f} s'),
              ('ALIGNED GROUPS', str(len(groups))),
              ('LARGEST GROUP', f'{len(groups[main])} / {len(recorder.robots)} robots'),
              ('RETAINED LOOPS', str(len(recorder.loops))),
              ('CBS REVISION', str(recorder.revision))]
    for i, (label, value) in enumerate(values):
        x = 38 + 355 * i
        text(x, 116, label, 14, (130, 148, 169))
        text(x, 139, value, 28)

    yaw = .55
    projection = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                           [.48 * math.sin(yaw), .48 * math.cos(yaw), .88]])
    for group, (x, y, width, height) in group_panels(groups).items():
        members = groups[group]
        # Render into a separate tile so points, labels and tracks cannot bleed
        # into another coordinate frame's panel.
        tile = Image.new('RGB', (width, height), (16, 27, 40))
        td = ImageDraw.Draw(tile)
        td.rounded_rectangle((0, 0, width-1, height-1), radius=10,
                             outline=(56, 87, 105) if len(members)>1 else (39, 59, 79), width=2)
        title = ('GLOBAL GROUP · ALL ROBOTS ALIGNED' if len(groups)==1 else
                 f'ALIGNED GROUP · {len(members)} ROBOTS' if len(members)>1 else 'NOT YET CONNECTED')
        if width < 380:
            title = f'ALIGNED · {len(members)} ROBOTS' if len(members)>1 else 'NOT YET CONNECTED'
        td.text((14, 10), title, font=fonts[16 if width>380 else 14], fill=(161, 205, 220))
        # A compact color legend remains in the global panel after all other
        # panels have disappeared. It is also the membership label of each tile.
        lx, ly = 14, 37
        for robot in members:
            label = robot.replace('aerial', 'A').replace('ground', 'G')
            if lx + 57 > width-14:
                lx, ly = 14, ly + 21
            td.ellipse((lx, ly+4, lx+8, ly+12), fill=tuple(recorder.colors[robot]))
            td.text((lx+13, ly), label, font=fonts[14], fill=tuple(recorder.colors[robot]))
            lx += 58
        top, bottom = ly+28, height-14
        projected_clouds = {r: clouds[r] @ projection.T for r in members}
        projected_tracks = {r: tracks[r] @ projection.T for r in members}
        valid = [a for a in [*projected_clouds.values(), *projected_tracks.values()] if len(a)]
        if valid and bottom > top:
            lo = np.min([a.min(axis=0) for a in valid], axis=0)
            hi = np.max([a.max(axis=0) for a in valid], axis=0)
            scale = .92 * min((width-28)/max(hi[0]-lo[0],10), (bottom-top)/max(hi[1]-lo[1],10))
            center = (lo+hi)/2

            def pixels(points):
                return np.rint((points-center)*[scale, -scale]+[width/2, (top+bottom)/2]).astype(int)

            canvas = np.asarray(tile).copy()
            for robot in members:
                p = pixels(projected_clouds[robot])
                keep = (p[:,0]>=5)&(p[:,0]<width-5)&(p[:,1]>=top)&(p[:,1]<bottom)
                p = p[keep]
                canvas[p[:,1], p[:,0]] = np.asarray(recorder.colors[robot])*.72
            tile = Image.fromarray(canvas)
            td = ImageDraw.Draw(tile)
            for robot in members:
                p = pixels(projected_tracks[robot])
                color = tuple(recorder.colors[robot])
                if len(p)>1:
                    td.line([tuple(v) for v in p], fill=color, width=3 if width>700 else 2)
                if len(p):
                    px, py = p[-1]
                    td.ellipse((px-4,py-4,px+4,py+4),fill=color,outline=(230,237,242))
        else:
            td.text((14, top+8), 'Waiting for map arrivals', font=fonts[14], fill=(108,133,157))
        image.paste(tile, (x, y))

    draw = ImageDraw.Draw(image)
    text(36, 990, 'EllipseLIO  →  accumulated area maps  →  multilayer ellipsoid BEVs  →  MapClosures  →  PCM / CBS', 22)
    label = 'Re-rendered from online arrival journals' if replay else 'Live arrival timeline'
    text(36, 1033, label+' · each panel has its own frame and scale · no ground-truth alignment', 16, (121,148,173))
    return image


def video_fonts():
    return {n: ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', n)
            for n in (14,16,19,22,28,38)}
