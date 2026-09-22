"""Adapt the audited area-map gallery renderer to this local two-robot result."""
from pathlib import Path
B=Path(__file__).resolve().parent;HOST=B.parents[1]
text=(HOST/'.ros2/upstream-area-graco-aerial148-20260921/render_bevs.py').read_text()
text=text.replace("ROOT = Path('/workspace')","ROOT = Path(__file__).resolve().parent/'source'")
text=text.replace("read(WORK.parent/'source-hashes.json')","read(WORK/'source-hashes.json')")
text=text.replace('GRACO aerial 05–08','GRACO ground-06 + aerial-06')
text=text.replace('Four-robot overview','Two-robot overview')
text=text.replace('Completed spatial submaps: 40 m horizontal radius, 20 m overlap, and a 120 s age guard.',
    'Accumulated-area snapshots: 80 m horizontal radius, all historical map points inside the area, and no height or age cutoff. Snapshot triggers are 20 m movement or 10 seconds.')
text=text.replace('The selected pairs survived geometric verification and PCM.',
    'The selected pairs passed geometric verification and the configured PCM gate. Singleton acceptance is enabled; one-loop pairs have no pairwise consistency check.')
(B/'render_bevs.py').write_text(text)
