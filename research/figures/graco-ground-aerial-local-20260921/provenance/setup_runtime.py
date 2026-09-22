import json,shutil
from pathlib import Path
B=Path(__file__).resolve().parent;HOST=B.parents[1];ROOT=B/'source'
runtime=ROOT/'.ros2'
if runtime.is_symlink():
    assert runtime.resolve()==HOST/'.ros2'
    runtime.unlink()
runtime.mkdir(exist_ok=True)
for name in ('research-venv','rerun-venv','dpgo-install','dpgo-build'):
    (runtime/name).symlink_to(HOST/'.ros2'/name,target_is_directory=True)
(runtime/'graco-ground-aerial-local-20260921').symlink_to(B,target_is_directory=True)
for name in ('cbs','cbs_ros'):(ROOT/name).symlink_to(HOST/name,target_is_directory=True)
(runtime/'ellipsoid-cuda').mkdir()
shutil.copy2(HOST/'.ros2/upstream-area-s3e-20260921/capacity-cuda/surface.cu',runtime/'ellipsoid-cuda/surface.cu')
(B/'native').mkdir()
for old in (HOST/'.ros2/spatial-submaps-20260920/native').glob('*.so'):shutil.copy2(old,B/'native'/old.name)
for old in (HOST/'.ros2/mapclosures-inspection-build').glob('s3e_mapclosures_inspection*.so'):shutil.copy2(old,B/'native'/old.name)
selection=dict(robots=['ground06','aerial06'],bags={'ground06':'/data/graco/ground-06_ros2','aerial06':'/data/graco/aerial-06-20m_full_ros2'},
    reason='Official route diagrams show overlapping northeastern street; aerial06 has nominal 20 m altitude.',
    route_sources=['https://github.com/SYSU-RoboticsLab/GrAco/blob/main/doc/sequence-ground.png','https://github.com/SYSU-RoboticsLab/GrAco/blob/main/doc/sequence-aerial.png'],
    numeric_ground_truth_used_for_selection=False,selection_is_visual_not_exhaustive_overlap_optimization=True,
    thresholds_unchanged=True,host='local laptop',workstation148_used=False)
(B/'selection.json').write_text(json.dumps(selection,indent=2)+'\n')
