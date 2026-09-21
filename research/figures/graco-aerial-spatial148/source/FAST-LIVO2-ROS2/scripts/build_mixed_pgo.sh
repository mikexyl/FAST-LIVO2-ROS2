#!/usr/bin/env bash
set -euo pipefail
S3E_MIXED_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$S3E_MIXED_ROOT"
S3E_POINTS_COMMIT=9d32e7dbecf6015560d84b4901d6b0a6f483ec46
S3E_GTSAM_PREFIX="${S3E_GTSAM_PREFIX:-/home/mikexyl/workspaces/sb_slam_ros2_ws/install/gtsam}"
S3E_POINTS_SOURCE="$S3E_MIXED_ROOT/.ros2/deps/gtsam_points"
mkdir -p .ros2/deps
if [[ ! -d "$S3E_POINTS_SOURCE/.git" ]]; then
  git clone --depth 1 --branch v1.2.2 https://github.com/koide3/gtsam_points.git "$S3E_POINTS_SOURCE"
fi
[[ "$(git -C "$S3E_POINTS_SOURCE" rev-parse HEAD)" == "$S3E_POINTS_COMMIT" ]] || {
  echo 'Expected the pinned gtsam_points v1.2.2 checkout' >&2; exit 1;
}
[[ -z "$(git -C "$S3E_POINTS_SOURCE" status --porcelain)" ]] || {
  echo 'gtsam_points checkout has local changes' >&2; exit 1;
}
cmake -S FAST-LIVO2-ROS2/research/adapters/gtsam_points -B .ros2/mixed-pgo-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$S3E_GTSAM_PREFIX" \
  -DGTSAM_POINTS_SOURCE_DIR="$S3E_POINTS_SOURCE" \
  -DCMAKE_INSTALL_PREFIX="$S3E_MIXED_ROOT/.ros2/mixed-pgo-install"
cmake --build .ros2/mixed-pgo-build --parallel "${S3E_BUILD_JOBS:-3}"
cmake --install .ros2/mixed-pgo-build
python3 - <<'PY'
import hashlib, json, pathlib, subprocess
root=pathlib.Path.cwd()
binary=root/'.ros2/mixed-pgo-install/bin/s3e_mixed_pgo'
sha=lambda p:hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
libraries={}
for line in subprocess.check_output(['ldd',str(binary)],text=True).splitlines():
    if 'gtsam' in line and '=>' in line:
        path=pathlib.Path(line.split('=>')[1].split()[0]).resolve()
        libraries[str(path)]=sha(path)
assert any('libgtsam.so' in p for p in libraries)
sources=root/'FAST-LIVO2-ROS2/research/adapters/gtsam_points'
manifest=dict(binary_sha256=sha(binary),libraries=libraries,
    versions=json.loads(subprocess.check_output([str(binary),'--version'],text=True)),
    adapter_sources={str(p.relative_to(sources)):sha(p) for p in sorted(sources.rglob('*')) if p.is_file()},
    upstream_commit='9d32e7dbecf6015560d84b4901d6b0a6f483ec46',
    build_scope='unmodified upstream CPU GICP component statically linked; installed GTSAM batch LM')
(binary.parent/'build.json').write_text(json.dumps(manifest,sort_keys=True,indent=2)+'\n')
print(json.dumps(manifest['versions']))
PY
