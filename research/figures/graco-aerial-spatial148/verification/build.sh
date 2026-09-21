#!/usr/bin/env bash
set -eo pipefail
cd /workspace
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR=/tmp/graco-spatial-build-ros
B=/workspace/.ros2/graco-aerial-spatial148-20260920
exec >"$B/build.log" 2>&1
/workspace/.ros2/research-venv/bin/cmake -S FAST-LIVO2-ROS2/research/adapters/mapclosures -B "$B/mapclosures" \
 -DCMAKE_BUILD_TYPE=Release -DMAPCLOSURES_SOURCE=/workspace/dependencies/mapclosures \
 -DHBST_SOURCE=/workspace/dependencies/mapclosures-hbst -DSOPHUS_SOURCE=/workspace/dependencies/mapclosures-sophus \
 -DPYTHON_EXECUTABLE=/workspace/.ros2/research-venv/bin/python \
 -DPYTHON_MODULE_DIR="$B/modules" \
 -Dpybind11_DIR=/workspace/.ros2/research-venv/lib/python3.10/site-packages/pybind11/share/cmake/pybind11
/workspace/.ros2/research-venv/bin/cmake --build "$B/mapclosures" --parallel "$(nproc)"
# /opt is already a runtime-library capability of isolated workers. This copy
# lives only in the dedicated container; historical venv modules stay untouched.
mkdir -p /opt/graco-spatial-mapclosures
cp "$B"/mapclosures/s3e_mapclosures_native*.so /opt/graco-spatial-mapclosures/
colcon --log-base "$B/build-log" build --base-paths /workspace/ellipselio \
 --build-base "$B/build" --install-base "$B/install" --cmake-args -DCMAKE_BUILD_TYPE=Release \
 -DBUILD_TESTING=ON -DS3E_RESEARCH_SOURCE=/workspace/FAST-LIVO2-ROS2 -DPYTHON_EXECUTABLE=/usr/bin/python3
source "$B/install/ellipselio/share/ellipselio/local_setup.bash"
cmake -S FAST-LIVO2-ROS2/scripts/ellipselio_live -B "$B/launcher" -DCMAKE_BUILD_TYPE=Release
cmake --build "$B/launcher" --parallel "$(nproc)"
ctest --test-dir "$B/build/ellipselio" --output-on-failure
touch "$B/BUILD_COMPLETE"
