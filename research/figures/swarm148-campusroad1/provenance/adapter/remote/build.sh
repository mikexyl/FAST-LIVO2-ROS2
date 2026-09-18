#!/usr/bin/env bash
# Run inside the pinned Humble image with the isolated source bind at /workspace.
set -eo pipefail
cd /workspace
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-6}"
export PIP_CACHE_DIR=/workspace/.ros2/pip-cache
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
mkdir -p .ros2/swarm/provenance
python3 -m venv --system-site-packages .ros2/swarm/venv
.ros2/swarm/venv/bin/pip install 'numpy==1.26.4' 'scipy==1.15.3' \
  'mcap==1.4.0' 'rosbags==0.11.5' 'pybind11==3.1.0' 'pytest==9.1.1'
python3 -m venv --system-site-packages .ros2/research-venv
.ros2/research-venv/bin/pip install 'numpy==1.26.4' 'scipy==1.15.3' \
  'mcap==1.4.0' 'rosbags==0.11.5' 'evo==1.36.5' 'gtsam==4.2'
.ros2/swarm/venv/bin/pip freeze > .ros2/swarm/provenance/swarm-pip-freeze.txt
.ros2/research-venv/bin/pip freeze > .ros2/swarm/provenance/research-pip-freeze.txt
cmake -S dependencies/gtsam -B .ros2/swarm/gtsam-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/workspace/.ros2/swarm/native \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TIMING_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_WITH_TBB=OFF
cmake --build .ros2/swarm/gtsam-build
cmake --install .ros2/swarm/gtsam-build
cmake -S dependencies/TEASER-plusplus -B .ros2/swarm/teaser-native-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/workspace/.ros2/swarm/native \
  -DBUILD_TESTING=OFF -DBUILD_DOC=OFF -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_TEASER_FPFH=OFF -DENABLE_DIAGNOSTIC_PRINT=OFF -DBUILD_WITH_MARCH_NATIVE=OFF \
  -DFETCHCONTENT_SOURCE_DIR_PMC=/workspace/dependencies/pmc-src \
  -DFETCHCONTENT_SOURCE_DIR_SPECTRA=/workspace/dependencies/spectra-src \
  -DFETCHCONTENT_SOURCE_DIR_TINYPLY=/workspace/dependencies/tinyply-src
cmake --build .ros2/swarm/teaser-native-build
cmake --install .ros2/swarm/teaser-native-build
cmake -S Swarm-SLAM/s3e/teaser_binding -B .ros2/swarm/teaser-build \
  -DCMAKE_BUILD_TYPE=Release -DTEASER_SOURCE=/workspace/dependencies/TEASER-plusplus \
  -Dteaserpp_DIR=/workspace/.ros2/swarm/native/lib/cmake/teaserpp \
  -Dpybind11_DIR=/workspace/.ros2/swarm/venv/lib/python3.10/site-packages/pybind11/share/cmake/pybind11 \
  -DPYTHON_EXECUTABLE=/workspace/.ros2/swarm/venv/bin/python
cmake --build .ros2/swarm/teaser-build
bash Swarm-SLAM/s3e/build.sh
colcon --log-base .ros2/ellipse-log build --base-paths ellipselio \
  --build-base .ros2/ellipse-build --install-base .ros2/ellipse-install \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DS3E_RESEARCH_SOURCE=/workspace/FAST-LIVO2-ROS2 \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source Swarm-SLAM/s3e/env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .ros2/swarm/venv/bin/python -m pytest \
  Swarm-SLAM/src/cslam/tests Swarm-SLAM/s3e/test_registration.py \
  Swarm-SLAM/s3e/test_replay_tracking.py -q \
  Swarm-SLAM/s3e/test_frontend_quality.py \
  --junitxml=.ros2/swarm/provenance/tests.xml
ldd .ros2/swarm/install/cslam/lib/cslam/pose_graph_manager > .ros2/swarm/provenance/pgo-ldd.txt
printf 'Build and native tests complete.\n'
