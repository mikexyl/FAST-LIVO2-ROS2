#!/usr/bin/env bash
set -eo pipefail
cd /workspace
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=6 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
export CMAKE_PREFIX_PATH="/workspace/.ros2/swarm/native:/workspace/dependencies/rerun-sdk:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="/workspace/.ros2/swarm/native/lib:${LD_LIBRARY_PATH:-}"
# The base image has an older /usr/local GTSAM make_shared header. The pinned
# 4.3 build uses std::shared_ptr; give the existing compatibility shim priority.
export CXXFLAGS="-I/workspace/FAST-LIVO2-ROS2/research/adapters/gtsam_points/compat"
export PIP_CACHE_DIR=/workspace/.ros2/pip-cache
.ros2/research-venv/bin/pip install 'small-gicp==1.0.0' 'pybind11==3.1.0' 'pytest==9.1.1' 'psutil==7.2.2' 'cmake==3.31.10'
export PATH="/workspace/.ros2/research-venv/bin:$PATH"
mkdir -p .ros2/ellipsoid-cbs148/provenance
colcon --log-base .ros2/cbs-underlay-log build --base-paths dependencies/aria_common dependencies/aria_visualization dependencies/pose_graph_tools_msgs \
 --build-base .ros2/cbs-underlay-build --install-base .ros2/cbs-underlay --executor sequential \
 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_DISABLE_FIND_PACKAGE_g2o=ON -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source .ros2/cbs-underlay/setup.bash
mkdir -p .ros2/deps
ln -sfn /workspace/dependencies/gtsam_points .ros2/deps/gtsam_points
CBS_UNDERLAY=/workspace/.ros2/cbs-underlay S3E_REGISTRATION_FACTORS=ON bash FAST-LIVO2-ROS2/scripts/build_dpgo.sh
colcon --log-base .ros2/ellipse-log build --base-paths ellipselio --build-base .ros2/ellipse-build --install-base .ros2/ellipse-install \
 --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DS3E_RESEARCH_SOURCE=/workspace/FAST-LIVO2-ROS2 \
 -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
for variant in native inspection; do
  inspect=OFF; [ "$variant" = inspection ] && inspect=ON
  cmake -S FAST-LIVO2-ROS2/research/adapters/mapclosures -B .ros2/mapclosures-"$variant"-148 \
    -DCMAKE_BUILD_TYPE=Release -DS3E_INSPECTION_ONLY="$inspect" \
    -DPYTHON_EXECUTABLE=/workspace/.ros2/research-venv/bin/python \
    -Dpybind11_DIR=/workspace/.ros2/research-venv/lib/python3.10/site-packages/pybind11/share/cmake/pybind11 \
    -DPYTHON_MODULE_DIR=/workspace/.ros2/research-venv/lib/python3.10/site-packages \
    -DMAPCLOSURES_SOURCE=/workspace/dependencies/mapclosures -DHBST_SOURCE=/workspace/dependencies/mapclosures-hbst \
    -DSOPHUS_SOURCE=/workspace/dependencies/mapclosures-sophus
  cmake --build .ros2/mapclosures-"$variant"-148 --parallel 4
  cmake --install .ros2/mapclosures-"$variant"-148
done
mkdir -p .ros2/ellipsoid-cuda
/usr/local/cuda/bin/nvcc -O3 --fmad=false -arch=sm_120 -std=c++17 --shared -Xcompiler=-fPIC \
 FAST-LIVO2-ROS2/research/adapters/ellipsoid_cuda/surface.cu -o .ros2/ellipsoid-cuda/libellipsoid_surface.so
python3 -m venv .ros2/rerun-venv
.ros2/rerun-venv/bin/pip install 'rerun-sdk==0.37.1' 'numpy==2.2.6' 'scipy==1.15.3' 'PyYAML==6.0.3'
source .ros2/dpgo-install/setup.bash
ctest --test-dir .ros2/dpgo-build/cbs --output-on-failure
ctest --test-dir .ros2/dpgo-build/cbs_ros --output-on-failure
.ros2/research-venv/bin/pip freeze > .ros2/ellipsoid-cbs148/provenance/pip-freeze.txt
ldd .ros2/dpgo-install/cbs_ros/lib/cbs_ros/cbs_ros_node > .ros2/ellipsoid-cbs148/provenance/cbs-ldd.txt
