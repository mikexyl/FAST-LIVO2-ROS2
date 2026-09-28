cmake_minimum_required(VERSION 3.22)
project(s3e_mixed_pgo VERSION 1.2.2 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(GTSAM 4.3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(nlohmann_json REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenSSL REQUIRED)
if(NOT EXISTS "${GTSAM_POINTS_SOURCE_DIR}/src/gtsam_points/factors/integrated_gicp_factor.cpp")
  message(FATAL_ERROR "Set GTSAM_POINTS_SOURCE_DIR to the pinned upstream checkout")
endif()
# Build the unmodified upstream CPU GICP component and its dependencies. The
# upstream extended ISAM2/LM optimizers depend on a different GTSAM development
# API; this adapter uses installed GTSAM's ordinary batch LM instead.
set(GTSAM_POINTS_GIT_HASH "9d32e7dbecf6015560d84b4901d6b0a6f483ec46")
file(REAL_PATH "${GTSAM_POINTS_SOURCE_DIR}" points_real_directory)
execute_process(COMMAND git -c "safe.directory=${points_real_directory}" -C "${GTSAM_POINTS_SOURCE_DIR}" rev-parse HEAD
  OUTPUT_VARIABLE points_revision OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
execute_process(COMMAND git -c "safe.directory=${points_real_directory}" -C "${GTSAM_POINTS_SOURCE_DIR}" status --porcelain
  OUTPUT_VARIABLE points_changes OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
if(NOT points_revision STREQUAL GTSAM_POINTS_GIT_HASH OR NOT points_changes STREQUAL "")
  message(FATAL_ERROR "Expected a clean pinned upstream gtsam_points v1.2.2 checkout")
endif()
set(GTSAM_POINTS_USE_OPENMP 1)
option(S3E_REGISTRATION_CUDA "Build GLIM's native CUDA VGICP factor path" OFF)
if(S3E_REGISTRATION_CUDA)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  set(CMAKE_CUDA_STANDARD 17)
  set(GTSAM_POINTS_USE_CUDA 1)
  set(GTSAM_POINTS_CUDA_VERSION ${CUDAToolkit_VERSION})
  set(GTSAM_POINTS_CUDA_VERSION_MAJOR ${CUDAToolkit_VERSION_MAJOR})
  set(GTSAM_POINTS_CUDA_VERSION_MINOR ${CUDAToolkit_VERSION_MINOR})
  set(GTSAM_POINTS_CUDA_VERSION_PATCH ${CUDAToolkit_VERSION_PATCH})
endif()
configure_file("${GTSAM_POINTS_SOURCE_DIR}/include/gtsam_points/config.hpp.in"
               "${CMAKE_CURRENT_BINARY_DIR}/include/gtsam_points/config.hpp")
set(points_sources
  util/parallelism.cpp features/covariance_estimation.cpp features/normal_estimation.cpp
  ann/kdtree.cpp ann/ivox.cpp ann/incremental_covariance_container.cpp ann/incremental_covariance_voxelmap.cpp
  types/point_cloud.cpp types/point_cloud_cpu.cpp
  factors/integrated_matching_cost_factor.cpp factors/integrated_gicp_factor.cpp)
list(TRANSFORM points_sources PREPEND "${GTSAM_POINTS_SOURCE_DIR}/src/gtsam_points/")
add_library(s3e_gtsam_points STATIC ${points_sources})
target_include_directories(s3e_gtsam_points PUBLIC compat
  "${GTSAM_POINTS_SOURCE_DIR}/include" "${GTSAM_POINTS_SOURCE_DIR}/thirdparty/nanoflann/include"
  "${CMAKE_CURRENT_BINARY_DIR}/include")
target_link_libraries(s3e_gtsam_points PUBLIC gtsam Eigen3::Eigen OpenMP::OpenMP_CXX Boost::filesystem OpenSSL::Crypto)
target_include_directories(s3e_gtsam_points PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
if(S3E_REGISTRATION_CUDA)
  set(gpu_sources
    cuda/check_error.cu cuda/cuda_memory.cu cuda/cuda_stream.cu cuda/cuda_buffer.cu
    cuda/cuda_device_sync.cu cuda/cuda_device_prop.cu cuda/cuda_graph.cu cuda/cuda_graph_exec.cu
    cuda/nonlinear_factor_set_gpu.cpp cuda/stream_roundrobin.cu cuda/stream_temp_buffer_roundrobin.cu
    types/point_cloud.cu types/point_cloud_gpu.cu types/gaussian_voxelmap_gpu.cu types/gaussian_voxelmap_gpu_funcs.cu
    factors/integrated_vgicp_derivatives.cu factors/integrated_vgicp_derivatives_inliers.cu
    factors/integrated_vgicp_derivatives_compute.cu factors/integrated_vgicp_derivatives_linearize.cu
    factors/integrated_vgicp_factor_gpu.cpp types/offloadable.cpp)
  list(TRANSFORM gpu_sources PREPEND "${GTSAM_POINTS_SOURCE_DIR}/src/gtsam_points/")
  target_sources(s3e_gtsam_points PRIVATE ${gpu_sources} vgicp_gpu_factor.cpp)
  target_link_libraries(s3e_gtsam_points PUBLIC CUDA::cudart)
  target_compile_options(s3e_gtsam_points PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)
  target_compile_definitions(s3e_gtsam_points PUBLIC S3E_REGISTRATION_CUDA)
endif()
option(S3E_BUILD_CENTRALIZED_PGO "Build the batch adapter executable" ON)
if(S3E_BUILD_CENTRALIZED_PGO)
add_executable(s3e_mixed_pgo mixed_pgo.cpp)
target_link_libraries(s3e_mixed_pgo PRIVATE s3e_gtsam_points nlohmann_json::nlohmann_json TBB::tbb)
target_include_directories(s3e_mixed_pgo PRIVATE compat)
# Keep the native GTSAM ABI outside the Python gtsam 4.2 process.
set_target_properties(s3e_mixed_pgo PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
target_link_options(s3e_mixed_pgo PRIVATE -Wl,--disable-new-dtags)
install(TARGETS s3e_mixed_pgo RUNTIME DESTINATION bin)
endif()
