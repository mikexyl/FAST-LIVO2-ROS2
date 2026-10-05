cmake_minimum_required(VERSION 3.16)
project(cbs VERSION 1.0 DESCRIPTION "CBS - Collaborative Belief Space" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-sign-compare")
add_compile_options(-Wno-unused-parameter -Wno-unused-function)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

message(STATUS "aria_common dir: ${aria_common_DIR}")

find_package(GTSAM 4.3 REQUIRED)
find_package(TBB 4.4 COMPONENTS tbb tbbmalloc)
find_package(aria_common REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

option(RAIDO_USE_BATCH_SAM_WRAPPER "Use Batch SAM Wrapper" OFF)
option(CBS_BUILD_UTILS "Build CBS command-line utility executables" OFF)
option(CBS_BUILD_EXAMPLES "Build CBS example executables" OFF)

get_target_property(aria_common_LIBRARIES aria_common LOCATION)
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "aria_common_LIBRARIES: ${aria_common_LIBRARIES}")

add_subdirectory(thirdparty/gkcm)

set(INCLUDES
    ${TBB_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
    src/bpsam/bpsam.cpp
    src/gbp/gaussian.cpp
    src/robust/inter_robot_pcm.cpp
    src/robust/gkcm_wrapper.cpp)
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    gtsam
    TBB::tbb TBB::tbbmalloc
    aria_common
    spdlog::spdlog
    gkcm
    pcm)
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${INCLUDES}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/gkcm/src>)
target_compile_definitions(${PROJECT_NAME} PUBLIC TBB_USE_THREADING_TOOLS)

if(CBS_BUILD_UTILS)
    add_subdirectory(src/utils)
endif()
if(CBS_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(test_persistent_anchor_relay tests/test_persistent_anchor_relay.cpp)
  target_link_libraries(test_persistent_anchor_relay PRIVATE ${PROJECT_NAME})
  add_test(NAME persistent_anchor_relay COMMAND test_persistent_anchor_relay)
  add_executable(test_anchor_bootstrap tests/test_anchor_bootstrap.cpp)
  target_link_libraries(test_anchor_bootstrap PRIVATE ${PROJECT_NAME})
  add_test(NAME anchor_bootstrap COMMAND test_anchor_bootstrap)
  add_executable(test_persistent_beliefs tests/test_persistent_beliefs.cpp)
  target_link_libraries(test_persistent_beliefs PRIVATE ${PROJECT_NAME})
  add_test(NAME persistent_beliefs COMMAND test_persistent_beliefs)
  add_executable(test_inter_robot_pcm tests/test_inter_robot_pcm.cpp)
  target_link_libraries(test_inter_robot_pcm PRIVATE ${PROJECT_NAME})
  add_test(NAME inter_robot_pcm COMMAND test_inter_robot_pcm)
  add_executable(test_between_initialization tests/test_between_initialization.cpp)
  target_link_libraries(test_between_initialization PRIVATE ${PROJECT_NAME})
  add_test(NAME between_initialization COMMAND test_between_initialization)

  add_executable(test_sim3_transport tests/test_sim3_transport.cpp)
  target_link_libraries(test_sim3_transport PRIVATE ${PROJECT_NAME})
  add_test(NAME sim3_transport COMMAND test_sim3_transport)

  add_executable(test_belief_republish tests/test_belief_republish.cpp)
  target_link_libraries(test_belief_republish PRIVATE ${PROJECT_NAME})
  add_test(NAME belief_republish COMMAND test_belief_republish)

  add_executable(test_absolute_pose_scale
      tests/test_absolute_pose_scale.cpp)
  target_link_libraries(test_absolute_pose_scale PRIVATE ${PROJECT_NAME})
  add_test(NAME absolute_pose_scale COMMAND test_absolute_pose_scale)
endif()

# If Google Test requires pthread, uncomment this
# find_package(Threads REQUIRED)
# target_link_libraries(${test_name} PRIVATE Threads::Threads)
install(TARGETS cbs gkcm gen_max_clique lie pcm
    EXPORT cbsTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

# Install the public headers
install(DIRECTORY include/cbs
    DESTINATION include)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/cbsConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

export(TARGETS cbs gkcm gen_max_clique lie pcm
    FILE "${CMAKE_CURRENT_BINARY_DIR}/cbsTargets.cmake")

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

set(ConfigPackageLocation lib/cmake/cbs)
install(EXPORT cbsTargets
    FILE cbsTargets.cmake
    DESTINATION ${ConfigPackageLocation})

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/cbsConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/cbsConfigVersion.cmake"
    DESTINATION ${ConfigPackageLocation})
