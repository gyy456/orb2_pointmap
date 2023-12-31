cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)
find_package(PCL 1.8.1 REQUIRED )  
find_package(OpenCV 3.4.5 REQUIRED)
find_package(Eigen3 3.2.1 REQUIRED)
find_package(Boost COMPONENTS system)
find_package(Pangolin REQUIRED)
find_package( G2O REQUIRED )

include_directories(
${PROJECT_SOURCE_DIR}
${PCL_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${Boost_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/../../../../g2o_with_orbslam2
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sim3 g2o_types_sba
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
${PCL_LIBRARIES}
)

# Node for monocular camera
rosbuild_add_executable(Mono
src/ros_mono.cc
)

target_link_libraries(Mono
${LIBS}
)

# Node for RGB-D camera
rosbuild_add_executable(RGBD
src/ros_rgbd.cc
)
rosbuild_add_executable(RGBD_1
src/ros_rgbd_1.cc
)
target_link_libraries(RGBD
${LIBS}
)
target_link_libraries(RGBD_1
${LIBS}
)
# rosbuild_add_executable(pose_server2d
# src/pose_estimation_2d2d.cpp
# )
# target_link_libraries(pose_server2d
# ${LIBS}
# )
rosbuild_add_executable(pose_server3d
src/pose3d3d.cpp
)
target_link_libraries(pose_server3d
${LIBS}
)

rosbuild_add_executable(cloud_fusion
src/icp_1.cc
)
target_link_libraries(cloud_fusion
${LIBS}
)


