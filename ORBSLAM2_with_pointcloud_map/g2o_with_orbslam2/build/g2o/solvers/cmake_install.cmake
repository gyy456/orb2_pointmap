# Install script for directory: /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/g2o/solvers

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/solvers/pcg/cmake_install.cmake")
  include("/home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/solvers/dense/cmake_install.cmake")
  include("/home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/solvers/structure_only/cmake_install.cmake")
  include("/home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/solvers/csparse/cmake_install.cmake")
  include("/home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/solvers/slam2d_linear/cmake_install.cmake")
  include("/home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/solvers/cholmod/cmake_install.cmake")
  include("/home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/solvers/eigen/cmake_install.cmake")

endif()
