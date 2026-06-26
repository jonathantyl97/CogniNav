cmake_minimum_required(VERSION 3.8)
project(cogninav_vslam)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

if(NOT DEFINED ENV{ORB_SLAM3_ROOT_DIR})
  set(ENV{ORB_SLAM3_ROOT_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/ORB_SLAM3")
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 QUIET)

if(NOT ORB_SLAM3_FOUND)
  message(WARNING
    "ORB-SLAM3 not found at $ENV{ORB_SLAM3_ROOT_DIR}. "
    "Run ./scripts/setup_deps.sh in the CogniNav Docker container, then rebuild.")
  ament_package()
  return()
endif()

set(SOPHUS_INCLUDE_DIR "${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus")

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${SOPHUS_INCLUDE_DIR}
)

add_executable(orb_slam3_stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(orb_slam3_stereo
  rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)
target_link_libraries(orb_slam3_stereo ${OpenCV_LIBS})

add_executable(orb_slam3_stereo_inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(orb_slam3_stereo_inertial
  rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)
target_link_libraries(orb_slam3_stereo_inertial ${OpenCV_LIBS})

install(TARGETS orb_slam3_stereo orb_slam3_stereo_inertial
  DESTINATION lib/${PROJECT_NAME})

ament_package()
