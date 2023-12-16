cmake_minimum_required(VERSION 3.5)
project(experiment_nodes)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# setup sources
set(CAMERA_RECORDER_SOURCES 
     src/nodes/CameraRecorderNode.cpp
     src/nodes/CameraRecorderNode.hpp)

     # setup sources
set(IMU_RECORDER_SOURCES 
    src/nodes/ImuRecorderNode.cpp
    src/nodes/ImuRecorderNode.hpp)


add_executable(camera_recorder src/camera_interface.cpp ${CAMERA_RECORDER_SOURCES})
add_executable(imu_recorder src/imu_interface.cpp ${IMU_RECORDER_SOURCES})

ament_target_dependencies(camera_recorder rclcpp std_msgs OpenCV ament_index_cpp)
ament_target_dependencies(imu_recorder rclcpp std_msgs)

# Set include directories per executable
target_include_directories(camera_recorder PRIVATE src/nodes)
target_include_directories(imu_recorder PRIVATE src/nodes)


install(TARGETS
  camera_recorder
  imu_recorder
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
