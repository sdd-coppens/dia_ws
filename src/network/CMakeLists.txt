cmake_minimum_required(VERSION 3.8)
project(network)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
elseif(MSVC)
  add_compile_options(/W4 /WX)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#setup network_domain
add_subdirectory(lib/network_domain)

# Setup source sets
set(RECEIVER_SOURCES src/core/ReceiverNode.h src/core/ReceiverNode.cpp)
set(SENDER_SOURCES src/core/SenderNode.h src/core/SenderNode.cpp)

# Add executables (nodes)
add_executable(receiver src/receiver.cpp ${RECEIVER_SOURCES})
add_executable(sender src/sender.cpp ${SENDER_SOURCES})

# Add the dependencies of each executable
ament_target_dependencies(receiver std_msgs rclcpp geometry_msgs)
target_link_libraries(receiver network_domain)

ament_target_dependencies(sender std_msgs rclcpp geometry_msgs)
target_link_libraries(sender network_domain)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  receiver
  sender
  DESTINATION lib/${PROJECT_NAME})

ament_package()
