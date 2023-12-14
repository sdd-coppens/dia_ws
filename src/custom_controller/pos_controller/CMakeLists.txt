cmake_minimum_required(VERSION 3.8)
project(pos_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-variable)
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wno-narrowing)
  add_compile_options(-Wno-unused-parameter)
  add_compile_options(-Wno-unused-variable)
  add_compile_options(-Wno-unused-but-set-variable)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(custom_controller_interfaces REQUIRED)
find_package(xarm_sdk REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(xarm_planner REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(moveit REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

# find_package(lodepng REQUIRED)

add_library(lodepng STATIC
  src/lodepng/lodepng.cpp
  # Add other source files if there are any
)
target_include_directories(lodepng PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lodepng
)

# include_directories(/home/stijn/thesis/xarm_ws/install/xarm_planner/include)

# link_directories(/home/stijn/thesis/xarm_ws/install/xarm_planner/lib/libxarm_planner.so)

# ament_export_include_directories(xarm_planner/include)

add_executable(box_pos_controller src/box_pos_controller.cpp)
ament_target_dependencies(box_pos_controller rclcpp custom_controller_interfaces std_msgs xarm_sdk xarm_msgs)#xarm_planner xarm_msgs moveit moveit_ros_planning_interface moveit_msgs)

add_executable(publisher_temp src/publisher_temp.cpp)
ament_target_dependencies(publisher_temp rclcpp custom_controller_interfaces std_msgs xarm_sdk xarm_msgs)

add_executable(keyboard src/keyboard.cpp)
ament_target_dependencies(keyboard rclcpp std_msgs)

add_executable(temp src/temp.cpp)
ament_target_dependencies(temp rclcpp std_msgs xarm_msgs)

add_executable(test_controller_jog src/test_controller_jog.cpp)
ament_target_dependencies(test_controller_jog rclcpp std_msgs custom_controller_interfaces xarm_sdk)

add_executable(moveit_test src/moveit_test.cpp)
ament_target_dependencies(moveit_test rclcpp std_msgs moveit moveit_msgs moveit_ros_planning_interface)

add_executable(novint_falcon_test src/novint_falcon_test.cpp)
ament_target_dependencies(novint_falcon_test rclcpp xarm_sdk geometry_msgs)

add_executable(new_novint src/new_novint.cpp)
ament_target_dependencies(new_novint rclcpp xarm_sdk std_msgs geometry_msgs custom_controller_interfaces tf2 tf2_ros)

add_executable(moveit_test2 src/moveit_test2.cpp)
ament_target_dependencies(moveit_test2 rclcpp std_msgs moveit moveit_msgs moveit_ros_planning_interface)

add_executable(camera_tracking_node src/camera_tracking_node.cpp)
ament_target_dependencies(camera_tracking_node rclcpp custom_controller_interfaces xarm_sdk xarm_msgs)

add_executable(pre_programmed_path src/pre_programmed_path.cpp)
ament_target_dependencies(pre_programmed_path rclcpp xarm_sdk rosbag2_cpp std_msgs)

add_executable(operator_domain_manip src/operator_domain_manip.cpp)
ament_target_dependencies(operator_domain_manip rclcpp custom_controller_interfaces std_msgs xarm_sdk xarm_msgs)

add_executable(draw_png src/draw_png.cpp)
ament_target_dependencies(draw_png rclcpp xarm_sdk)
target_link_libraries(draw_png lodepng)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
  box_pos_controller
  keyboard
  temp
  publisher_temp
  test_controller_jog
  moveit_test
  novint_falcon_test
  moveit_test2
  camera_tracking_node
  pre_programmed_path
  new_novint
  draw_png
  operator_domain_manip
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
