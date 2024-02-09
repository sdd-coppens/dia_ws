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
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(rosbag2_cpp REQUIRED)
find_package(custom_controller_interfaces REQUIRED)
find_package(xarm_sdk REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(xarm_planner REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(moveit REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)


add_executable(keyboard src/keyboard.cpp)
ament_target_dependencies(keyboard rclcpp std_msgs)

add_executable(new_novint src/new_novint.cpp)
ament_target_dependencies(new_novint rclcpp xarm_sdk std_msgs geometry_msgs custom_controller_interfaces tf2 tf2_ros)

add_executable(compliant_loadcell src/compliant_loadcell.cpp)
ament_target_dependencies(compliant_loadcell rclcpp xarm_sdk std_msgs geometry_msgs custom_controller_interfaces)

add_executable(operator_domain_manip src/operator_domain_manip.cpp)
ament_target_dependencies(operator_domain_manip rclcpp custom_controller_interfaces std_msgs xarm_sdk xarm_msgs tf2 tf2_ros)

add_executable(spinning src/spinning.cpp)
ament_target_dependencies(spinning rclcpp custom_controller_interfaces std_msgs xarm_sdk xarm_msgs)

add_executable(arduino_angle_listener src/arduino_angle_listener.cpp)
ament_target_dependencies(arduino_angle_listener rclcpp xarm_sdk std_msgs geometry_msgs custom_controller_interfaces tf2 tf2_ros)

add_executable(websocket_arduino src/websocket_arduino.cpp)
ament_target_dependencies(websocket_arduino rclcpp std_msgs geometry_msgs tf2 tf2_ros custom_controller_interfaces)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
  keyboard
  new_novint
  operator_domain_manip
  spinning
  compliant_loadcell
  arduino_angle_listener
  websocket_arduino
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


ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/fk_vector_predict.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
