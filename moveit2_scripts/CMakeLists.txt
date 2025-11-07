cmake_minimum_required(VERSION 3.8)
project(moveit2_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(detection_interfaces REQUIRED)
find_package(moveit_visual_tools REQUIRED)



# Generate the executable
add_executable(pick_and_place
    src/pick_and_place.cpp)
add_executable(pick_and_place_real
    src/pick_and_place_real.cpp)
add_executable(pick_and_place_perception
	src/pick_and_place_perception.cpp)
add_executable(add_ur3_sim_scene
	src/add_ur3_sim_scene.cpp)
add_executable(move_to_pose
	src/move_to_pose.cpp)

target_include_directories(pick_and_place
    PUBLIC include)


ament_target_dependencies(pick_and_place
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)
ament_target_dependencies(pick_and_place_real
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)
ament_target_dependencies(pick_and_place_perception
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs
	detection_interfaces)
ament_target_dependencies(add_ur3_sim_scene
	ament_cmake
	rclcpp
	moveit_ros_planning_interface)
ament_target_dependencies(move_to_pose
	ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs)

# Install the executable
install(TARGETS
    pick_and_place
	pick_and_place_real
	pick_and_place_perception
	add_ur3_sim_scene
	move_to_pose
    DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME}
)

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
