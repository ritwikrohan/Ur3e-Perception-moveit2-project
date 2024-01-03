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
find_package(grasping_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

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

# # Generate the executable
# add_executable(test_trajectory
#     src/test_trajectory.cpp)
# target_include_directories(test_trajectory
#     PUBLIC include)
# ament_target_dependencies(test_trajectory
#     ament_cmake
# 	rclcpp
# 	rclcpp_action
# 	moveit_core
# 	moveit_ros_planning_interface
# 	interactive_markers
# 	moveit_ros_planning
# 	control_msgs)

# add_executable(test_trajectory2
#     src/test_trajectory2.cpp)
# target_include_directories(test_trajectory2
#     PUBLIC include)
# ament_target_dependencies(test_trajectory2
#     ament_cmake
# 	rclcpp
# 	rclcpp_action
# 	moveit_core
# 	moveit_ros_planning_interface
# 	interactive_markers
# 	moveit_ros_planning
# 	control_msgs)

# add_executable(test_trajectory3
#     src/test_trajectory3.cpp)
# target_include_directories(test_trajectory3
#     PUBLIC include)
# ament_target_dependencies(test_trajectory3
#     ament_cmake
# 	rclcpp
# 	rclcpp_action
# 	moveit_core
# 	moveit_ros_planning_interface
# 	interactive_markers
# 	moveit_ros_planning
# 	control_msgs)

# install(TARGETS 
# 	test_trajectory
# 	test_trajectory2
# 	test_trajectory2
#     DESTINATION lib/${PROJECT_NAME}
# )

# add_executable(test_gripper
#     src/test_gripper.cpp)
# target_include_directories(test_gripper
#     PUBLIC include)
# ament_target_dependencies(test_gripper
#     ament_cmake
# 	rclcpp
# 	rclcpp_action
# 	moveit_core
# 	moveit_ros_planning_interface
# 	interactive_markers
# 	moveit_ros_planning
# 	control_msgs)

# # Install the executable
# install(TARGETS 
# 	test_gripper
#     DESTINATION lib/${PROJECT_NAME}
# )

# add_executable(approach_retreat
#     src/approach_retreat.cpp)
# target_include_directories(approach_retreat
#     PUBLIC include)
# ament_target_dependencies(approach_retreat
#     ament_cmake
# 	rclcpp
# 	rclcpp_action
# 	moveit_core
# 	moveit_ros_planning_interface
# 	interactive_markers
# 	moveit_ros_planning
# 	control_msgs)

# # Install the executable
# install(TARGETS 
# 	approach_retreat
#     DESTINATION lib/${PROJECT_NAME}
# )
# # Generate the executable
# add_executable(cartesian_path
#     src/cartesian_path.cpp)
# target_include_directories(cartesian_path
#     PUBLIC include)
# ament_target_dependencies(cartesian_path
#     ament_cmake
# 	rclcpp
# 	rclcpp_action
# 	moveit_core
# 	moveit_ros_planning_interface
# 	interactive_markers
# 	moveit_ros_planning
# 	control_msgs
# 	grasping_msgs)

# # Install the executable
# install(TARGETS 
# 	cartesian_path
#     DESTINATION lib/${PROJECT_NAME}
# )

# # Generate the executable
# add_executable(test_kinematics_plugin
#     src/test_kinematics_plugin.cpp)
# target_include_directories(test_kinematics_plugin
#     PUBLIC include)
# ament_target_dependencies(test_kinematics_plugin
#     ament_cmake
# 	rclcpp
# 	rclcpp_action
# 	moveit_core
# 	moveit_ros_planning_interface
# 	interactive_markers
# 	moveit_ros_planning
# 	control_msgs
# 	grasping_msgs)

# # Install the executable
# install(TARGETS 
# 	test_kinematics_plugin
#     DESTINATION lib/${PROJECT_NAME}
# )

# Generate the executable
add_executable(pick_place
    src/pick_place.cpp)
target_include_directories(pick_place
    PUBLIC include)
ament_target_dependencies(pick_place
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	grasping_msgs
	shape_msgs)

# Install the executable
install(TARGETS 
	pick_place
    DESTINATION lib/${PROJECT_NAME}
)



# Generate the executable
add_executable(pick_and_place_perception
    src/pick_and_place_perception.cpp)
target_include_directories(pick_and_place_perception
    PUBLIC include)
ament_target_dependencies(pick_and_place_perception
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	grasping_msgs
	shape_msgs)

# Install the executable
install(TARGETS 
	pick_and_place_perception
    DESTINATION lib/${PROJECT_NAME}
)

# Generate the executable
add_executable(pick_and_place_perception_real
    src/pick_and_place_perception_real.cpp)
target_include_directories(pick_and_place_perception_real
    PUBLIC include)
ament_target_dependencies(pick_and_place_perception_real
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	grasping_msgs
	shape_msgs)

# Install the executable
install(TARGETS 
	pick_and_place_perception_real
    DESTINATION lib/${PROJECT_NAME}
)

# Generate the executable
add_executable(collision_objects
    src/collision_objects.cpp)
target_include_directories(collision_objects
    PUBLIC include)
ament_target_dependencies(collision_objects
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	grasping_msgs
	shape_msgs)

# Install the executable
install(TARGETS 
	collision_objects
    DESTINATION lib/${PROJECT_NAME}
)


# Install the launch file
install(DIRECTORY 
	launch
    DESTINATION share/${PROJECT_NAME}
)


ament_package()
