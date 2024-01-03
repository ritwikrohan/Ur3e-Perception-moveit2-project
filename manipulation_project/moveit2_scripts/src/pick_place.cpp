// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto move_group_node =
//       rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(move_group_node);
//   std::thread([&executor]() { executor.spin(); }).detach();

	

//   static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
//   static const std::string PLANNING_GROUP_GRIPPER = "gripper";

//   moveit::planning_interface::MoveGroupInterface move_group_arm(
//       move_group_node, PLANNING_GROUP_ARM);
//   moveit::planning_interface::MoveGroupInterface move_group_gripper(
//       move_group_node, PLANNING_GROUP_GRIPPER);


// 	// // Create collision object for the robot to avoid
// 	// auto const collision_object = [frame_id =
// 	// 																move_group_arm.getPlanningFrame()] {
// 	// 	moveit_msgs::msg::CollisionObject collision_object;
// 	// 	collision_object.header.frame_id = frame_id;
// 	// 	collision_object.id = "box1";
// 	// 	shape_msgs::msg::SolidPrimitive primitive;

// 	// 	// Define the size of the box in meters
// 	// 	primitive.type = primitive.BOX;
// 	// 	primitive.dimensions.resize(3);
// 	// 	primitive.dimensions[primitive.BOX_X] = 1.0;
// 	// 	primitive.dimensions[primitive.BOX_Y] = 0.4;
// 	// 	primitive.dimensions[primitive.BOX_Z] = 0.03;

// 	// 	// Define the pose of the box (relative to the frame_id)
// 	// 	geometry_msgs::msg::Pose box_pose;
// 	// 	box_pose.orientation.w = 0.707;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0 
// 	// 	box_pose.position.x = 0.4;
// 	// 	box_pose.position.y = 0.1;
// 	// 	box_pose.position.z = -0.03;

// 	// 	collision_object.primitives.push_back(primitive);
// 	// 	collision_object.primitive_poses.push_back(box_pose);
// 	// 	collision_object.operation = collision_object.ADD;

// 	// 	return collision_object;
// 	// }();
	
// 	// // Add the collision object to the scene
// 	// moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
// 	// planning_scene_interface.applyCollisionObject(collision_object);

// 	// // Create collision object for the robot to avoid
// 	// auto const collision_object_new = [frame_id =
// 	// 																move_group_arm.getPlanningFrame()] {
// 	// 	moveit_msgs::msg::CollisionObject collision_object_new;
// 	// 	collision_object_new.header.frame_id = frame_id;
// 	// 	collision_object_new.id = "box2";
// 	// 	shape_msgs::msg::SolidPrimitive primitive;

// 	// 	// Define the size of the box in meters
// 	// 	primitive.type = primitive.BOX;
// 	// 	primitive.dimensions.resize(3);
// 	// 	primitive.dimensions[primitive.BOX_X] = 1.5;
// 	// 	primitive.dimensions[primitive.BOX_Y] = 1.5;
// 	// 	primitive.dimensions[primitive.BOX_Z] = 0.01;

// 	// 	// Define the pose of the box (relative to the frame_id)
// 	// 	geometry_msgs::msg::Pose box_pose;
// 	// 	box_pose.orientation.w = 0.707;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
// 	// 	box_pose.orientation.x = -0.707;
// 	// 	box_pose.orientation.y = 0.0; 
// 	// 	box_pose.orientation.z = 0.0;  
// 	// 	box_pose.position.x = 0.0;
// 	// 	box_pose.position.y = 0.5;
// 	// 	box_pose.position.z = 0.55;

// 	// 	collision_object_new.primitives.push_back(primitive);
// 	// 	collision_object_new.primitive_poses.push_back(box_pose);
// 	// 	collision_object_new.operation = collision_object_new.ADD;

// 	// 	return collision_object_new;
// 	// }();
	
// 	// // Add the collision object to the scene
// 	// // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
// 	// planning_scene_interface.applyCollisionObject(collision_object_new);

//   const moveit::core::JointModelGroup *joint_model_group_arm =
//       move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
//   const moveit::core::JointModelGroup *joint_model_group_gripper =
//       move_group_gripper.getCurrentState()->getJointModelGroup(
//           PLANNING_GROUP_GRIPPER);

//   // Get Current State
//   moveit::core::RobotStatePtr current_state_arm =
//       move_group_arm.getCurrentState(10);
//   moveit::core::RobotStatePtr current_state_gripper =
//       move_group_gripper.getCurrentState(10);

//   std::vector<double> joint_group_positions_arm;
//   std::vector<double> joint_group_positions_gripper;
//   current_state_arm->copyJointGroupPositions(joint_model_group_arm,
//                                              joint_group_positions_arm);
//   current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
//                                                  joint_group_positions_gripper);

//   move_group_arm.setStartStateToCurrentState();
//   move_group_gripper.setStartStateToCurrentState();

//   // Go Home
//   // RCLCPP_INFO(LOGGER, "Going Home");
// 	RCLCPP_INFO(LOGGER, "Pregrasp Position");

//   joint_group_positions_arm[0] = -0.457655;  // Shoulder Pan
//   joint_group_positions_arm[1] = -1.463627; // Shoulder Lift
//   joint_group_positions_arm[2] = 1.744042;  // Elbow
//   joint_group_positions_arm[3] = -1.867829; // Wrist 1
//   joint_group_positions_arm[4] = -1.539464; // Wrist 2
//   joint_group_positions_arm[5] = -0.538744;  // Wrist 3

//   move_group_arm.setJointValueTarget(joint_group_positions_arm);

//   moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
//   bool success_arm = (move_group_arm.plan(my_plan_arm) ==
//                       moveit::core::MoveItErrorCode::SUCCESS);

//   move_group_arm.execute(my_plan_arm);


//     //   - -6.280320802665661
//     // - -1.3739757302375497
//     // - 1.5373043889848415
//     // - 4.711669087612103
//     // - -1.568440109738244
//     // - -1.734407002054146


// //     - 5.891114583166587
// // - -1.4804764749713488
// // - 1.661024452016
// // - -1.5704747624281774
// // - 4.319779756898502
// // - 4.531864153441864
//   // Pregrasp
//   RCLCPP_INFO(LOGGER, "Pregrasp Position");

// 	// joint_group_positions_arm[0] = 5.891114583166587;  // Shoulder Pan
//   // joint_group_positions_arm[1] = -1.4804764749713488; // Shoulder Lift
//   // joint_group_positions_arm[2] = 1.661024452016;  // Elbow
//   // joint_group_positions_arm[3] = -1.5704747624281774; // Wrist 1
//   // joint_group_positions_arm[4] = 4.319779756898502; // Wrist 2
//   // joint_group_positions_arm[5] = 4.531864153441864;  // Wrist 3

//   // move_group_arm.setJointValueTarget(joint_group_positions_arm);

//   // // moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
//   // success_arm = (move_group_arm.plan(my_plan_arm) ==
//   //                     moveit::core::MoveItErrorCode::SUCCESS);

//   // move_group_arm.execute(my_plan_arm);

//   // target_pose1.orientation.x = 0.707;
//   // target_pose1.orientation.y = -0.707;
//   // target_pose1.orientation.z = 0.00;
//   // target_pose1.orientation.w = 0.00;
//   // target_pose1.position.x = 0.343;
//   // target_pose1.position.y = 0.0;
//   // target_pose1.position.z = 0.264;
//   // move_group_arm.setPoseTarget(target_pose1);

//   // success_arm = (move_group_arm.plan(my_plan_arm) ==
//   //                moveit::core::MoveItErrorCode::SUCCESS);

//   // move_group_arm.execute(my_plan_arm);

//   // Open Gripper

//   RCLCPP_INFO(LOGGER, "Open Gripper!");

//   move_group_gripper.setNamedTarget("gripper_open");

//   moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
//   bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
//                           moveit::core::MoveItErrorCode::SUCCESS);

//   move_group_gripper.execute(my_plan_gripper);

//   // Approach
//   RCLCPP_INFO(LOGGER, "Approach to object!");
//   geometry_msgs::msg::Pose target_pose1;

//   std::vector<geometry_msgs::msg::Pose> approach_waypoints;
// 	target_pose1.orientation.x = 0.707;
//   target_pose1.orientation.y = -0.707;
//   target_pose1.orientation.z = 0.00;
//   target_pose1.orientation.w = 0.00;
//   target_pose1.position.x = 0.34;
//   target_pose1.position.y = -0.020;
//   target_pose1.position.z = 0.2;
//   approach_waypoints.push_back(target_pose1);


// 	target_pose1.orientation.x = 0.707;
//   target_pose1.orientation.y = -0.707;
//   target_pose1.orientation.z = 0.00;
//   target_pose1.orientation.w = 0.00;
//   target_pose1.position.x = 0.34;
//   target_pose1.position.y = -0.020;
//   target_pose1.position.z = 0.18;
//   approach_waypoints.push_back(target_pose1);

//   moveit_msgs::msg::RobotTrajectory trajectory_approach;
//   const double jump_threshold = 0.0;
//   const double eef_step = 0.01;

//   double fraction = move_group_arm.computeCartesianPath(
//       approach_waypoints, eef_step, jump_threshold, trajectory_approach);

//   move_group_arm.execute(trajectory_approach);

//   // Close Gripper

//   RCLCPP_INFO(LOGGER, "Close Gripper!");

//   move_group_gripper.setNamedTarget("gripper_close");

//   success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
//                      moveit::core::MoveItErrorCode::SUCCESS);

//   move_group_gripper.execute(my_plan_gripper);

//   // Retreat

//   RCLCPP_INFO(LOGGER, "Retreat from object!");

//   std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
//   // std::vector<geometry_msgs::msg::Pose> approach_waypoints;
// 	target_pose1.orientation.x = 0.707;
//   target_pose1.orientation.y = -0.707;
//   target_pose1.orientation.z = 0.00;
//   target_pose1.orientation.w = 0.00;
//   target_pose1.position.x = 0.34;
//   target_pose1.position.y = -0.020;
//   target_pose1.position.z = 0.2;
//   retreat_waypoints.push_back(target_pose1);


// 	target_pose1.orientation.x = 0.707;
//   target_pose1.orientation.y = -0.707;
//   target_pose1.orientation.z = 0.00;
//   target_pose1.orientation.w = 0.00;
//   target_pose1.position.x = 0.34;
//   target_pose1.position.y = -0.020;
//   target_pose1.position.z = 0.25;
//   retreat_waypoints.push_back(target_pose1);

//   moveit_msgs::msg::RobotTrajectory trajectory_retreat;
//   // const double jump_threshold = 0.0;
//   // const double eef_step = 0.01;

//   fraction = move_group_arm.computeCartesianPath(
//       retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

//   move_group_arm.execute(trajectory_retreat);

//   // Place

//   RCLCPP_INFO(LOGGER, "Rotating Arm");

//   current_state_arm = move_group_arm.getCurrentState(10);
//   current_state_arm->copyJointGroupPositions(joint_model_group_arm,
//                                              joint_group_positions_arm);

//   joint_group_positions_arm[0] = 3.14; // Shoulder Pan

//   move_group_arm.setJointValueTarget(joint_group_positions_arm);

//   success_arm = (move_group_arm.plan(my_plan_arm) ==
//                  moveit::core::MoveItErrorCode::SUCCESS);

//   move_group_arm.execute(my_plan_arm);

//   // Open Gripper

//   RCLCPP_INFO(LOGGER, "Release Object!");

//   move_group_gripper.setNamedTarget("gripper_open");

//   success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
//                      moveit::core::MoveItErrorCode::SUCCESS);

//   move_group_gripper.execute(my_plan_gripper);

//   rclcpp::shutdown();
//   return 0;
// }




/////////////////////////////////////////REAL////////////////////////////////////////////////////////////////



#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

	

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);


	// // Create collision object for the robot to avoid
	// auto const collision_object = [frame_id =
	// 																move_group_arm.getPlanningFrame()] {
	// 	moveit_msgs::msg::CollisionObject collision_object;
	// 	collision_object.header.frame_id = frame_id;
	// 	collision_object.id = "box1";
	// 	shape_msgs::msg::SolidPrimitive primitive;

	// 	// Define the size of the box in meters
	// 	primitive.type = primitive.BOX;
	// 	primitive.dimensions.resize(3);
	// 	primitive.dimensions[primitive.BOX_X] = 1.0;
	// 	primitive.dimensions[primitive.BOX_Y] = 0.4;
	// 	primitive.dimensions[primitive.BOX_Z] = 0.03;

	// 	// Define the pose of the box (relative to the frame_id)
	// 	geometry_msgs::msg::Pose box_pose;
	// 	box_pose.orientation.w = 0.707;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0 
	// 	box_pose.position.x = 0.4;
	// 	box_pose.position.y = 0.1;
	// 	box_pose.position.z = -0.03;

	// 	collision_object.primitives.push_back(primitive);
	// 	collision_object.primitive_poses.push_back(box_pose);
	// 	collision_object.operation = collision_object.ADD;

	// 	return collision_object;
	// }();
	
	// // Add the collision object to the scene
	// moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	// planning_scene_interface.applyCollisionObject(collision_object);

	// // Create collision object for the robot to avoid
	// auto const collision_object_new = [frame_id =
	// 																move_group_arm.getPlanningFrame()] {
	// 	moveit_msgs::msg::CollisionObject collision_object_new;
	// 	collision_object_new.header.frame_id = frame_id;
	// 	collision_object_new.id = "box2";
	// 	shape_msgs::msg::SolidPrimitive primitive;

	// 	// Define the size of the box in meters
	// 	primitive.type = primitive.BOX;
	// 	primitive.dimensions.resize(3);
	// 	primitive.dimensions[primitive.BOX_X] = 1.5;
	// 	primitive.dimensions[primitive.BOX_Y] = 1.5;
	// 	primitive.dimensions[primitive.BOX_Z] = 0.01;

	// 	// Define the pose of the box (relative to the frame_id)
	// 	geometry_msgs::msg::Pose box_pose;
	// 	box_pose.orientation.w = 0.707;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
	// 	box_pose.orientation.x = -0.707;
	// 	box_pose.orientation.y = 0.0; 
	// 	box_pose.orientation.z = 0.0;  
	// 	box_pose.position.x = 0.0;
	// 	box_pose.position.y = 0.5;
	// 	box_pose.position.z = 0.55;

	// 	collision_object_new.primitives.push_back(primitive);
	// 	collision_object_new.primitive_poses.push_back(box_pose);
	// 	collision_object_new.operation = collision_object_new.ADD;

	// 	return collision_object_new;
	// }();
	
	// // Add the collision object to the scene
	// // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	// planning_scene_interface.applyCollisionObject(collision_object_new);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();

  // Go Home
  // RCLCPP_INFO(LOGGER, "Going Home");
	RCLCPP_INFO(LOGGER, "Pregrasp Position");

  joint_group_positions_arm[0] = 0.00; //-0.457655;  // Shoulder Pan
  joint_group_positions_arm[1] = -1.396; //-1.463627; // Shoulder Lift
  joint_group_positions_arm[2] = 1.260653;  // Elbow
  joint_group_positions_arm[3] = -1.5; // Wrist 1
  joint_group_positions_arm[4] = -1.55; // Wrist 2
  joint_group_positions_arm[5] = 0.00;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);


    //   - -6.280320802665661
    // - -1.3739757302375497
    // - 1.5373043889848415
    // - 4.711669087612103
    // - -1.568440109738244
    // - -1.734407002054146


//     - 5.891114583166587
// - -1.4804764749713488
// - 1.661024452016
// - -1.5704747624281774
// - 4.319779756898502
// - 4.531864153441864
  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

	// joint_group_positions_arm[0] = 5.891114583166587;  // Shoulder Pan
  // joint_group_positions_arm[1] = -1.4804764749713488; // Shoulder Lift
  // joint_group_positions_arm[2] = 1.661024452016;  // Elbow
  // joint_group_positions_arm[3] = -1.5704747624281774; // Wrist 1
  // joint_group_positions_arm[4] = 4.319779756898502; // Wrist 2
  // joint_group_positions_arm[5] = 4.531864153441864;  // Wrist 3

  // move_group_arm.setJointValueTarget(joint_group_positions_arm);

  // // moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  // success_arm = (move_group_arm.plan(my_plan_arm) ==
  //                     moveit::core::MoveItErrorCode::SUCCESS);

  // move_group_arm.execute(my_plan_arm);

  // target_pose1.orientation.x = 0.707;
  // target_pose1.orientation.y = -0.707;
  // target_pose1.orientation.z = 0.00;
  // target_pose1.orientation.w = 0.00;
  // target_pose1.position.x = 0.343;
  // target_pose1.position.y = 0.0;
  // target_pose1.position.z = 0.264;
  // move_group_arm.setPoseTarget(target_pose1);

  // success_arm = (move_group_arm.plan(my_plan_arm) ==
  //                moveit::core::MoveItErrorCode::SUCCESS);

  // move_group_arm.execute(my_plan_arm);

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Open Gripper!");

  move_group_gripper.setNamedTarget("gripper_open");
//   joint_group_positions_gripper[0] = 0.645;
//   move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

//   // Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");
  geometry_msgs::msg::Pose target_pose1;

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
	target_pose1.orientation.x = 0.707;
  target_pose1.orientation.y = -0.707;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  target_pose1.position.x = 0.343;
  target_pose1.position.y = 0.132;
  target_pose1.position.z = 0.30;
  approach_waypoints.push_back(target_pose1);


	target_pose1.orientation.x = 0.707;
  target_pose1.orientation.y = -0.707;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  target_pose1.position.x = 0.343;
  target_pose1.position.y = 0.132;
  target_pose1.position.z = 0.25;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);

//   // Close Gripper

  RCLCPP_INFO(LOGGER, "Close Gripper!");

  move_group_gripper.setNamedTarget("gripper_close");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

//   // Retreat

  RCLCPP_INFO(LOGGER, "Retreat from object!");

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  // std::vector<geometry_msgs::msg::Pose> approach_waypoints;
	target_pose1.orientation.x = 0.707;
  target_pose1.orientation.y = -0.707;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  target_pose1.position.x = 0.343;
  target_pose1.position.y = 0.132;
  target_pose1.position.z = 0.30;
  retreat_waypoints.push_back(target_pose1);


	target_pose1.orientation.x = 0.707;
  target_pose1.orientation.y = -0.707;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  target_pose1.position.x = 0.343;
  target_pose1.position.y = 0.132;
  target_pose1.position.z = 0.35;
  retreat_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;
  // const double jump_threshold = 0.0;
  // const double eef_step = 0.01;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  move_group_arm.execute(trajectory_retreat);

//   // Place

  RCLCPP_INFO(LOGGER, "Rotating Arm");

  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  joint_group_positions_arm[0] = 3.14; // Shoulder Pan

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Release Object!");

  move_group_gripper.setNamedTarget("gripper_open");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  rclcpp::shutdown();
  return 0;
}