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
      rclcpp::Node::make_shared("collision_objects", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

	

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);


	// Create collision object for the robot to avoid
	auto const collision_object = [frame_id = move_group_arm.getPlanningFrame()] {
		moveit_msgs::msg::CollisionObject collision_object;
		collision_object.header.frame_id = frame_id;
		collision_object.id = "box1";
		shape_msgs::msg::SolidPrimitive primitive;

		// Define the size of the box in meters
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[primitive.BOX_X] = 1.0;
		primitive.dimensions[primitive.BOX_Y] = 0.4;
		primitive.dimensions[primitive.BOX_Z] = 0.03;

		// Define the pose of the box (relative to the frame_id)
		geometry_msgs::msg::Pose box_pose;
		box_pose.orientation.w = 0.707;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0 
		box_pose.position.x = 0.4;
		box_pose.position.y = 0.1;
		box_pose.position.z = -0.03;

		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(box_pose);
		collision_object.operation = collision_object.ADD;

		return collision_object;
	}();
	
	// Add the collision object to the scene
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	planning_scene_interface.applyCollisionObject(collision_object);

	// Create collision object for the robot to avoid
	auto const collision_object_new = [frame_id = move_group_arm.getPlanningFrame()] {
		moveit_msgs::msg::CollisionObject collision_object_new;
		collision_object_new.header.frame_id = frame_id;
		collision_object_new.id = "box2";
		shape_msgs::msg::SolidPrimitive primitive;

		// Define the size of the box in meters
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[primitive.BOX_X] = 1.5;
		primitive.dimensions[primitive.BOX_Y] = 1.5;
		primitive.dimensions[primitive.BOX_Z] = 0.01;

		// Define the pose of the box (relative to the frame_id)
		geometry_msgs::msg::Pose box_pose;
		box_pose.orientation.w = 0.707;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
		box_pose.orientation.x = -0.707;
		box_pose.orientation.y = 0.0; 
		box_pose.orientation.z = 0.0;  
		box_pose.position.x = 0.0;
		box_pose.position.y = 0.5;
		box_pose.position.z = 0.55;

		collision_object_new.primitives.push_back(primitive);
		collision_object_new.primitive_poses.push_back(box_pose);
		collision_object_new.operation = collision_object_new.ADD;

		return collision_object_new;
	}();
	
	// Add the collision object to the scene
	// moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	planning_scene_interface.applyCollisionObject(collision_object_new);

  rclcpp::shutdown();
  return 0;
}