#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char *argv[]) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "trajectory_planning",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("start trajectory_planning_node");

  // We spin up a SingleThreadedExecutor for the current state monitor to get information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "aubo_arm");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closure for updating the text in rviz
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };

  // 在 RViz 上显示中间点
  auto const draw_intermediate_point =
      [&moveit_visual_tools](const geometry_msgs::msg::Pose &pose) {
        moveit_visual_tools.publishSphere(pose, rviz_visual_tools::RED,
                                          rviz_visual_tools::XXLARGE);
      };
  // 逆运动学
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 0.707;
    msg.orientation.x = 0.707;
    msg.position.x = 0.3;
    msg.position.y = -0.3;
    msg.position.z = 0.1;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // 正运动学
  // std::vector<double> joint_values = {1.0, 1.0, 1.0,
  //                                     1.0, 1.0, 1.0}; // 6个关节的角度(弧度)
  // move_group_interface.setJointValueTarget(joint_values);

  // 创建碰撞体
  auto const collision_object = [frame_id =
                                     move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.05;
    primitive.dimensions[primitive.BOX_Y] = 0.3;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 0.92;
    box_pose.orientation.z = -0.38;
    box_pose.position.x = 0.2;
    box_pose.position.y = -0.2;
    box_pose.position.z = 0.5;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  draw_intermediate_point(target_pose);
  // Create a plan to that target pose
  prompt("Press 'next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    moveit_visual_tools.trigger();
    prompt("Press 'next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);

    // 获取末端执行器的当前位姿
    auto current_pose = move_group_interface.getCurrentPose().pose;
    RCLCPP_INFO(
        logger,
        "Current Pose: x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f",
        current_pose.position.x, current_pose.position.y,
        current_pose.position.z, 0.370585, 0.776431, 0.112297, 0.567834);

    // 获取各关节的当前角度
    auto joint_values = move_group_interface.getCurrentJointValues();
    RCLCPP_INFO(logger, "Current Joint Values:");
    RCLCPP_INFO(logger, "shoulder_joint: %f", joint_values[0]);
    RCLCPP_INFO(logger, "upperArm_joint: %f", joint_values[1]);
    RCLCPP_INFO(logger, "foreArm_joint: %f", joint_values[2]);
    RCLCPP_INFO(logger, "wrist1_joint: %f", joint_values[3]);
    RCLCPP_INFO(logger, "wrist2_joint: %f", joint_values[4]);
    RCLCPP_INFO(logger, "wrist3_joint: %f", joint_values[5]);

  } else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
