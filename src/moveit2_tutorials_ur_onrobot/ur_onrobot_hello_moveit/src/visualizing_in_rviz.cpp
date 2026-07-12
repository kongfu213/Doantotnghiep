#include <memory>
#include <thread>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "visualizing_in_rviz",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = rclcpp::get_logger("visualizing_in_rviz");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_onrobot_manipulator");
  move_group_interface.setPlanningTime(5.0);
  move_group_interface.setNumPlanningAttempts(10);

  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.707;
    msg.orientation.y = 0.707;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    msg.position.x = 0.1;
    msg.position.y = 0.3;
    msg.position.z = 0.2;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  moveit_visual_tools::MoveItVisualTools moveit_visual_tools(
      node,
      "base_link",
      rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel());
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  const auto draw_title = [&moveit_visual_tools](const auto& text) {
    auto text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.6;
    moveit_visual_tools.publishText(
        text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };

  const auto prompt = [&moveit_visual_tools](const auto& text) {
    moveit_visual_tools.prompt(text);
  };

  const auto draw_trajectory = [&moveit_visual_tools,
                                joint_model_group = move_group_interface.getRobotModel()
                                                        ->getJointModelGroup("ur_onrobot_manipulator")](
                                   const auto& trajectory) {
    moveit_visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
  };

  prompt("Press Next in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();

  MoveGroupInterface::Plan plan;
  const bool success = static_cast<bool>(move_group_interface.plan(plan));

  if (success)
  {
    draw_trajectory(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press Next in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  }
  else
  {
    draw_title("Planning failed");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed.");
  }

  rclcpp::shutdown();
  spinner.join();
  return success ? 0 : 1;
}
