#include <memory>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "your_first_project",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = rclcpp::get_logger("your_first_project");

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

  MoveGroupInterface::Plan plan;
  const bool success = static_cast<bool>(move_group_interface.plan(plan));

  if (success)
  {
    RCLCPP_INFO(logger, "Planning succeeded. Executing trajectory.");
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed.");
  }

  rclcpp::shutdown();
  spinner.join();
  return success ? 0 : 1;
}
