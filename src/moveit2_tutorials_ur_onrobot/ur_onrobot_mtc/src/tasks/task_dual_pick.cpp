#include "ur_onrobot_mtc/tasks/task_dual_pick.hpp"
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/container.h> // Bổ sung header container

namespace dual_ur_tasks {
namespace mtc = moveit::task_constructor;

DualPickTask::DualPickTask(const rclcpp::Node::SharedPtr& node) : node_(node) {}

mtc::Task DualPickTask::createSingleTask(const std::string& arm_side, const geometry_msgs::msg::Pose& target_pose) {
  mtc::Task task;
  task.stages()->setName("Single Arm Pick: " + arm_side);
  task.loadRobotModel(node_);

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  task.add(std::make_unique<mtc::stages::CurrentState>("current_state"));

  std::string arm_group = arm_side + "_ur_onrobot_manipulator";
  std::string gripper_group = arm_side + "_ur_onrobot_gripper";

  // Mở kẹp
  auto open_gripper = std::make_unique<mtc::stages::MoveTo>("open_gripper", sampling_planner);
  open_gripper->setGroup(gripper_group);
  open_gripper->setGoal("open");
  task.add(std::move(open_gripper));

  // Di chuyển đến điểm gắp
  auto move_arm = std::make_unique<mtc::stages::MoveTo>("move_arm", sampling_planner);
  move_arm->setGroup(arm_group);
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "world";
  goal.pose = target_pose;
  move_arm->setGoal(goal);
  task.add(std::move(move_arm));

  // Đóng kẹp
  auto close_gripper = std::make_unique<mtc::stages::MoveTo>("close_gripper", sampling_planner);
  close_gripper->setGroup(gripper_group);
  close_gripper->setGoal("closed");
  task.add(std::move(close_gripper));

  return task;
}

mtc::Task DualPickTask::createDualTask(const geometry_msgs::msg::Pose& left_target, const geometry_msgs::msg::Pose& right_target) {
  mtc::Task task;
  task.stages()->setName("Dual Arm Parallel Pick");
  task.loadRobotModel(node_);

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  task.add(std::make_unique<mtc::stages::CurrentState>("current_state"));

  // SỬ DỤNG mtc::Merger ĐỂ GỘP VÀ CHẠY SONG SONG 2 NHÁNH
  auto parallel = std::make_unique<mtc::Merger>("Parallel Execution Zone");

  // === NHÁNH TAY TRÁI ===
  {
    auto left_serial = std::make_unique<mtc::SerialContainer>("Left Pick");
    
    auto open_left = std::make_unique<mtc::stages::MoveTo>("open_left", sampling_planner);
    open_left->setGroup("left_ur_onrobot_gripper");
    open_left->setGoal("open");
    left_serial->insert(std::move(open_left));

    auto move_left = std::make_unique<mtc::stages::MoveTo>("move_left_arm", sampling_planner);
    move_left->setGroup("left_ur_onrobot_manipulator");
    geometry_msgs::msg::PoseStamped left_goal;
    left_goal.header.frame_id = "world";
    left_goal.pose = left_target;
    move_left->setGoal(left_goal);
    left_serial->insert(std::move(move_left));

    auto close_left = std::make_unique<mtc::stages::MoveTo>("close_left", sampling_planner);
    close_left->setGroup("left_ur_onrobot_gripper");
    close_left->setGoal("closed");
    left_serial->insert(std::move(close_left));

    parallel->insert(std::move(left_serial));
  }

  // === NHÁNH TAY PHẢI ===
  {
    auto right_serial = std::make_unique<mtc::SerialContainer>("Right Pick");
    
    auto open_right = std::make_unique<mtc::stages::MoveTo>("open_right", sampling_planner);
    open_right->setGroup("right_ur_onrobot_gripper");
    open_right->setGoal("open");
    right_serial->insert(std::move(open_right));

    auto move_right = std::make_unique<mtc::stages::MoveTo>("move_right_arm", sampling_planner);
    move_right->setGroup("right_ur_onrobot_manipulator");
    geometry_msgs::msg::PoseStamped right_goal;
    right_goal.header.frame_id = "world";
    right_goal.pose = right_target;
    move_right->setGoal(right_goal);
    right_serial->insert(std::move(move_right));

    auto close_right = std::make_unique<mtc::stages::MoveTo>("close_right", sampling_planner);
    close_right->setGroup("right_ur_onrobot_gripper");
    close_right->setGoal("closed");
    right_serial->insert(std::move(close_right));

    parallel->insert(std::move(right_serial));
  }

  task.add(std::move(parallel));
  return task;
}

} // namespace dual_ur_tasks