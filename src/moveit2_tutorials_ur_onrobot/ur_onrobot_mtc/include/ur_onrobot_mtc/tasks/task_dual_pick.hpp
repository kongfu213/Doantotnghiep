#ifndef UR_ONROBOT_MTC_TASK_DUAL_PICK_HPP_
#define UR_ONROBOT_MTC_TASK_DUAL_PICK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <geometry_msgs/msg/pose.hpp>
#include <string>

namespace dual_ur_tasks {

class DualPickTask {
public:
  explicit DualPickTask(const rclcpp::Node::SharedPtr& node);

  // Khai báo hàm tạo task gắp cho 1 tay
  moveit::task_constructor::Task createSingleTask(
      const std::string& arm_side,
      const geometry_msgs::msg::Pose& target_pose);

  // Khai báo hàm tạo task gắp cho cả 2 tay
  moveit::task_constructor::Task createDualTask(
      const geometry_msgs::msg::Pose& left_target_pose,
      const geometry_msgs::msg::Pose& right_target_pose);

private:
  rclcpp::Node::SharedPtr node_;
};

} // namespace dual_ur_tasks

#endif // UR_ONROBOT_MTC_TASK_DUAL_PICK_HPP_