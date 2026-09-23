#include "ur_onrobot_mtc/tasks/move_task.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace ur_onrobot_mtc
{

MoveTask::MoveTask(std::shared_ptr<DualArmInterface> robot)
  : TaskBase(std::move(robot))
{
}

bool MoveTask::execute(const std::string& object_name,
                       double x, double y, double z,
                       double yaw_rad)
{
  const std::string object = DualArmInterface::normalizeObjectName(object_name);

  if (std::isfinite(yaw_rad)) {
    RCLCPP_INFO(
        robot_->node()->get_logger(),
        "MOVE TASK object=%s target=(%.3f, %.3f, %.3f) yaw=%.1f deg",
        object.c_str(), x, y, z, yaw_rad * 180.0 / M_PI);
  } else {
    RCLCPP_INFO(
        robot_->node()->get_logger(),
        "MOVE TASK object=%s target=(%.3f, %.3f, %.3f) yaw=AUTO",
        object.c_str(), x, y, z);
  }

  // Keep MOVE logic in DualArmInterface so the task layer stays thin.
  // The interface is responsible for selecting an arm, picking from the
  // current PlanningScene pose, and placing at the dynamic target pose.
  return robot_->move(object, x, y, z, yaw_rad);
}

}  // namespace ur_onrobot_mtc
