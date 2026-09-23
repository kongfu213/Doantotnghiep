#include "ur_onrobot_mtc/tasks/pick_task.hpp"

#include <rclcpp/rclcpp.hpp>

namespace ur_onrobot_mtc
{

PickTask::PickTask(std::shared_ptr<DualArmInterface> robot,
                   std::shared_ptr<RobotSelector> selector)
  : TaskBase(std::move(robot)),
    selector_(std::move(selector))
{
}

bool PickTask::execute(const std::string& object_name, ArmSide requested_arm)
{
  const std::string object = DualArmInterface::normalizeObjectName(object_name);
  ArmSide selected = requested_arm;

  if (selected == ArmSide::AUTO)
    selected = selector_->selectBestArm(object);

  if (selected != ArmSide::LEFT && selected != ArmSide::RIGHT) {
    RCLCPP_ERROR(
        robot_->node()->get_logger(),
        "PICK %s failed: neither arm has a feasible solution",
        object.c_str());
    return false;
  }

  RCLCPP_INFO(
      robot_->node()->get_logger(),
      "PICK TASK object=%s selected_arm=%s",
      object.c_str(),
      DualArmInterface::armSideName(selected));

  // Passing a fixed arm here prevents the low-level layer from choosing again.
  return robot_->pick(object, selected);
}

}  // namespace ur_onrobot_mtc
