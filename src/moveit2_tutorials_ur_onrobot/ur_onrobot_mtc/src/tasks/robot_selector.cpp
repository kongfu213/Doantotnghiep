#include "ur_onrobot_mtc/tasks/robot_selector.hpp"

#include <rclcpp/rclcpp.hpp>
#include <utility>

namespace ur_onrobot_mtc
{

RobotSelector::RobotSelector(
    std::shared_ptr<DualArmInterface> robot)
  : robot_(std::move(robot))
{
}

ArmScore RobotSelector::evaluateArm(
    const std::string& object_name,
    ArmSide arm)
{
  return robot_->evaluateArm(
      DualArmInterface::normalizeObjectName(object_name),
      arm);
}

ArmSide RobotSelector::selectBestArm(
    const std::string& object_name)
{
  const std::string object =
      DualArmInterface::normalizeObjectName(
          object_name);

  const ArmScore left =
      evaluateArm(object, ArmSide::LEFT);

  const ArmScore right =
      evaluateArm(object, ArmSide::RIGHT);

  auto logger =
      robot_->node()->get_logger();

  RCLCPP_INFO(
      logger,
      "SELECTOR object=%s | "
      "LEFT feasible=%s cost=%.3f | "
      "RIGHT feasible=%s cost=%.3f",
      object.c_str(),
      left.reachable ? "YES" : "NO",
      left.total_cost,
      right.reachable ? "YES" : "NO",
      right.total_cost);

  if (!left.reachable &&
      !right.reachable)
  {
    return ArmSide::NONE;
  }

  if (left.reachable &&
      !right.reachable)
  {
    return ArmSide::LEFT;
  }

  if (!left.reachable &&
      right.reachable)
  {
    return ArmSide::RIGHT;
  }

  return left.total_cost <= right.total_cost
      ? ArmSide::LEFT
      : ArmSide::RIGHT;
}

}  // namespace ur_onrobot_mtc