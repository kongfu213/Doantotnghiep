#include "ur_onrobot_mtc/tasks/swap_task.hpp"

namespace ur_onrobot_mtc
{

SwapTask::SwapTask(std::shared_ptr<DualArmInterface> robot)
  : TaskBase(std::move(robot))
{
}

bool SwapTask::execute(const std::string& object_a,
                       const std::string& object_b)
{
  return robot_->swap(
      DualArmInterface::normalizeObjectName(object_a),
      DualArmInterface::normalizeObjectName(object_b));
}

}  // namespace ur_onrobot_mtc
