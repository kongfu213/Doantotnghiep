#include "ur_onrobot_mtc/tasks/stack_task.hpp"

namespace ur_onrobot_mtc
{

StackTask::StackTask(std::shared_ptr<DualArmInterface> robot)
  : TaskBase(std::move(robot))
{
}

bool StackTask::execute(const std::vector<std::string>& objects,
                        double center_x,
                        double center_y)
{
  std::vector<std::string> normalized;
  normalized.reserve(objects.size());

  for (const auto& object : objects)
    normalized.push_back(DualArmInterface::normalizeObjectName(object));

  // The proven STACK4 placement/contact logic remains in DualArmInterface.
  // This task module owns the high-level "stack" command boundary.
  return robot_->stack(normalized, center_x, center_y);
}

}  // namespace ur_onrobot_mtc
