#pragma once

#include <memory>
#include <string>

#include "ur_onrobot_mtc/tasks/task_base.hpp"

namespace ur_onrobot_mtc
{

class MoveTask : public TaskBase
{
public:
  explicit MoveTask(std::shared_ptr<DualArmInterface> robot);

  // High-level MOVE: pick the object from its current PlanningScene pose,
  // then place it at the requested world-frame target pose.
  // yaw_rad = NaN: choose orientation automatically.
  // finite yaw_rad: use the requested yaw.
  bool execute(const std::string& object_name,
               double x, double y, double z,
               double yaw_rad);
};

}  // namespace ur_onrobot_mtc
