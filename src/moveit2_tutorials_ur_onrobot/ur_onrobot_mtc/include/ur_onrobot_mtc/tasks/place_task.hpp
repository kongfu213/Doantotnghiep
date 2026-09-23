#pragma once

#include <string>

#include "ur_onrobot_mtc/tasks/task_base.hpp"

namespace ur_onrobot_mtc
{

class PlaceTask : public TaskBase
{
public:
  explicit PlaceTask(std::shared_ptr<DualArmInterface> robot);

  bool execute(const std::string& object_name,
               double x, double y, double z,
               double yaw_rad);
};

}  // namespace ur_onrobot_mtc
