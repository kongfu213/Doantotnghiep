#pragma once

#include <string>
#include <vector>

#include "ur_onrobot_mtc/tasks/task_base.hpp"

namespace ur_onrobot_mtc
{

class StackTask : public TaskBase
{
public:
  explicit StackTask(std::shared_ptr<DualArmInterface> robot);

  bool execute(const std::vector<std::string>& objects,
               double center_x,
               double center_y);
};

}  // namespace ur_onrobot_mtc
