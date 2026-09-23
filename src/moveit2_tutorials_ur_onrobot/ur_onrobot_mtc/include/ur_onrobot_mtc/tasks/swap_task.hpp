#pragma once

#include <string>

#include "ur_onrobot_mtc/tasks/task_base.hpp"

namespace ur_onrobot_mtc
{

class SwapTask : public TaskBase
{
public:
  explicit SwapTask(std::shared_ptr<DualArmInterface> robot);

  bool execute(const std::string& object_a,
               const std::string& object_b);
};

}  // namespace ur_onrobot_mtc
