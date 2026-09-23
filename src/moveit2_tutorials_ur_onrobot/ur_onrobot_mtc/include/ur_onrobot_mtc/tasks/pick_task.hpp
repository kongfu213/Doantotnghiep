#pragma once

#include <memory>
#include <string>

#include "ur_onrobot_mtc/tasks/task_base.hpp"
#include "ur_onrobot_mtc/tasks/robot_selector.hpp"

namespace ur_onrobot_mtc
{

class PickTask : public TaskBase
{
public:
  PickTask(std::shared_ptr<DualArmInterface> robot,
           std::shared_ptr<RobotSelector> selector);

  bool execute(const std::string& object_name,
               ArmSide requested_arm = ArmSide::AUTO);

private:
  std::shared_ptr<RobotSelector> selector_;
};

}  // namespace ur_onrobot_mtc
