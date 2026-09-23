#pragma once

#include <memory>
#include <string>

#include "ur_onrobot_mtc/dual_arm_interface.hpp"

namespace ur_onrobot_mtc
{

class RobotSelector
{
public:
  explicit RobotSelector(std::shared_ptr<DualArmInterface> robot);

  ArmScore evaluateArm(
      const std::string& object_name,
      ArmSide arm);

  ArmSide selectBestArm(
      const std::string& object_name);

private:
  std::shared_ptr<DualArmInterface> robot_;
};

}  // namespace ur_onrobot_mtc