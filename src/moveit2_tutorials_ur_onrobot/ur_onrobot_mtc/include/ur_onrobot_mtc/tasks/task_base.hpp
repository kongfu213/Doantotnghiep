#pragma once

#include <memory>
#include <utility>

#include "ur_onrobot_mtc/dual_arm_interface.hpp"

namespace ur_onrobot_mtc
{

class TaskBase
{
public:
  explicit TaskBase(std::shared_ptr<DualArmInterface> robot)
    : robot_(std::move(robot))
  {
  }

  virtual ~TaskBase() = default;

protected:
  std::shared_ptr<DualArmInterface> robot_;
};

}  // namespace ur_onrobot_mtc
