#pragma once

#include "ur_onrobot_mtc/tasks/task_base.hpp"
#include <memory>

namespace ur_onrobot_mtc
{
class SolderTask : public TaskBase
{
public:
  explicit SolderTask(std::shared_ptr<DualArmInterface> robot);
  ~SolderTask();
  // 0 = all three components; 1..3 = one matching component/slot.
  bool execute(int component = 0);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace ur_onrobot_mtc
