#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "ur_onrobot_mtc/tasks/task_base.hpp"

namespace ur_onrobot_mtc
{

class MissionTask : public TaskBase
{
public:
  using StepExecutor = std::function<bool(const std::string&)>;

  explicit MissionTask(std::shared_ptr<DualArmInterface> robot);

  // Execute a named mission loaded from ROS parameters:
  //   missions.<name>: ["command 1", "command 2", ...]
  // Stops immediately if one step fails.
  bool execute(const std::string& mission_name,
               const StepExecutor& executor);

  // Print mission_names from missions.yaml.
  void printAvailable() const;

private:
  bool loadSteps(const std::string& mission_name,
                 std::vector<std::string>& steps) const;

  std::vector<std::string> availableNames() const;
};

}  // namespace ur_onrobot_mtc
