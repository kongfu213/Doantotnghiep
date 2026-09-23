#include "ur_onrobot_mtc/tasks/place_task.hpp"

namespace ur_onrobot_mtc
{

PlaceTask::PlaceTask(std::shared_ptr<DualArmInterface> robot)
  : TaskBase(std::move(robot))
{
}

bool PlaceTask::execute(const std::string& object_name,
                        double x, double y, double z,
                        double yaw_rad)
{
  return robot_->place(
      DualArmInterface::normalizeObjectName(object_name),
      x, y, z, yaw_rad);
}

}  // namespace ur_onrobot_mtc
