#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace ur_onrobot_mtc
{

enum class ArmSide
{
  LEFT,
  RIGHT,
  AUTO,
  NONE
};

struct ArmScore
{
  ArmSide arm = ArmSide::NONE;
  bool ik_ok = false;
  bool collision_free = false;
  bool approach_ok = false;
  bool plan_ok = false;
  bool reachable = false;

  double cartesian_distance = 0.0;
  double joint_cost = 0.0;
  double max_joint_delta = 0.0;
  double total_cost = 0.0;
  double grasp_yaw = 0.0;
};

class DualArmInterface
{
public:
  explicit DualArmInterface(const rclcpp::NodeOptions& options);
  ~DualArmInterface();

  DualArmInterface(const DualArmInterface&) = delete;
  DualArmInterface& operator=(const DualArmInterface&) = delete;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  rclcpp::Node::SharedPtr node() const;

  bool initializeScene();
  bool resetScene();
  void printStatus();

  bool executeEnabled() const;

  ArmScore evaluateArm(const std::string& object_name, ArmSide arm);
  ArmSide selectBestArm(const std::string& object_name);

  bool pick(const std::string& object_name, ArmSide requested_arm = ArmSide::AUTO);
  // yaw_rad = NaN means automatic orientation selection.
  // A finite yaw_rad means the caller explicitly requests that yaw.
  bool place(const std::string& object_name,
             double x, double y, double z,
             double yaw_rad);
  // yaw_rad = NaN means automatic orientation selection.
  // A finite yaw_rad means the caller explicitly requests that yaw.
  bool move(const std::string& object_name,
            double x, double y, double z,
            double yaw_rad);
  bool getObjectPose(const std::string& object_name,
                     double& x, double& y, double& z);
  bool stack(const std::vector<std::string>& objects,
             double center_x, double center_y);
  bool swap(const std::string& object_a, const std::string& object_b);

  static std::string normalizeObjectName(const std::string& name);
  static ArmSide parseArmSide(const std::string& text);
  static const char* armSideName(ArmSide arm);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ur_onrobot_mtc
