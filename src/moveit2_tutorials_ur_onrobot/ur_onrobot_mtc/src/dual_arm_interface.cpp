#include "ur_onrobot_mtc/dual_arm_interface.hpp"
#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <limits>
#include <algorithm>
#include <sstream>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <array>
#include <functional>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Geometry>

namespace ur_onrobot_mtc
{

namespace mtc = moveit::task_constructor;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("dual_arm_mtc_node");

struct ArmConfig
{
  std::string arm_group;
  std::string hand_group;
  std::string hand_frame;
  std::string object_name;
};

struct Candidate
{
  moveit::core::RobotState pregrasp;
  moveit::core::RobotState grasp;
  double angle = 0.0;
  double rank_score = std::numeric_limits<double>::infinity();

  explicit Candidate(const moveit::core::RobotModelConstPtr& model)
    : pregrasp(model), grasp(model) {}
};

struct DualTaskTargets
{
  std::map<std::string, double> pregrasp;
  std::map<std::string, double> grasp;
  std::map<std::string, double> lift;
  std::map<std::string, double> preplace;
  std::map<std::string, double> place;
  std::map<std::string, double> home;
  double left_angle = 0.0;
  double right_angle = 0.0;
  double score = 0.0;
};



struct HeldObject
{
  bool valid = false;
  std::string object_name;
  ArmSide arm = ArmSide::NONE;
  Eigen::Isometry3d hand_to_object = Eigen::Isometry3d::Identity();
};

struct ArmEvaluation
{
  ArmSide arm = ArmSide::NONE;
  bool ik_ok = false;
  bool collision_free = false;
  bool approach_ok = false;
  bool plan_ok = false;
  bool reachable = false;
  double cartesian_distance = std::numeric_limits<double>::infinity();
  double joint_cost = std::numeric_limits<double>::infinity();
  double max_joint_delta = std::numeric_limits<double>::infinity();
  double total_cost = std::numeric_limits<double>::infinity();
  double grasp_yaw = 0.0;
  moveit::core::RobotState pregrasp;
  moveit::core::RobotState grasp;
  moveit::core::RobotState lift;

  explicit ArmEvaluation(const moveit::core::RobotModelConstPtr& model)
    : pregrasp(model), grasp(model), lift(model) {}
};

static Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose& p)
{
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = q.normalized().toRotationMatrix();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  return t;
}

static Eigen::Isometry3d graspFrameTransform()
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
  tf.linear() = q.matrix();
  tf.translation().z() = 0.015;
  return tf;
}


static Eigen::Isometry3d topDownHandPose(
    double object_x,
    double object_y,
    double object_z,
    double grasp_yaw,
    double object_yaw = 0.0)
{
  // TCP local +Z points DOWN. Therefore translating along local -Z
  // moves the hand straight UP for PREGRASP.
  constexpr double TCP_Z_OFFSET = 0.015;

  Eigen::Isometry3d hand = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(object_yaw + grasp_yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

  hand.linear() = q.normalized().toRotationMatrix();
  hand.translation() =
      Eigen::Vector3d(object_x, object_y, object_z + TCP_Z_OFFSET);
  return hand;
}

static Eigen::Isometry3d topDownHandPose(
    const Eigen::Isometry3d& world_object,
    double grasp_yaw)
{
  return topDownHandPose(
      world_object.translation().x(),
      world_object.translation().y(),
      world_object.translation().z(),
      grasp_yaw,
      0.0);
}


static Eigen::Isometry3d leftStackHandPose(
    const Eigen::Isometry3d& world_object,
    double grasp_yaw)
{
  constexpr double TCP_Z_OFFSET = 0.015;
  constexpr double LEFT_TILT_X = 10.0 * M_PI / 180.0;

  Eigen::Isometry3d hand = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(grasp_yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(LEFT_TILT_X, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

  hand.linear() = q.normalized().toRotationMatrix();
  hand.translation() =
      world_object.translation() + Eigen::Vector3d(0.0, 0.0, TCP_Z_OFFSET);
  return hand;
}


static Eigen::Isometry3d rightHighStackHandPose(
    const Eigen::Isometry3d& world_object,
    double grasp_yaw)
{
  constexpr double TCP_Z_OFFSET = 0.015;
  constexpr double RIGHT_TILT_X = -10.0 * M_PI / 180.0;

  Eigen::Isometry3d hand = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(grasp_yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(RIGHT_TILT_X, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

  hand.linear() = q.normalized().toRotationMatrix();
  hand.translation() =
      world_object.translation() + Eigen::Vector3d(0.0, 0.0, TCP_Z_OFFSET);
  return hand;
}

static bool solveIKCollisionFree(
    const planning_scene::PlanningScenePtr& scene,
    const ArmConfig& cfg,
    const Eigen::Isometry3d& target,
    const moveit::core::RobotState& seed,
    moveit::core::RobotState& solution,
    int attempts = 12)
{
  const auto* jmg = scene->getRobotModel()->getJointModelGroup(cfg.arm_group);
  if (!jmg)
    return false;

  for (int i = 0; i < attempts; ++i) {
    moveit::core::RobotState trial(seed);

    if (i > 0) {
      trial.setToRandomPositions(jmg);
      trial.update();
    }

    if (!trial.setFromIK(jmg, target, cfg.hand_frame, 0.15))
      continue;

    trial.update();

    if (!trial.satisfiesBounds(jmg))
      continue;

    if (scene->isStateColliding(trial, cfg.arm_group, false))
      continue;

    solution = trial;
    return true;
  }

  return false;
}

static bool validateApproach(
    const planning_scene::PlanningScenePtr& scene,
    const ArmConfig& cfg,
    const Eigen::Isometry3d& world_hand_grasp,
    const moveit::core::RobotState& pregrasp_state)
{
  const auto* jmg = scene->getRobotModel()->getJointModelGroup(cfg.arm_group);
  if (!jmg)
    return false;

  constexpr double APPROACH_DISTANCE = 0.05;
  constexpr int STEPS = 10;

  moveit::core::RobotState state(pregrasp_state);

  for (int i = 1; i <= STEPS; ++i) {
    const double d = APPROACH_DISTANCE * static_cast<double>(i) / static_cast<double>(STEPS);
    Eigen::Isometry3d pose =
        world_hand_grasp * Eigen::Translation3d(0.0, 0.0, d - APPROACH_DISTANCE);

    if (!state.setFromIK(jmg, pose, cfg.hand_frame, 0.10))
      return false;

    state.update();

    if (scene->isStateColliding(state, cfg.arm_group, false))
      return false;
  }

  return true;
}

static std::vector<Candidate> findCandidates(
    const planning_scene::PlanningScenePtr& scene,
    const ArmConfig& cfg,
    const Eigen::Isometry3d& world_object,
    const moveit::core::RobotState& seed)
{
  std::vector<Candidate> out;
  const Eigen::Isometry3d hand_to_grasp = graspFrameTransform();

  constexpr double APPROACH_DISTANCE = 0.05;

  // Quet day du 0..350 deg, khong uu tien cung 260 deg nua.
  for (int idx = 0; idx < 36; ++idx) {
    const double angle = idx * M_PI / 18.0;

    Eigen::Isometry3d object_grasp =
        Eigen::Isometry3d(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));

    const Eigen::Isometry3d world_grasp = world_object * object_grasp;
    const Eigen::Isometry3d world_hand_grasp = world_grasp * hand_to_grasp.inverse();
    const Eigen::Isometry3d world_hand_pregrasp =
        world_hand_grasp * Eigen::Translation3d(0.0, 0.0, -APPROACH_DISTANCE);

    moveit::core::RobotState grasp(scene->getRobotModel());
    if (!solveIKCollisionFree(scene, cfg, world_hand_grasp, seed, grasp))
      continue;

    moveit::core::RobotState pregrasp(scene->getRobotModel());
    if (!solveIKCollisionFree(scene, cfg, world_hand_pregrasp, grasp, pregrasp))
      continue;

    if (!validateApproach(scene, cfg, world_hand_grasp, pregrasp))
      continue;

    Candidate c(scene->getRobotModel());
    c.pregrasp = pregrasp;
    c.grasp = grasp;
    c.angle = angle;
    out.push_back(c);
  }

  return out;
}

static double nearestEquivalentAngle(
    double target,
    double current,
    const moveit::core::VariableBounds& bounds)
{
  double best = target;
  double best_delta = std::abs(target - current);

  for (int k = -2; k <= 2; ++k) {
    const double candidate = target + static_cast<double>(k) * 2.0 * M_PI;

    if (bounds.position_bounded_ &&
        (candidate < bounds.min_position_ || candidate > bounds.max_position_))
      continue;

    const double delta = std::abs(candidate - current);

    if (delta < best_delta) {
      best = candidate;
      best_delta = delta;
    }
  }

  return best;
}

static void normalizeGroupNearState(
    const moveit::core::RobotModelConstPtr& robot_model,
    const moveit::core::JointModelGroup* jmg,
    const moveit::core::RobotState& reference,
    std::vector<double>& q)
{
  const auto& names = jmg->getVariableNames();
  std::vector<double> ref_q;
  reference.copyJointGroupPositions(jmg, ref_q);

  for (std::size_t i = 0; i < q.size() && i < names.size(); ++i) {
    const auto& bounds = robot_model->getVariableBounds(names[i]);
    q[i] = nearestEquivalentAngle(q[i], ref_q[i], bounds);
  }
}

static double jointTravelScore(
    const std::vector<double>& left_q,
    const std::vector<double>& right_q,
    const std::vector<double>& left_current,
    const std::vector<double>& right_current,
    double& max_delta)
{
  double sum = 0.0;
  max_delta = 0.0;

  for (std::size_t i = 0; i < left_q.size(); ++i) {
    const double d = std::abs(left_q[i] - left_current[i]);
    sum += d;
    max_delta = std::max(max_delta, d);
  }

  for (std::size_t i = 0; i < right_q.size(); ++i) {
    const double d = std::abs(right_q[i] - right_current[i]);
    sum += d;
    max_delta = std::max(max_delta, d);
  }

  return sum + 2.0 * max_delta;
}

static bool validateDualInterpolation(
    const planning_scene::PlanningScenePtr& scene,
    const moveit::core::RobotState& start,
    const moveit::core::RobotState& goal,
    const moveit::core::JointModelGroup* dual_jmg,
    int steps = 30)
{
  std::vector<double> q0;
  std::vector<double> q1;
  start.copyJointGroupPositions(dual_jmg, q0);
  goal.copyJointGroupPositions(dual_jmg, q1);

  for (int i = 0; i <= steps; ++i) {
    const double u = static_cast<double>(i) / static_cast<double>(steps);
    std::vector<double> q(q0.size());

    for (std::size_t j = 0; j < q.size(); ++j)
      q[j] = q0[j] + u * (q1[j] - q0[j]);

    moveit::core::RobotState state(start);
    state.setJointGroupPositions(dual_jmg, q);
    state.update();

    if (!state.satisfiesBounds(dual_jmg))
      return false;

    if (scene->isStateColliding(state, "", false))
      return false;
  }

  return true;
}

static std::map<std::string, double> groupGoal(
    const moveit::core::RobotState& state,
    const moveit::core::JointModelGroup* jmg)
{
  std::vector<double> q;
  state.copyJointGroupPositions(jmg, q);

  std::map<std::string, double> goal;
  const auto& names = jmg->getVariableNames();

  for (std::size_t i = 0; i < names.size(); ++i)
    goal[names[i]] = q[i];

  return goal;
}

class DualArmInterface::Impl
{
public:
  explicit Impl(const rclcpp::NodeOptions& options)
  {
    node_ = std::make_shared<rclcpp::Node>("dual_arm_mtc_node", options);

    if (!node_->has_parameter("execute"))
      node_->declare_parameter<bool>("execute", false);

    if (!node_->has_parameter("left_place_x"))
      node_->declare_parameter<double>("left_place_x", -0.25);
    if (!node_->has_parameter("left_place_y"))
      node_->declare_parameter<double>("left_place_y", 0.10);
    if (!node_->has_parameter("left_place_z"))
      node_->declare_parameter<double>("left_place_z", 0.052);

    if (!node_->has_parameter("right_place_x"))
      node_->declare_parameter<double>("right_place_x", -0.25);
    if (!node_->has_parameter("right_place_y"))
      node_->declare_parameter<double>("right_place_y", -0.10);
    if (!node_->has_parameter("right_place_z"))
      node_->declare_parameter<double>("right_place_z", 0.052);

    if (!node_->has_parameter("left_place_roll_deg"))
      node_->declare_parameter<double>("left_place_roll_deg", 0.0);
    if (!node_->has_parameter("left_place_pitch_deg"))
      node_->declare_parameter<double>("left_place_pitch_deg", 0.0);
    if (!node_->has_parameter("left_place_yaw_deg"))
      node_->declare_parameter<double>("left_place_yaw_deg", 0.0);

    if (!node_->has_parameter("right_place_roll_deg"))
      node_->declare_parameter<double>("right_place_roll_deg", 0.0);
    if (!node_->has_parameter("right_place_pitch_deg"))
      node_->declare_parameter<double>("right_place_pitch_deg", 0.0);
    if (!node_->has_parameter("right_place_yaw_deg"))
      node_->declare_parameter<double>("right_place_yaw_deg", 0.0);

    if (!node_->has_parameter("task_mode"))
      node_->declare_parameter<std::string>("task_mode", "pick_place");
    if (!node_->has_parameter("demo_parallel_dz"))
      node_->declare_parameter<double>("demo_parallel_dz", 0.05);
    if (!node_->has_parameter("demo_single_dx"))
      node_->declare_parameter<double>("demo_single_dx", 0.05);
    if (!node_->has_parameter("demo_joint_delta"))
      node_->declare_parameter<double>("demo_joint_delta", 0.08);

    if (!node_->has_parameter("coord_center_shift"))
      node_->declare_parameter<double>("coord_center_shift", 0.05);
    if (!node_->has_parameter("coord_both_dz"))
      node_->declare_parameter<double>("coord_both_dz", 0.03);
    if (!node_->has_parameter("coord_rotate_deg"))
      node_->declare_parameter<double>("coord_rotate_deg", 20.0);

    if (!node_->has_parameter("stack_center_x"))
      node_->declare_parameter<double>("stack_center_x", 0.0);
    if (!node_->has_parameter("stack_center_y"))
      node_->declare_parameter<double>("stack_center_y", 0.0);
    if (!node_->has_parameter("stack_object_size"))
      node_->declare_parameter<double>("stack_object_size", 0.04);
    // STACK4 uses one explicit table definition instead of trusting a possibly
    // stale/overwritten "table" already present in PlanningScene.
    if (!node_->has_parameter("stack_table_top"))
      node_->declare_parameter<double>("stack_table_top", -0.003);
    if (!node_->has_parameter("stack_table_thickness"))
      node_->declare_parameter<double>("stack_table_thickness", 0.10);
    if (!node_->has_parameter("stack_gap"))
      node_->declare_parameter<double>("stack_gap", 0.003);
    if (!node_->has_parameter("stack_pre_dz"))
      node_->declare_parameter<double>("stack_pre_dz", 0.08);
    if (!node_->has_parameter("stack_transit_height"))
      node_->declare_parameter<double>("stack_transit_height", 0.22);
    if (!node_->has_parameter("stack_entry_offset"))
      node_->declare_parameter<double>("stack_entry_offset", 0.10);
    if (!node_->has_parameter("stack_a_x"))
      node_->declare_parameter<double>("stack_a_x", 0.30);
    if (!node_->has_parameter("stack_a_y"))
      node_->declare_parameter<double>("stack_a_y", 0.25);
    if (!node_->has_parameter("stack_b_x"))
      node_->declare_parameter<double>("stack_b_x", -0.25);
    if (!node_->has_parameter("stack_b_y"))
      node_->declare_parameter<double>("stack_b_y", 0.25);
    if (!node_->has_parameter("stack_c_x"))
      node_->declare_parameter<double>("stack_c_x", 0.30);
    if (!node_->has_parameter("stack_c_y"))
      node_->declare_parameter<double>("stack_c_y", -0.25);
    if (!node_->has_parameter("stack_d_x"))
      node_->declare_parameter<double>("stack_d_x", -0.25);
    if (!node_->has_parameter("stack_d_y"))
      node_->declare_parameter<double>("stack_d_y", -0.25);

    // Command-server parameters. Commands come from robot_cli on /robot_command.
    if (!node_->has_parameter("command_initialize_stack_scene"))
      node_->declare_parameter<bool>("command_initialize_stack_scene", true);
    if (!node_->has_parameter("swap_buffer_x"))
      node_->declare_parameter<double>("swap_buffer_x", 0.0);
    if (!node_->has_parameter("swap_buffer_y"))
      node_->declare_parameter<double>("swap_buffer_y", 0.36);

  }

  ~Impl() = default;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
  {
    return node_->get_node_base_interface();
  }

  bool isCommandServerMode() const
  {
    return node_->get_parameter("task_mode").as_string() == "command_server";
  }

  void run()
  {
    const std::string task_mode = node_->get_parameter("task_mode").as_string();

    if (task_mode == "command_server") {
      try {
        if (node_->get_parameter("command_initialize_stack_scene").as_bool()) {
          stack_column_yaw_ = std::numeric_limits<double>::quiet_NaN();
          stack_table_top_ = setupStack4Scene();
        } else {
          stack_table_top_ = node_->get_parameter("stack_table_top").as_double();
        }
      }
      catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "COMMAND SERVER scene setup failed: %s", e.what());
        rclcpp::shutdown();
        return;
      }

      command_server_ready_.store(true);
      command_cv_.notify_all();
      printCommandHelp();
      RCLCPP_INFO(LOGGER, "COMMAND SERVER READY - waiting for terminal commands");
      return;
    }

    if (task_mode == "primitive_demo") {
      runPrimitiveDemo();
      return;
    }

    if (task_mode == "pose_demo") {
      runPoseDemo();
      return;
    }

    if (task_mode == "dual_pick_demo") {
      runDualPickDemo();
      return;
    }

    if (task_mode == "coordination_demo") {
      runCoordinationDemo();
      return;
    }

    if (task_mode == "stack4_demo") {
      runStack4Demo();
      return;
    }

    if (task_mode != "pick_place") {
      RCLCPP_ERROR(
          LOGGER,
          "task_mode='%s' khong hop le. Dung command_server, pick_place, primitive_demo, pose_demo, dual_pick_demo, coordination_demo hoac stack4_demo.",
          task_mode.c_str());
      return;
    }

    try {
      task_ = createTask();
      task_.init();
    }
    catch (const mtc::InitStageException& e) {
      RCLCPP_ERROR_STREAM(LOGGER, "MTC init failed:\n" << e);
      return;
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(LOGGER, "Create task failed: %s", e.what());
      return;
    }

    if (!task_.plan(5)) {
      RCLCPP_ERROR(LOGGER, "DUAL ARM MTC planning failed");
      return;
    }

    RCLCPP_INFO(LOGGER, "==========================================");
    RCLCPP_INFO(LOGGER, " DUAL ARM MTC FULL PICK+PLACE PLAN SUCCESS");
    RCLCPP_INFO(LOGGER, " PREGRASP -> GRASP -> CLOSE -> ATTACH -> LIFT -> PLACE -> OPEN -> DETACH -> HOME");
    RCLCPP_INFO(LOGGER, "==========================================");

    task_.introspection().publishSolution(*task_.solutions().front());

    const bool execute = node_->get_parameter("execute").as_bool();

    if (!execute) {
      RCLCPP_INFO(LOGGER, "Chi PLAN. Chay execute:=true de chay FULL PICK + PLACE.");
      return;
    }

    const auto result = task_.execute(*task_.solutions().front());

    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "DUAL ARM MTC FULL PICK+PLACE execution failed");
      return;
    }

    RCLCPP_INFO(LOGGER, "DUAL ARM MTC FULL PICK+PLACE EXECUTION SUCCESS");
  }

  bool executeDualGrippers(double width, const std::string& phase_name)
  {
    moveit::planning_interface::MoveGroupInterface grippers(node_, "dual_grippers");
    grippers.setStartStateToCurrentState();
    grippers.setMaxVelocityScalingFactor(0.35);
    grippers.setMaxAccelerationScalingFactor(0.30);

    if (!grippers.setJointValueTarget(std::map<std::string, double>{
            {"left_finger_width", width},
            {"right_finger_width", width}})) {
      RCLCPP_ERROR(LOGGER, "[%s] set gripper target failed", phase_name.c_str());
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (grippers.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "[%s] gripper PLAN FAILED", phase_name.c_str());
      return false;
    }

    if (!node_->get_parameter("execute").as_bool()) {
      RCLCPP_INFO(LOGGER, "[%s] gripper plan-only", phase_name.c_str());
      return true;
    }

    if (grippers.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "[%s] gripper EXECUTION FAILED", phase_name.c_str());
      return false;
    }

    RCLCPP_INFO(LOGGER, "[%s] gripper EXECUTION SUCCESS", phase_name.c_str());
    return true;
  }

  void runDualPickDemo()
  {
    moveit::planning_interface::MoveGroupInterface dual_group(node_, "dual_arms");
    const auto model = dual_group.getRobotModel();
    const auto* left_jmg = model->getJointModelGroup("left_ur_onrobot_manipulator");
    const auto* right_jmg = model->getJointModelGroup("right_ur_onrobot_manipulator");
    const auto* dual_jmg = model->getJointModelGroup("dual_arms");

    if (!left_jmg || !right_jmg || !dual_jmg) {
      RCLCPP_ERROR(LOGGER, "DIRECT PICK: thieu JointModelGroup");
      return;
    }

    std::vector<std::string> left_hand_links;
    std::vector<std::string> right_hand_links;
    DualTaskTargets targets;

    try {
      RCLCPP_INFO(LOGGER, "DIRECT PICK: tim target PREGRASP/GRASP/LIFT...");
      targets = preparePickTargetsFast(model, left_hand_links, right_hand_links);
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(LOGGER, "DIRECT PICK target search failed: %s", e.what());
      return;
    }

    auto home = dual_group.getCurrentState(5.0);
    if (!home) {
      RCLCPP_ERROR(LOGGER, "DIRECT PICK: khong doc duoc current state");
      return;
    }

    auto applyGoal = [](moveit::core::RobotState& state,
                        const std::map<std::string, double>& goal) {
      for (const auto& kv : goal)
        state.setVariablePosition(kv.first, kv.second);
      state.update();
    };

    moveit::core::RobotState pre(*home);
    moveit::core::RobotState grasp(*home);
    moveit::core::RobotState lift(*home);
    applyGoal(pre, targets.pregrasp);
    applyGoal(grasp, targets.grasp);
    applyGoal(lift, targets.lift);

    // Use the same contact-near state that visually worked in pose_demo.
    // 55% is close enough for the grippers to reach the object without relying
    // on the old MTC arm execution path.
    constexpr double CONTACT_FRACTION = 0.55;
    std::vector<double> left_pre_q, right_pre_q, left_grasp_q, right_grasp_q;
    pre.copyJointGroupPositions(left_jmg, left_pre_q);
    pre.copyJointGroupPositions(right_jmg, right_pre_q);
    grasp.copyJointGroupPositions(left_jmg, left_grasp_q);
    grasp.copyJointGroupPositions(right_jmg, right_grasp_q);

    std::vector<double> left_contact_q(left_pre_q.size());
    std::vector<double> right_contact_q(right_pre_q.size());
    for (std::size_t i = 0; i < left_pre_q.size(); ++i)
      left_contact_q[i] = left_pre_q[i] + CONTACT_FRACTION * (left_grasp_q[i] - left_pre_q[i]);
    for (std::size_t i = 0; i < right_pre_q.size(); ++i)
      right_contact_q[i] = right_pre_q[i] + CONTACT_FRACTION * (right_grasp_q[i] - right_pre_q[i]);

    moveit::core::RobotState contact(pre);
    contact.setJointGroupPositions(left_jmg, left_contact_q);
    contact.setJointGroupPositions(right_jmg, right_contact_q);
    contact.update();

    RCLCPP_INFO(LOGGER, "==========================================");
    RCLCPP_INFO(LOGGER, " DIRECT DUAL PICK START");
    RCLCPP_INFO(LOGGER, " P1 OPEN BOTH GRIPPERS");
    RCLCPP_INFO(LOGGER, " P2 BOTH -> PREGRASP (12DOF)");
    RCLCPP_INFO(LOGGER, " P3 BOTH -> CONTACT (12DOF)");
    RCLCPP_INFO(LOGGER, " P4 CLOSE BOTH + ATTACH 2 OBJECTS");
    RCLCPP_INFO(LOGGER, " P5 BOTH LIFT (12DOF)");
    RCLCPP_INFO(LOGGER, " P6 PUT BACK + OPEN + DETACH");
    RCLCPP_INFO(LOGGER, " P7 BOTH -> HOME");
    RCLCPP_INFO(LOGGER, "==========================================");

    if (!executeDualGrippers(0.100, "P1 OPEN BOTH"))
      return;

    if (!planExecuteDualState(dual_group, *home, pre, HoldMode::NONE,
                              "P2 PARALLEL PREGRASP"))
      return;

    if (!node_->get_parameter("execute").as_bool()) {
      RCLCPP_WARN(LOGGER, "execute:=false, stop after planning P2");
      return;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(700));

    auto s2 = dual_group.getCurrentState(5.0);
    if (!s2)
      return;
    if (!planExecuteDualState(dual_group, *s2, contact, HoldMode::NONE,
                              "P3 PARALLEL CONTACT"))
      return;

    rclcpp::sleep_for(std::chrono::milliseconds(700));

    if (!executeDualGrippers(0.0, "P4 CLOSE BOTH"))
      return;

    // Attach through MoveGroupInterface so the objects follow the grippers in
    // the real PlanningScene during the lift.
    const bool left_attached =
        dual_group.attachObject("left_object", "left_gripper_tcp", left_hand_links);
    const bool right_attached =
        dual_group.attachObject("right_object", "right_gripper_tcp", right_hand_links);

    if (!left_attached || !right_attached) {
      RCLCPP_ERROR(LOGGER,
                   "P4 ATTACH FAILED: left=%s right=%s",
                   left_attached ? "OK" : "FAIL",
                   right_attached ? "OK" : "FAIL");
      return;
    }

    RCLCPP_INFO(LOGGER, "P4 ATTACH SUCCESS: both objects attached");
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    auto s4 = dual_group.getCurrentState(5.0);
    if (!s4)
      return;
    if (!planExecuteDualState(dual_group, *s4, lift, HoldMode::NONE,
                              "P5 PARALLEL LIFT"))
      return;

    RCLCPP_INFO(LOGGER, "P5 HOLD LIFT 2 seconds for visual check");
    rclcpp::sleep_for(std::chrono::seconds(2));

    auto s5 = dual_group.getCurrentState(5.0);
    if (!s5)
      return;
    if (!planExecuteDualState(dual_group, *s5, contact, HoldMode::NONE,
                              "P6 PARALLEL PUT BACK"))
      return;

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    if (!executeDualGrippers(0.100, "P6 OPEN BOTH"))
      return;

    const bool left_detached = dual_group.detachObject("left_object");
    const bool right_detached = dual_group.detachObject("right_object");
    RCLCPP_INFO(LOGGER,
                "P6 DETACH: left=%s right=%s",
                left_detached ? "OK" : "FAIL",
                right_detached ? "OK" : "FAIL");

    auto s6 = dual_group.getCurrentState(5.0);
    if (!s6)
      return;
    if (!planExecuteDualState(dual_group, *s6, pre, HoldMode::NONE,
                              "P6 PARALLEL RETREAT"))
      return;

    auto s7 = dual_group.getCurrentState(5.0);
    if (!s7)
      return;
    if (!planExecuteDualState(dual_group, *s7, *home, HoldMode::NONE,
                              "P7 PARALLEL HOME"))
      return;

    RCLCPP_INFO(LOGGER, "==========================================");
    RCLCPP_INFO(LOGGER, " DIRECT DUAL PICK SUCCESS");
    RCLCPP_INFO(LOGGER, " 2 ROBOT CLOSE + ATTACH + LIFT SONG SONG PASS");
    RCLCPP_INFO(LOGGER, "==========================================");
  }

  bool coordinatedJointNudge(
      moveit::planning_interface::MoveGroupInterface& dual_group,
      double requested_delta,
      const std::string& phase_name)
  {
    const auto model = dual_group.getRobotModel();
    const auto* left_jmg =
        model->getJointModelGroup("left_ur_onrobot_manipulator");
    const auto* right_jmg =
        model->getJointModelGroup("right_ur_onrobot_manipulator");
    const auto* dual_jmg =
        model->getJointModelGroup("dual_arms");

    if (!left_jmg || !right_jmg || !dual_jmg)
      return false;

    auto current = dual_group.getCurrentState(5.0);
    if (!current)
      return false;

    std::vector<double> left_q;
    std::vector<double> right_q;
    current->copyJointGroupPositions(left_jmg, left_q);
    current->copyJointGroupPositions(right_jmg, right_q);

    const std::vector<double> scales = {1.0, 0.5, 0.25};
    const std::vector<int> preferred_joints = {1, 2, 0, 3, 4, 5};

    for (double scale : scales) {
      const double d = requested_delta * scale;

      for (int li : preferred_joints) {
        for (int ri : preferred_joints) {
          for (int sign_l : {1, -1}) {
            for (int sign_r : {1, -1}) {
              std::vector<double> lq = left_q;
              std::vector<double> rq = right_q;

              lq[static_cast<std::size_t>(li)] += sign_l * d;
              rq[static_cast<std::size_t>(ri)] += sign_r * d;

              moveit::core::RobotState target(*current);
              target.setJointGroupPositions(left_jmg, lq);
              target.setJointGroupPositions(right_jmg, rq);
              target.update();

              if (!target.satisfiesBounds(dual_jmg))
                continue;

              std::vector<double> dual_target;
              target.copyJointGroupPositions(dual_jmg, dual_target);

              dual_group.setPlannerId("RRTConnectkConfigDefault");
              dual_group.setPlanningTime(4.0);
              dual_group.setNumPlanningAttempts(2);
              dual_group.setMaxVelocityScalingFactor(0.25);
              dual_group.setMaxAccelerationScalingFactor(0.20);
              dual_group.setStartState(*current);
              dual_group.clearPathConstraints();
              dual_group.setJointValueTarget(dual_target);

              moveit::planning_interface::MoveGroupInterface::Plan plan;
              const auto result = dual_group.plan(plan);

              if (result != moveit::core::MoveItErrorCode::SUCCESS)
                continue;

              RCLCPP_INFO(
                  LOGGER,
                  "[%s] coordinated target FOUND: Ljoint=%d d=%+.3f, Rjoint=%d d=%+.3f, joints=%zu points=%zu",
                  phase_name.c_str(),
                  li, sign_l * d,
                  ri, sign_r * d,
                  plan.trajectory_.joint_trajectory.joint_names.size(),
                  plan.trajectory_.joint_trajectory.points.size());

              if (!node_->get_parameter("execute").as_bool())
                return true;

              if (dual_group.execute(plan) !=
                  moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(
                    LOGGER, "[%s] EXECUTION FAILED", phase_name.c_str());
                return false;
              }

              RCLCPP_INFO(
                  LOGGER, "[%s] EXECUTION SUCCESS", phase_name.c_str());
              return true;
            }
          }
        }
      }
    }

    RCLCPP_ERROR(
        LOGGER,
        "[%s] khong tim duoc coordinated 12DOF nudge collision-free",
        phase_name.c_str());
    return false;
  }

  void runCoordinationDemo()
  {
    moveit::planning_interface::MoveGroupInterface dual_group(node_, "dual_arms");
    const auto model = dual_group.getRobotModel();
    const auto* left_jmg = model->getJointModelGroup("left_ur_onrobot_manipulator");
    const auto* right_jmg = model->getJointModelGroup("right_ur_onrobot_manipulator");

    if (!left_jmg || !right_jmg) {
      RCLCPP_ERROR(LOGGER, "COORD: thieu arm JointModelGroup");
      return;
    }

    std::vector<std::string> left_hand_links;
    std::vector<std::string> right_hand_links;
    DualTaskTargets targets;

    try {
      RCLCPP_INFO(LOGGER, "COORD: tim PREGRASP/GRASP/LIFT nhanh...");
      targets = preparePickTargetsFast(model, left_hand_links, right_hand_links);
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(LOGGER, "COORD target search failed: %s", e.what());
      return;
    }

    auto home = dual_group.getCurrentState(5.0);
    if (!home) {
      RCLCPP_ERROR(LOGGER, "COORD: khong doc duoc current state");
      return;
    }

    auto applyGoal = [](moveit::core::RobotState& state,
                        const std::map<std::string, double>& goal) {
      for (const auto& kv : goal)
        state.setVariablePosition(kv.first, kv.second);
      state.update();
    };

    moveit::core::RobotState pre(*home);
    moveit::core::RobotState grasp(*home);
    moveit::core::RobotState lift(*home);
    applyGoal(pre, targets.pregrasp);
    applyGoal(grasp, targets.grasp);
    applyGoal(lift, targets.lift);

    // Same safe contact point that worked in pose_demo: approach toward grasp,
    // but do not force the arm through the object before attach.
    constexpr double CONTACT_FRACTION = 0.55;
    std::vector<double> left_pre_q, right_pre_q, left_grasp_q, right_grasp_q;
    pre.copyJointGroupPositions(left_jmg, left_pre_q);
    pre.copyJointGroupPositions(right_jmg, right_pre_q);
    grasp.copyJointGroupPositions(left_jmg, left_grasp_q);
    grasp.copyJointGroupPositions(right_jmg, right_grasp_q);

    std::vector<double> left_contact_q(left_pre_q.size());
    std::vector<double> right_contact_q(right_pre_q.size());
    for (std::size_t i = 0; i < left_pre_q.size(); ++i)
      left_contact_q[i] = left_pre_q[i] + CONTACT_FRACTION * (left_grasp_q[i] - left_pre_q[i]);
    for (std::size_t i = 0; i < right_pre_q.size(); ++i)
      right_contact_q[i] = right_pre_q[i] + CONTACT_FRACTION * (right_grasp_q[i] - right_pre_q[i]);

    moveit::core::RobotState contact(pre);
    contact.setJointGroupPositions(left_jmg, left_contact_q);
    contact.setJointGroupPositions(right_jmg, right_contact_q);
    contact.update();

    const double center_shift = node_->get_parameter("coord_center_shift").as_double();
    const double both_dz = node_->get_parameter("coord_both_dz").as_double();
    const double rotate_rad = node_->get_parameter("coord_rotate_deg").as_double() * M_PI / 180.0;

    auto pause = []() { rclcpp::sleep_for(std::chrono::milliseconds(700)); };

    RCLCPP_INFO(LOGGER, "==========================================");
    RCLCPP_INFO(LOGGER, " COORDINATION DEMO START");
    RCLCPP_INFO(LOGGER, " P1 OPEN BOTH");
    RCLCPP_INFO(LOGGER, " P2 BOTH -> PREGRASP");
    RCLCPP_INFO(LOGGER, " P3 BOTH -> CONTACT");
    RCLCPP_INFO(LOGGER, " P4 CLOSE + ATTACH");
    RCLCPP_INFO(LOGGER, " P5 BOTH LIFT");
    RCLCPP_INFO(LOGGER, " P6 LEFT HOLD, RIGHT MOVE TO CENTER");
    RCLCPP_INFO(LOGGER, " P7 RIGHT HOLD, LEFT MOVE TO CENTER");
    RCLCPP_INFO(LOGGER, " P8 BOTH MOVE TOGETHER (12DOF coordinated)");
    RCLCPP_INFO(LOGGER, " P9 LEFT HOLD, RIGHT ROTATE OBJECT");
    RCLCPP_INFO(LOGGER, " P10 RIGHT HOLD, LEFT ROTATE OBJECT");
    RCLCPP_INFO(LOGGER, " P11 BOTH RETURN TO LIFT");
    RCLCPP_INFO(LOGGER, " P12 PUT BACK + OPEN + DETACH + HOME");
    RCLCPP_INFO(LOGGER, "==========================================");

    if (!executeDualGrippers(0.100, "COORD P1 OPEN BOTH"))
      return;

    if (!planExecuteDualState(dual_group, *home, pre, HoldMode::NONE,
                              "COORD P2 PARALLEL PREGRASP"))
      return;

    if (!node_->get_parameter("execute").as_bool()) {
      RCLCPP_WARN(LOGGER, "COORD execute:=false, stop after P2 plan-only");
      return;
    }
    pause();

    auto s2 = dual_group.getCurrentState(5.0);
    if (!s2)
      return;
    if (!planExecuteDualState(dual_group, *s2, contact, HoldMode::NONE,
                              "COORD P3 PARALLEL CONTACT"))
      return;
    pause();

    if (!executeDualGrippers(0.0, "COORD P4 CLOSE BOTH"))
      return;

    const bool left_attached =
        dual_group.attachObject("left_object", "left_gripper_tcp", left_hand_links);
    const bool right_attached =
        dual_group.attachObject("right_object", "right_gripper_tcp", right_hand_links);

    if (!left_attached || !right_attached) {
      RCLCPP_ERROR(LOGGER, "COORD attach failed: LEFT=%s RIGHT=%s",
                   left_attached ? "OK" : "FAIL",
                   right_attached ? "OK" : "FAIL");
      return;
    }
    RCLCPP_INFO(LOGGER, "COORD P4 ATTACH SUCCESS");
    pause();

    auto s4 = dual_group.getCurrentState(5.0);
    if (!s4)
      return;
    if (!planExecuteDualState(dual_group, *s4, lift, HoldMode::NONE,
                              "COORD P5 PARALLEL LIFT"))
      return;
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Save the real post-lift state. All later coordinated interaction returns here.
    auto lift_reference_ptr = dual_group.getCurrentState(5.0);
    if (!lift_reference_ptr)
      return;
    moveit::core::RobotState lift_reference(*lift_reference_ptr);

    // P6: LEFT is hard-constrained. RIGHT carries object B toward shared workspace.
    auto s6 = dual_group.getCurrentState(5.0);
    if (!s6)
      return;
    Eigen::Isometry3d right_center = s6->getGlobalLinkTransform("right_gripper_tcp");
    right_center.translation().y() += center_shift;
    moveit::core::RobotState p6(*s6);
    if (!buildDualPoseTargetIKOnly(*s6, nullptr, &right_center, p6)) {
      RCLCPP_ERROR(LOGGER, "COORD P6: RIGHT center IK failed");
      return;
    }
    if (!planExecuteDualState(dual_group, *s6, p6, HoldMode::HOLD_LEFT,
                              "COORD P6 LEFT HOLD + RIGHT TO CENTER"))
      return;
    pause();

    // P7: switch roles. RIGHT is constrained; LEFT carries object A toward center.
    auto s7 = dual_group.getCurrentState(5.0);
    if (!s7)
      return;
    Eigen::Isometry3d left_center = s7->getGlobalLinkTransform("left_gripper_tcp");
    left_center.translation().y() -= center_shift;
    moveit::core::RobotState p7(*s7);
    if (!buildDualPoseTargetIKOnly(*s7, &left_center, nullptr, p7)) {
      RCLCPP_ERROR(LOGGER, "COORD P7: LEFT center IK failed");
      return;
    }
    if (!planExecuteDualState(dual_group, *s7, p7, HoldMode::HOLD_RIGHT,
                              "COORD P7 RIGHT HOLD + LEFT TO CENTER"))
      return;
    pause();

    // P8: both arms move together in one 12DOF trajectory.
    // Avoid fragile Cartesian +Z dual IK after P6/P7; use a robust
    // coordinated joint-space motion instead.
    const double coord_joint_delta =
        std::clamp(both_dz * 2.0, 0.03, 0.10);

    if (!coordinatedJointNudge(
            dual_group,
            coord_joint_delta,
            "COORD P8 BOTH MOVE TOGETHER 12DOF"))
      return;
    pause();

    // P9: LEFT holds object A, RIGHT rotates object B around its local tool Z.
    auto s9 = dual_group.getCurrentState(5.0);
    if (!s9)
      return;
    Eigen::Isometry3d right_rotate = s9->getGlobalLinkTransform("right_gripper_tcp");
    right_rotate.linear() =
        right_rotate.linear() *
        Eigen::AngleAxisd(rotate_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    moveit::core::RobotState p9(*s9);
    if (!buildDualPoseTargetIKOnly(*s9, nullptr, &right_rotate, p9)) {
      RCLCPP_ERROR(LOGGER, "COORD P9: RIGHT rotate IK failed");
      return;
    }
    if (!planExecuteDualState(dual_group, *s9, p9, HoldMode::HOLD_LEFT,
                              "COORD P9 LEFT HOLD + RIGHT ROTATE"))
      return;
    pause();

    // P10: switch roles again. RIGHT holds, LEFT rotates the other object.
    auto s10 = dual_group.getCurrentState(5.0);
    if (!s10)
      return;
    Eigen::Isometry3d left_rotate = s10->getGlobalLinkTransform("left_gripper_tcp");
    left_rotate.linear() =
        left_rotate.linear() *
        Eigen::AngleAxisd(-rotate_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    moveit::core::RobotState p10(*s10);
    if (!buildDualPoseTargetIKOnly(*s10, &left_rotate, nullptr, p10)) {
      RCLCPP_ERROR(LOGGER, "COORD P10: LEFT rotate IK failed");
      return;
    }
    if (!planExecuteDualState(dual_group, *s10, p10, HoldMode::HOLD_RIGHT,
                              "COORD P10 RIGHT HOLD + LEFT ROTATE"))
      return;
    rclcpp::sleep_for(std::chrono::seconds(1));

    // P11: both arms return together to the post-lift state.
    auto s11 = dual_group.getCurrentState(5.0);
    if (!s11)
      return;
    if (!planExecuteDualState(dual_group, *s11, lift_reference, HoldMode::NONE,
                              "COORD P11 PARALLEL RETURN TO LIFT"))
      return;
    pause();

    // P12: lower to the same safe contact state, release both objects, retreat and home.
    auto s12 = dual_group.getCurrentState(5.0);
    if (!s12)
      return;
    if (!planExecuteDualState(dual_group, *s12, contact, HoldMode::NONE,
                              "COORD P12 PARALLEL PUT BACK"))
      return;
    pause();

    if (!executeDualGrippers(0.100, "COORD P12 OPEN BOTH"))
      return;

    const bool left_detached = dual_group.detachObject("left_object");
    const bool right_detached = dual_group.detachObject("right_object");
    RCLCPP_INFO(LOGGER, "COORD DETACH: LEFT=%s RIGHT=%s",
                left_detached ? "OK" : "FAIL",
                right_detached ? "OK" : "FAIL");

    auto s13 = dual_group.getCurrentState(5.0);
    if (!s13)
      return;
    if (!planExecuteDualState(dual_group, *s13, pre, HoldMode::NONE,
                              "COORD P12 PARALLEL RETREAT"))
      return;

    auto s14 = dual_group.getCurrentState(5.0);
    if (!s14)
      return;
    if (!planExecuteDualState(dual_group, *s14, *home, HoldMode::NONE,
                              "COORD P12 PARALLEL HOME"))
      return;

    RCLCPP_INFO(LOGGER, "==========================================");
    RCLCPP_INFO(LOGGER, " COORDINATION DEMO SUCCESS");
    RCLCPP_INFO(LOGGER, " PARALLEL -> HOLD/ROLE SWITCH -> COORDINATED -> ROTATE -> ROLE SWITCH PASS");
    RCLCPP_INFO(LOGGER, "==========================================");
  }

private:
  // ========================================================================
  // HIGH-LEVEL COMMAND SERVER
  // ========================================================================

  static const char* armSideName(ArmSide arm)
  {
    switch (arm) {
      case ArmSide::LEFT: return "LEFT";
      case ArmSide::RIGHT: return "RIGHT";
      case ArmSide::AUTO: return "AUTO";
      default: return "NONE";
    }
  }

  static ArmSide parseArmSide(const std::string& text)
  {
    if (text == "left" || text == "LEFT" || text == "l")
      return ArmSide::LEFT;
    if (text == "right" || text == "RIGHT" || text == "r")
      return ArmSide::RIGHT;
    if (text == "auto" || text == "AUTO" || text.empty())
      return ArmSide::AUTO;
    return ArmSide::NONE;
  }

  static std::string normalizeObjectName(const std::string& name)
  {
    if (name == "A" || name == "a") return "stack_A";
    if (name == "B" || name == "b") return "stack_B";
    if (name == "C" || name == "c") return "stack_C";
    if (name == "D" || name == "d") return "stack_D";
    return name;
  }

  ArmConfig armConfig(ArmSide arm, const std::string& object_name) const
  {
    if (arm == ArmSide::LEFT) {
      return ArmConfig{
          "left_ur_onrobot_manipulator",
          "left_ur_onrobot_gripper",
          "left_gripper_tcp",
          object_name};
    }
    return ArmConfig{
        "right_ur_onrobot_manipulator",
        "right_ur_onrobot_gripper",
        "right_gripper_tcp",
        object_name};
  }

  HeldObject& heldForArm(ArmSide arm)
  {
    return arm == ArmSide::LEFT ? left_held_ : right_held_;
  }

  const HeldObject& heldForArm(ArmSide arm) const
  {
    return arm == ArmSide::LEFT ? left_held_ : right_held_;
  }

  ArmSide holdingArm(const std::string& object_name) const
  {
    if (left_held_.valid && left_held_.object_name == object_name)
      return ArmSide::LEFT;
    if (right_held_.valid && right_held_.object_name == object_name)
      return ArmSide::RIGHT;
    return ArmSide::NONE;
  }

  void enqueueCommand(const std::string& command)
  {
    {
      std::lock_guard<std::mutex> lock(command_queue_mutex_);
      command_queue_.push_back(command);
    }
    command_cv_.notify_all();
    RCLCPP_INFO(LOGGER, "COMMAND QUEUED: %s", command.c_str());
  }

  void commandWorkerLoop()
  {
    while (true) {
      std::string command;
      {
        std::unique_lock<std::mutex> lock(command_queue_mutex_);
        command_cv_.wait(lock, [this]() {
          return command_worker_stop_ ||
                 (command_server_ready_.load() && !command_queue_.empty());
        });

        if (command_worker_stop_)
          return;

        command = command_queue_.front();
        command_queue_.pop_front();
      }

      RCLCPP_INFO(LOGGER, "==========================================");
      RCLCPP_INFO(LOGGER, " COMMAND START: %s", command.c_str());
      RCLCPP_INFO(LOGGER, "==========================================");

      const bool ok = executeCommandString(command);

      if (ok)
        RCLCPP_INFO(LOGGER, "COMMAND SUCCESS: %s", command.c_str());
      else
        RCLCPP_ERROR(LOGGER, "COMMAND FAILED: %s", command.c_str());
    }
  }

  void printCommandHelp()
  {
    RCLCPP_INFO(LOGGER, "==========================================");
    RCLCPP_INFO(LOGGER, " DUAL ARM TASK CONTROLLER");
    RCLCPP_INFO(LOGGER, " Start CLI in another terminal: ros2 run ur_onrobot_mtc robot_cli");
    RCLCPP_INFO(LOGGER, " CLI publishes std_msgs/String on /robot_command");
    RCLCPP_INFO(LOGGER, " Object aliases A/B/C/D are accepted as stack_A/B/C/D");
    RCLCPP_INFO(LOGGER, " Commands:");
    RCLCPP_INFO(LOGGER, "   pick <A|B|C|D|object> [auto|left|right]");
    RCLCPP_INFO(LOGGER, "   place <object> <x> <y> <z> [yaw_deg]");
    RCLCPP_INFO(LOGGER, "   move <object> <x> <y> <z> [yaw_deg]");
    RCLCPP_INFO(LOGGER, "   stack <object1> <object2> ...");
    RCLCPP_INFO(LOGGER, "   stack_at <x> <y> <object1> <object2> ...");
    RCLCPP_INFO(LOGGER, "   swap <objectA> <objectB>");
    RCLCPP_INFO(LOGGER, "   status");
    RCLCPP_INFO(LOGGER, "   reset_scene");
    RCLCPP_INFO(LOGGER, "   help");
    RCLCPP_INFO(LOGGER, "==========================================");
  }

  bool parseDouble(const std::string& text, double& value) const
  {
    char* end = nullptr;
    const double v = std::strtod(text.c_str(), &end);
    if (!end || end == text.c_str() || *end != '\0')
      return false;
    value = v;
    return true;
  }

  bool executeCommandString(const std::string& command)
  {
    std::istringstream ss(command);
    std::string op;
    ss >> op;

    if (op.empty())
      return false;

    if (op == "help") {
      printCommandHelp();
      return true;
    }

    if (op == "status") {
      printSystemStatus();
      return true;
    }

    if (op == "reset_scene") {
      if (left_held_.valid || right_held_.valid) {
        RCLCPP_ERROR(LOGGER, "reset_scene refused: an object is currently held");
        return false;
      }
      try {
        stack_column_yaw_ = std::numeric_limits<double>::quiet_NaN();
        stack_table_top_ = setupStack4Scene();
        return true;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "reset_scene failed: %s", e.what());
        return false;
      }
    }

    if (op == "pick") {
      std::string object_name;
      std::string arm_text = "auto";
      ss >> object_name;
      if (object_name.empty()) {
        RCLCPP_ERROR(LOGGER, "Usage: pick <object> [auto|left|right]");
        return false;
      }
      if (ss >> arm_text) {
        // optional arm parsed below
      }
      const ArmSide requested = parseArmSide(arm_text);
      if (requested == ArmSide::NONE) {
        RCLCPP_ERROR(LOGGER, "Invalid arm '%s'", arm_text.c_str());
        return false;
      }
      object_name = normalizeObjectName(object_name);
      return pickObject(object_name, requested);
    }

    if (op == "place") {
      std::string object_name;
      double x = 0.0, y = 0.0, z = 0.0;
      if (!(ss >> object_name >> x >> y >> z)) {
        RCLCPP_ERROR(LOGGER, "Usage: place <object> <x> <y> <z> [yaw_deg]");
        return false;
      }
      double yaw_rad = std::numeric_limits<double>::quiet_NaN();
      double yaw_deg = 0.0;
      if (ss >> yaw_deg)
        yaw_rad = yaw_deg * M_PI / 180.0;
      object_name = normalizeObjectName(object_name);
      return placeObject(object_name, x, y, z, yaw_rad);
    }

    if (op == "move") {
      std::string object_name;
      double x = 0.0, y = 0.0, z = 0.0;
      if (!(ss >> object_name >> x >> y >> z)) {
        RCLCPP_ERROR(LOGGER, "Usage: move <object> <x> <y> <z> [yaw_deg]");
        return false;
      }
      double yaw_rad = std::numeric_limits<double>::quiet_NaN();
      double yaw_deg = 0.0;
      if (ss >> yaw_deg)
        yaw_rad = yaw_deg * M_PI / 180.0;
      object_name = normalizeObjectName(object_name);
      return moveObject(object_name, x, y, z, yaw_rad);
    }

    if (op == "stack" || op == "stack_at") {
      double cx = node_->get_parameter("stack_center_x").as_double();
      double cy = node_->get_parameter("stack_center_y").as_double();

      if (op == "stack_at") {
        if (!(ss >> cx >> cy)) {
          RCLCPP_ERROR(LOGGER, "Usage: stack_at <x> <y> <object1> <object2> ...");
          return false;
        }
      }

      std::vector<std::string> objects;
      std::string object_name;
      while (ss >> object_name)
        objects.push_back(normalizeObjectName(object_name));

      if (objects.empty()) {
        RCLCPP_ERROR(LOGGER, "Usage: stack <object1> <object2> ...");
        return false;
      }
      return stackObjects(objects, cx, cy);
    }

    if (op == "swap") {
      std::string a, b;
      if (!(ss >> a >> b)) {
        RCLCPP_ERROR(LOGGER, "Usage: swap <objectA> <objectB>");
        return false;
      }
      a = normalizeObjectName(a);
      b = normalizeObjectName(b);
      return swapObjects(a, b);
    }

    RCLCPP_ERROR(LOGGER, "Unknown command '%s'. Type 'help' in robot_cli", op.c_str());
    return false;
  }

  void printSystemStatus()
  {
    moveit::planning_interface::PlanningSceneInterface psi;
    const auto objects = psi.getObjects();

    RCLCPP_INFO(LOGGER, "--------------- SYSTEM STATUS ---------------");
    RCLCPP_INFO(LOGGER, "LEFT : %s%s%s",
                left_held_.valid ? "HOLDING " : "FREE",
                left_held_.valid ? left_held_.object_name.c_str() : "",
                left_held_.valid ? "" : "");
    RCLCPP_INFO(LOGGER, "RIGHT: %s%s%s",
                right_held_.valid ? "HOLDING " : "FREE",
                right_held_.valid ? right_held_.object_name.c_str() : "",
                right_held_.valid ? "" : "");

    for (const auto& kv : objects) {
      if (kv.first == "table" || kv.second.primitive_poses.empty())
        continue;
      const auto& p = kv.second.primitive_poses.front();
      RCLCPP_INFO(LOGGER, "WORLD %-12s (%.3f, %.3f, %.3f)",
                  kv.first.c_str(), p.position.x, p.position.y, p.position.z);
    }
    RCLCPP_INFO(LOGGER, "---------------------------------------------");
  }

  bool getObjectWorldPose(const std::string& object_name, Eigen::Isometry3d& pose)
  {
    moveit::planning_interface::PlanningSceneInterface psi;
    const auto poses = psi.getObjectPoses({object_name});
    const auto it = poses.find(object_name);
    if (it == poses.end())
      return false;
    pose = poseToEigen(it->second);
    return true;
  }

  bool canPlanArmToState(
      const moveit::core::RobotState& current,
      const moveit::core::RobotState& target,
      ArmSide arm)
  {
    const auto model = current.getRobotModel();
    const ArmConfig cfg = armConfig(arm, "");
    const auto* jmg = model->getJointModelGroup(cfg.arm_group);
    if (!jmg)
      return false;

    std::vector<double> q;
    target.copyJointGroupPositions(jmg, q);

    moveit::planning_interface::MoveGroupInterface group(node_, cfg.arm_group);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPlanningTime(3.0);
    group.setNumPlanningAttempts(2);
    group.setMaxVelocityScalingFactor(0.30);
    group.setMaxAccelerationScalingFactor(0.25);
    group.setStartState(current);
    group.setJointValueTarget(q);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    return group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  ArmEvaluation evaluateArmForObject(
      const std::string& object_name,
      ArmSide arm)
  {
    moveit::planning_interface::MoveGroupInterface dual_group(node_, "dual_arms");
    const auto model = dual_group.getRobotModel();
    ArmEvaluation failed(model);
    failed.arm = arm;

    if (arm != ArmSide::LEFT && arm != ArmSide::RIGHT)
      return failed;

    if (heldForArm(arm).valid) {
      RCLCPP_WARN(LOGGER, "EVAL %s: arm is busy holding %s",
                  armSideName(arm), heldForArm(arm).object_name.c_str());
      return failed;
    }

    auto current = dual_group.getCurrentState(5.0);
    if (!current)
      return failed;
    current->update();

    moveit::planning_interface::PlanningSceneInterface psi;
    const auto object_poses = psi.getObjectPoses({object_name});
    if (!object_poses.count(object_name)) {
      RCLCPP_WARN(LOGGER, "EVAL %s: object '%s' is not a world object",
                  armSideName(arm), object_name.c_str());
      return failed;
    }

    const Eigen::Isometry3d world_object = poseToEigen(object_poses.at(object_name));
    const ArmConfig cfg = armConfig(arm, object_name);

    const auto* arm_jmg = model->getJointModelGroup(cfg.arm_group);
    const auto* dual_jmg = model->getJointModelGroup("dual_arms");
    const auto* hand_jmg = model->getJointModelGroup(cfg.hand_group);
    if (!arm_jmg || !dual_jmg || !hand_jmg)
      return failed;

    auto scene = std::make_shared<planning_scene::PlanningScene>(model);
    scene->setCurrentState(*current);
    const auto all_objects = psi.getObjects();
    for (const auto& kv : all_objects)
      scene->processCollisionObjectMsg(kv.second);

    const auto hand_links = hand_jmg->getLinkModelNamesWithCollisionGeometry();
    auto& acm = scene->getAllowedCollisionMatrixNonConst();
    acm.setEntry(object_name, hand_links, true);
    acm.setEntry(hand_links, hand_links, true);

    constexpr double APPROACH_DISTANCE = 0.07;
    constexpr double LIFT_DISTANCE = 0.10;

    std::vector<ArmEvaluation> candidates;
    candidates.reserve(36);

    const Eigen::Vector3d tcp_now =
        current->getGlobalLinkTransform(cfg.hand_frame).translation();
    const double cartesian_distance =
        (tcp_now - world_object.translation()).norm();

    for (int idx = 0; idx < 36; ++idx) {
      const double yaw = idx * M_PI / 18.0;

      Eigen::Isometry3d world_hand_grasp;
      if (arm == ArmSide::LEFT)
        world_hand_grasp = leftStackHandPose(world_object, yaw);
      else if (object_name == "stack_D")
        world_hand_grasp = rightHighStackHandPose(world_object, yaw);
      else
        world_hand_grasp = topDownHandPose(world_object, yaw);

      const Eigen::Isometry3d world_hand_pregrasp =
          world_hand_grasp * Eigen::Translation3d(0.0, 0.0, -APPROACH_DISTANCE);

      moveit::core::RobotState grasp_solution(model);
      if (!solveIKCollisionFree(scene, cfg, world_hand_grasp, *current, grasp_solution, 6))
        continue;

      moveit::core::RobotState pre_solution(model);
      if (!solveIKCollisionFree(scene, cfg, world_hand_pregrasp, grasp_solution, pre_solution, 6))
        continue;

      std::vector<double> pre_q;
      pre_solution.copyJointGroupPositions(arm_jmg, pre_q);
      normalizeGroupNearState(model, arm_jmg, *current, pre_q);
      moveit::core::RobotState pre(*current);
      pre.setJointGroupPositions(arm_jmg, pre_q);
      pre.update();

      std::vector<double> grasp_q;
      grasp_solution.copyJointGroupPositions(arm_jmg, grasp_q);
      normalizeGroupNearState(model, arm_jmg, pre, grasp_q);
      moveit::core::RobotState grasp(pre);
      grasp.setJointGroupPositions(arm_jmg, grasp_q);
      grasp.update();

      if (!pre.satisfiesBounds(dual_jmg) || !grasp.satisfiesBounds(dual_jmg))
        continue;
      if (scene->isStateColliding(pre, "", false) ||
          scene->isStateColliding(grasp, "", false))
        continue;
      if (!validateApproach(scene, cfg, world_hand_grasp, pre))
        continue;
      if (!validateDualInterpolation(scene, pre, grasp, dual_jmg, 20))
        continue;

      Eigen::Isometry3d world_hand_lift = world_hand_grasp;
      world_hand_lift.translation().z() += LIFT_DISTANCE;
      moveit::core::RobotState lift_solution(model);
      if (!solveIKCollisionFree(scene, cfg, world_hand_lift, grasp, lift_solution, 8))
        continue;

      std::vector<double> lift_q;
      lift_solution.copyJointGroupPositions(arm_jmg, lift_q);
      normalizeGroupNearState(model, arm_jmg, grasp, lift_q);
      moveit::core::RobotState lift(grasp);
      lift.setJointGroupPositions(arm_jmg, lift_q);
      lift.update();

      if (!lift.satisfiesBounds(dual_jmg) || scene->isStateColliding(lift, "", false))
        continue;
      if (!validateDualInterpolation(scene, grasp, lift, dual_jmg, 20))
        continue;

      std::vector<double> current_q;
      current->copyJointGroupPositions(arm_jmg, current_q);

      double joint_sum = 0.0;
      double max_delta = 0.0;
      for (std::size_t i = 0; i < current_q.size(); ++i) {
        const double d0 = std::abs(pre_q[i] - current_q[i]);
        const double d1 = std::abs(grasp_q[i] - pre_q[i]);
        const double d2 = std::abs(lift_q[i] - grasp_q[i]);
        joint_sum += d0 + d1 + d2;
        max_delta = std::max(max_delta, std::max(d0, std::max(d1, d2)));
      }

      ArmEvaluation ev(model);
      ev.arm = arm;
      ev.ik_ok = true;
      ev.collision_free = true;
      ev.approach_ok = true;
      ev.cartesian_distance = cartesian_distance;
      ev.joint_cost = joint_sum;
      ev.max_joint_delta = max_delta;
      ev.total_cost = cartesian_distance + joint_sum + 2.0 * max_delta;
      ev.grasp_yaw = yaw;
      ev.pregrasp = pre;
      ev.grasp = grasp;
      ev.lift = lift;
      candidates.push_back(ev);
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const ArmEvaluation& a, const ArmEvaluation& b) {
                return a.total_cost < b.total_cost;
              });

    const std::size_t plan_checks = std::min<std::size_t>(candidates.size(), 8);
    for (std::size_t i = 0; i < plan_checks; ++i) {
      if (!canPlanArmToState(*current, candidates[i].pregrasp, arm))
        continue;
      candidates[i].plan_ok = true;
      candidates[i].reachable = true;
      RCLCPP_INFO(LOGGER,
                  "EVAL %s object=%s FEASIBLE yaw=%.0f dist=%.3f joint=%.3f max=%.3f total=%.3f",
                  armSideName(arm), object_name.c_str(),
                  candidates[i].grasp_yaw * 180.0 / M_PI,
                  candidates[i].cartesian_distance,
                  candidates[i].joint_cost,
                  candidates[i].max_joint_delta,
                  candidates[i].total_cost);
      return candidates[i];
    }

    RCLCPP_WARN(LOGGER, "EVAL %s object=%s NOT FEASIBLE",
                armSideName(arm), object_name.c_str());
    return failed;
  }

  ArmSide selectBestArm(const ArmEvaluation& left, const ArmEvaluation& right) const
  {
    if (!left.reachable && !right.reachable)
      return ArmSide::NONE;
    if (left.reachable && !right.reachable)
      return ArmSide::LEFT;
    if (!left.reachable && right.reachable)
      return ArmSide::RIGHT;
    return left.total_cost <= right.total_cost ? ArmSide::LEFT : ArmSide::RIGHT;
  }

  bool pickObject(const std::string& object_name, ArmSide requested_arm = ArmSide::AUTO)
  {
    moveit::planning_interface::MoveGroupInterface dual_group(node_, "dual_arms");
    const auto model = dual_group.getRobotModel();

    if (holdingArm(object_name) != ArmSide::NONE) {
      RCLCPP_ERROR(LOGGER, "PICK: %s is already held", object_name.c_str());
      return false;
    }

    ArmEvaluation left(model);
    ArmEvaluation right(model);
    left.arm = ArmSide::LEFT;
    right.arm = ArmSide::RIGHT;

    ArmSide selected = ArmSide::NONE;
    ArmEvaluation* chosen = nullptr;

    if (requested_arm == ArmSide::LEFT) {
      left = evaluateArmForObject(object_name, ArmSide::LEFT);
      selected = left.reachable ? ArmSide::LEFT : ArmSide::NONE;
      chosen = &left;
    }
    else if (requested_arm == ArmSide::RIGHT) {
      right = evaluateArmForObject(object_name, ArmSide::RIGHT);
      selected = right.reachable ? ArmSide::RIGHT : ArmSide::NONE;
      chosen = &right;
    }
    else {
      left = evaluateArmForObject(object_name, ArmSide::LEFT);
      right = evaluateArmForObject(object_name, ArmSide::RIGHT);
      selected = selectBestArm(left, right);
      if (selected == ArmSide::LEFT) chosen = &left;
      if (selected == ArmSide::RIGHT) chosen = &right;
    }

    RCLCPP_INFO(LOGGER, "PICK SCHEDULER object=%s requested=%s selected=%s",
                object_name.c_str(), armSideName(requested_arm), armSideName(selected));

    if (!chosen || selected == ArmSide::NONE || !chosen->reachable) {
      RCLCPP_ERROR(LOGGER, "PICK %s: no feasible arm", object_name.c_str());
      return false;
    }

    const bool move_left = selected == ArmSide::LEFT;
    const HoldMode hold_mode = move_left ? HoldMode::HOLD_RIGHT : HoldMode::HOLD_LEFT;
    const ArmConfig cfg = armConfig(selected, object_name);

    if (!executeOneGripper(move_left, 0.100, "PICK OPEN " + std::string(armSideName(selected))))
      return false;

    auto current = dual_group.getCurrentState(5.0);
    if (!current)
      return false;
    if (!planExecuteDualState(dual_group, *current, chosen->pregrasp, hold_mode,
                              "PICK " + std::string(armSideName(selected)) + " PREGRASP"))
      return false;

    // Plan-only mode stops before mutating the PlanningScene.
    if (!node_->get_parameter("execute").as_bool()) {
      if (!planExecuteDualState(dual_group, chosen->pregrasp, chosen->grasp, hold_mode,
                                "PICK PLAN-ONLY GRASP"))
        return false;
      if (!planExecuteDualState(dual_group, chosen->grasp, chosen->lift, hold_mode,
                                "PICK PLAN-ONLY LIFT"))
        return false;
      RCLCPP_INFO(LOGGER, "PICK plan-only complete; no attach performed");
      return true;
    }

    current = dual_group.getCurrentState(5.0);
    if (!current)
      return false;
    if (!planExecuteDualState(dual_group, *current, chosen->grasp, hold_mode,
                              "PICK " + std::string(armSideName(selected)) + " GRASP"))
      return false;

    auto attach_state = dual_group.getCurrentState(5.0);
    if (!attach_state)
      return false;
    attach_state->update();

    Eigen::Isometry3d world_object;
    if (!getObjectWorldPose(object_name, world_object)) {
      RCLCPP_ERROR(LOGGER, "PICK: cannot read object pose before attach");
      return false;
    }

    const Eigen::Isometry3d hand_to_object =
        attach_state->getGlobalLinkTransform(cfg.hand_frame).inverse() * world_object;

    const auto* hand_jmg = model->getJointModelGroup(cfg.hand_group);
    if (!hand_jmg)
      return false;
    const auto touch_links = hand_jmg->getLinkModelNamesWithCollisionGeometry();

    if (!dual_group.attachObject(object_name, cfg.hand_frame, touch_links)) {
      RCLCPP_ERROR(LOGGER, "PICK: attach %s to %s failed",
                   object_name.c_str(), cfg.hand_frame.c_str());
      return false;
    }

    if (!executeOneGripper(move_left, 0.0,
                           "PICK CLOSE " + std::string(armSideName(selected)))) {
      dual_group.detachObject(object_name);
      return false;
    }

    HeldObject& held = heldForArm(selected);
    held.valid = true;
    held.object_name = object_name;
    held.arm = selected;
    held.hand_to_object = hand_to_object;

    current = dual_group.getCurrentState(5.0);
    if (!current)
      return false;
    if (!planExecuteDualState(dual_group, *current, chosen->lift, hold_mode,
                              "PICK " + std::string(armSideName(selected)) + " LIFT")) {
      RCLCPP_ERROR(LOGGER, "PICK lift failed; object remains attached to %s",
                   armSideName(selected));
      return false;
    }

    RCLCPP_INFO(LOGGER, "PICK SUCCESS object=%s arm=%s cost=%.3f",
                object_name.c_str(), armSideName(selected), chosen->total_cost);
    return true;
  }

  bool tryPlaceAttachedObjectMTC(
      ArmSide arm,
      const std::string& object_name,
      double x,
      double y,
      double z,
      double yaw,
      const std::string& support_object,
      bool& execution_started)
  {
    execution_started = false;
    const ArmConfig cfg = armConfig(arm, object_name);

    mtc::Task t;
    t.stages()->setName("command_place_" + object_name);
    t.loadRobotModel(node_);
    t.setProperty("group", cfg.arm_group);
    t.setProperty("eef", cfg.hand_group);
    t.setProperty("ik_frame", cfg.hand_frame);

    mtc::Stage* current_state_ptr = nullptr;
    auto current = std::make_unique<mtc::stages::CurrentState>("current attached state");
    current_state_ptr = current.get();
    t.add(std::move(current));

    mtc::Stage* place_monitor_ptr = current_state_ptr;
    const auto* hand_jmg = t.getRobotModel()->getJointModelGroup(cfg.hand_group);
    if (!hand_jmg)
      return false;
    const auto hand_links = hand_jmg->getLinkModelNamesWithCollisionGeometry();

    if (!support_object.empty()) {
      auto allow_support = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow support contact");
      allow_support->allowCollisions(support_object, hand_links, true);
      allow_support->allowCollisions(object_name, support_object, true);
      place_monitor_ptr = allow_support.get();
      t.add(std::move(allow_support));
    }

    auto sampling = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    sampling->setProperty("planning_pipeline", "ompl");
    sampling->setPlannerId("RRTConnectkConfigDefault");

    auto interpolation = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    {
      auto connect = std::make_unique<mtc::stages::Connect>(
          "move active arm to place",
          mtc::stages::Connect::GroupPlannerVector{{cfg.arm_group, sampling}});
      connect->setTimeout(10.0);
      connect->properties().configureInitFrom(mtc::Stage::PARENT);
      t.add(std::move(connect));
    }

    {
      auto place = std::make_unique<mtc::SerialContainer>("place attached object");
      t.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
      place->properties().configureInitFrom(
          mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

      auto gen = std::make_unique<mtc::stages::GeneratePlacePose>("generate target object pose");
      gen->properties().configureInitFrom(mtc::Stage::PARENT);
      gen->properties().set("marker_ns", "command_place_pose");
      gen->setObject(object_name);

      geometry_msgs::msg::PoseStamped target;
      target.header.frame_id = "world";
      target.pose.position.x = x;
      target.pose.position.y = y;
      target.pose.position.z = z;
      Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
      target.pose.orientation.x = q.x();
      target.pose.orientation.y = q.y();
      target.pose.orientation.z = q.z();
      target.pose.orientation.w = q.w();
      gen->setPose(target);
      gen->setMonitoredStage(place_monitor_ptr);

      auto ik = std::make_unique<mtc::stages::ComputeIK>(
          "compute place IK using attached object frame", std::move(gen));
      ik->setMaxIKSolutions(16);
      ik->setMinSolutionDistance(0.15);
      ik->setIKFrame(object_name);
      ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(ik));

      auto open = std::make_unique<mtc::stages::MoveTo>("open active gripper", interpolation);
      open->setGroup(cfg.hand_group);
      open->setGoal("open");
      place->insert(std::move(open));

      auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      detach->detachObject(object_name, cfg.hand_frame);
      place->insert(std::move(detach));

      if (!support_object.empty()) {
        auto forbid_support = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "restore support collisions");
        forbid_support->allowCollisions(support_object, hand_links, false);
        forbid_support->allowCollisions(object_name, support_object, false);
        place->insert(std::move(forbid_support));
      }

      t.add(std::move(place));
    }

    try {
      t.init();
    } catch (const mtc::InitStageException& e) {
      RCLCPP_ERROR_STREAM(LOGGER, "COMMAND PLACE init failed:\n" << e);
      return false;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(LOGGER, "COMMAND PLACE init exception: %s", e.what());
      return false;
    }

    if (!t.plan(5) || t.solutions().empty()) {
      RCLCPP_WARN(LOGGER,
                  "PLACE no plan object=%s arm=%s target=(%.3f %.3f %.3f) yaw=%.0f",
                  object_name.c_str(), armSideName(arm), x, y, z, yaw * 180.0 / M_PI);
      return false;
    }

    RCLCPP_INFO(LOGGER,
                "PLACE PLAN SUCCESS object=%s arm=%s target=(%.3f %.3f %.3f) yaw=%.0f",
                object_name.c_str(), armSideName(arm), x, y, z, yaw * 180.0 / M_PI);
    t.introspection().publishSolution(*t.solutions().front());

    if (!node_->get_parameter("execute").as_bool())
      return true;

    execution_started = true;
    const auto result = t.execute(*t.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "PLACE EXECUTION FAILED object=%s", object_name.c_str());
      return false;
    }

    return true;
  }

  bool placeObject(
      const std::string& object_name,
      double x,
      double y,
      double z,
      double yaw)
  {
    const ArmSide arm = holdingArm(object_name);
    if (arm == ArmSide::NONE) {
      RCLCPP_ERROR(LOGGER, "PLACE: object %s is not held by either arm", object_name.c_str());
      return false;
    }

    moveit::planning_interface::MoveGroupInterface dual_group(node_, "dual_arms");
    auto reference_ptr = dual_group.getCurrentState(5.0);
    if (!reference_ptr)
      return false;
    moveit::core::RobotState reference(*reference_ptr);

    auto finish_successful_place = [&]() -> bool {
      if (!node_->get_parameter("execute").as_bool())
        return true;

      HeldObject& held = heldForArm(arm);
      held = HeldObject{};
      rclcpp::sleep_for(std::chrono::milliseconds(250));

      if (!restoreOneArmFromReference(
              dual_group, reference, arm == ArmSide::LEFT,
              "PLACE RETURN " + std::string(armSideName(arm)))) {
        RCLCPP_WARN(LOGGER, "PLACE succeeded but retreat/return failed");
        return false;
      }
      return true;
    };

    // Explicit yaw: obey the user's requested orientation exactly.
    if (std::isfinite(yaw)) {
      RCLCPP_INFO(LOGGER,
                  "PLACE requested yaw %.1f deg",
                  yaw * 180.0 / M_PI);

      bool execution_started = false;
      if (!tryPlaceAttachedObjectMTC(
              arm, object_name, x, y, z, yaw, "", execution_started)) {
        if (execution_started)
          RCLCPP_ERROR(LOGGER,
                       "PLACE failed during execution; inspect PlanningScene before retry");
        return false;
      }
      return finish_successful_place();
    }

    // AUTO yaw: preserve the object's current orientation first, then try
    // three quarter-turn variants. This avoids silently forcing yaw=0 and
    // avoids the old 0..350 degree brute-force scan.
    const ArmConfig cfg = armConfig(arm, object_name);
    const HeldObject& held = heldForArm(arm);

    Eigen::Isometry3d current_object_pose =
        reference.getGlobalLinkTransform(cfg.hand_frame) * held.hand_to_object;
    double base_yaw = std::atan2(current_object_pose.linear()(1, 0),
                                 current_object_pose.linear()(0, 0));

    const std::array<double, 4> offsets = {
        0.0, M_PI / 2.0, M_PI, 3.0 * M_PI / 2.0};

    for (double offset : offsets) {
      double candidate_yaw = base_yaw + offset;
      candidate_yaw = std::atan2(std::sin(candidate_yaw),
                                 std::cos(candidate_yaw));

      RCLCPP_INFO(LOGGER,
                  "PLACE AUTO trying yaw %.1f deg object=%s arm=%s target=(%.3f %.3f %.4f)",
                  candidate_yaw * 180.0 / M_PI,
                  object_name.c_str(), armSideName(arm), x, y, z);

      bool execution_started = false;
      if (tryPlaceAttachedObjectMTC(
              arm, object_name, x, y, z,
              candidate_yaw, "", execution_started)) {
        RCLCPP_INFO(LOGGER,
                    "PLACE AUTO SUCCESS yaw %.1f deg",
                    candidate_yaw * 180.0 / M_PI);
        return finish_successful_place();
      }

      // If execution itself started and failed, the physical/planning state is
      // no longer guaranteed to match the next candidate. Stop immediately.
      if (execution_started) {
        RCLCPP_ERROR(LOGGER,
                     "PLACE AUTO failed during execution; not trying another yaw");
        return false;
      }
    }

    RCLCPP_ERROR(LOGGER,
                 "PLACE AUTO failed: no feasible yaw among 4 quarter-turn candidates for %s",
                 object_name.c_str());
    return false;
  }

  bool moveObject(
      const std::string& object_name,
      double x,
      double y,
      double z,
      double yaw)
  {
    // Capture the object's world orientation BEFORE PICK. After attaching it to
    // a gripper, it is no longer a normal world CollisionObject, so querying
    // its world pose at that point may fail.
    double original_yaw = 0.0;
    Eigen::Isometry3d original_pose = Eigen::Isometry3d::Identity();
    const bool have_original_pose = getObjectWorldPose(object_name, original_pose);
    if (have_original_pose) {
      original_yaw = std::atan2(
          original_pose.linear()(1, 0),
          original_pose.linear()(0, 0));
    }

    if (!pickObject(object_name, ArmSide::AUTO))
      return false;

    if (!node_->get_parameter("execute").as_bool()) {
      RCLCPP_WARN(LOGGER,
                  "MOVE plan-only mode validates PICK only. Use execute:=true for full PICK+PLACE.");
      return true;
    }

    // Explicit yaw from the command: obey it exactly.
    if (std::isfinite(yaw)) {
      RCLCPP_INFO(LOGGER,
                  "MOVE requested yaw %.1f deg",
                  yaw * 180.0 / M_PI);
      return placeObject(object_name, x, y, z, yaw);
    }

    // No yaw was supplied. Do NOT silently force yaw=0.
    // Preserve the object's original yaw first, then try quarter-turn variants.
    // This is intentionally only 4 candidates, not a 0..350 degree brute-force scan.
    const std::array<double, 4> offsets = {
        0.0, M_PI / 2.0, M_PI, 3.0 * M_PI / 2.0};

    const double base_yaw = have_original_pose ? original_yaw : 0.0;

    for (double offset : offsets) {
      double candidate_yaw = base_yaw + offset;
      candidate_yaw = std::atan2(std::sin(candidate_yaw),
                                 std::cos(candidate_yaw));

      RCLCPP_INFO(LOGGER,
                  "MOVE AUTO trying yaw %.1f deg",
                  candidate_yaw * 180.0 / M_PI);

      if (placeObject(object_name, x, y, z, candidate_yaw)) {
        RCLCPP_INFO(LOGGER,
                    "MOVE AUTO SUCCESS yaw %.1f deg",
                    candidate_yaw * 180.0 / M_PI);
        return true;
      }
    }

    RCLCPP_ERROR(LOGGER,
                 "MOVE AUTO failed: no feasible yaw among 4 quarter-turn candidates for %s",
                 object_name.c_str());
    return false;
  }

  std::vector<double> stackYawCandidates() const
  {
    std::vector<double> out;
    auto add_unique = [&out](double yaw) {
      while (yaw < 0.0) yaw += 2.0 * M_PI;
      while (yaw >= 2.0 * M_PI) yaw -= 2.0 * M_PI;
      for (double v : out)
        if (std::abs(v - yaw) < 1e-6)
          return;
      out.push_back(yaw);
    };

    if (std::isfinite(stack_column_yaw_)) {
      for (int k = 0; k < 4; ++k)
        add_unique(stack_column_yaw_ + k * M_PI / 2.0);
    } else {
      add_unique(0.0);
      add_unique(M_PI / 2.0);
      add_unique(M_PI);
      add_unique(3.0 * M_PI / 2.0);
    }

    for (int i = 0; i < 36; ++i)
      add_unique(i * M_PI / 18.0);
    return out;
  }

  bool retreatReleasedArmUp(
      ArmSide arm,
      const std::string& phase_name)
  {
    const ArmConfig cfg = armConfig(arm, "");
    moveit::planning_interface::MoveGroupInterface arm_group(node_, cfg.arm_group);

    arm_group.setPlannerId("RRTConnectkConfigDefault");
    arm_group.setPlanningTime(3.0);
    arm_group.setNumPlanningAttempts(3);
    arm_group.setMaxVelocityScalingFactor(0.20);
    arm_group.setMaxAccelerationScalingFactor(0.20);

    const auto current_pose = arm_group.getCurrentPose(cfg.hand_frame);
    if (current_pose.header.frame_id.empty()) {
      RCLCPP_WARN(LOGGER, "[%s] cannot read current TCP pose", phase_name.c_str());
      return false;
    }

    // The old stack tried to return directly from the exact place/contact pose.
    // First create a small vertical clearance, then return to the saved lift state.
    // Try short distances only: 3 cm -> 2 cm -> 1 cm.
    const std::array<double, 3> retreat_distances = {0.03, 0.02, 0.01};

    for (double dz : retreat_distances) {
      auto target = current_pose;
      target.pose.position.z += dz;

      arm_group.setStartStateToCurrentState();
      arm_group.setPoseReferenceFrame(current_pose.header.frame_id);
      arm_group.setPoseTarget(target, cfg.hand_frame);

      RCLCPP_INFO(
          LOGGER,
          "[%s] trying released-arm vertical retreat dz=%.3f m",
          phase_name.c_str(), dz);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto result = arm_group.plan(plan);
      arm_group.clearPoseTargets();

      if (result != moveit::core::MoveItErrorCode::SUCCESS)
        continue;

      if (!node_->get_parameter("execute").as_bool()) {
        RCLCPP_INFO(LOGGER, "[%s] retreat PLAN SUCCESS", phase_name.c_str());
        return true;
      }

      const auto exec_result = arm_group.execute(plan);
      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(
            LOGGER,
            "[%s] retreat EXECUTION SUCCESS dz=%.3f m",
            phase_name.c_str(), dz);
        return true;
      }

      RCLCPP_WARN(
          LOGGER,
          "[%s] retreat execution failed at dz=%.3f m",
          phase_name.c_str(), dz);
      return false;
    }

    RCLCPP_WARN(
        LOGGER,
        "[%s] no short vertical retreat plan found",
        phase_name.c_str());
    return false;
  }

  bool placeStackLevel(
      const std::string& object_name,
      double cx,
      double cy,
      double z,
      double object_yaw,
      const std::string& support_object,
      int level)
  {
    const ArmSide arm = holdingArm(object_name);
    if (arm == ArmSide::NONE) {
      RCLCPP_ERROR(LOGGER, "STACK L%d: object %s is not held",
                   level + 1, object_name.c_str());
      return false;
    }

    // Same idea as MOVE: do not scan 0..350 deg and do not force a demo yaw.
    // Preserve the object's orientation from its source pose and let MoveIt/MTC
    // solve the robot joint configuration for the new XYZ target.
    moveit::planning_interface::MoveGroupInterface dual_group(node_, "dual_arms");
    auto reference_ptr = dual_group.getCurrentState(5.0);
    if (!reference_ptr)
      return false;
    moveit::core::RobotState reference(*reference_ptr);

    RCLCPP_INFO(
        LOGGER,
        "STACK L%d PLACE object=%s target=(%.3f %.3f %.4f) preserve_yaw=%.1f deg support=%s",
        level + 1, object_name.c_str(), cx, cy, z,
        object_yaw * 180.0 / M_PI,
        support_object.empty() ? "table" : support_object.c_str());

    bool execution_started = false;
    const bool ok = tryPlaceAttachedObjectMTC(
        arm, object_name, cx, cy, z, object_yaw, support_object, execution_started);

    if (!ok) {
      if (execution_started) {
        RCLCPP_ERROR(LOGGER,
                     "STACK L%d execution failed; object may still be attached",
                     level + 1);
      } else {
        RCLCPP_ERROR(LOGGER,
                     "STACK L%d: no plan for preserved object yaw %.1f deg",
                     level + 1, object_yaw * 180.0 / M_PI);
      }
      return false;
    }

    if (!node_->get_parameter("execute").as_bool())
      return true;

    heldForArm(arm) = HeldObject{};
    rclcpp::sleep_for(std::chrono::milliseconds(250));

    const std::string retreat_name =
        "STACK L" + std::to_string(level + 1) + " RETREAT";
    const bool retreat_ok = retreatReleasedArmUp(arm, retreat_name);
    if (!retreat_ok) {
      RCLCPP_WARN(
          LOGGER,
          "STACK L%d placed successfully, but short retreat was not available; trying return anyway",
          level + 1);
    }

    // Returning to the saved lift/reference pose is cleanup, not part of the
    // placement result.  Once the object has been opened + detached + placed,
    // a failed RETURN must not make STACK/MISSION report that the placement failed.
    if (!restoreOneArmFromReference(
            dual_group, reference, arm == ArmSide::LEFT,
            "STACK L" + std::to_string(level + 1) + " RETURN")) {
      RCLCPP_WARN(
          LOGGER,
          "STACK L%d PLACE SUCCESS; active arm RETURN failed, continuing stack",
          level + 1);
      return true;
    }

    return true;
  }

  bool placeAttachedObjectGenericStackMTC(
      bool place_left,
      const std::string& object_name,
      double cx,
      double cy,
      double z,
      const std::string& support_object,
      int level,
      double& selected_yaw)
  {
    const std::string arm_group =
        place_left ? "left_ur_onrobot_manipulator" : "right_ur_onrobot_manipulator";
    const std::string hand_group =
        place_left ? "left_ur_onrobot_gripper" : "right_ur_onrobot_gripper";
    const std::string hand_frame =
        place_left ? "left_gripper_tcp" : "right_gripper_tcp";

    // AUTO orientation: only try four square-equivalent directions.
    // No 0..350 degree brute-force scan.
    std::array<double, 4> yaw_candidates{};
    const double base_yaw = std::isfinite(stack_column_yaw_) ? stack_column_yaw_ : 0.0;
    for (std::size_t i = 0; i < yaw_candidates.size(); ++i)
      yaw_candidates[i] = base_yaw + static_cast<double>(i) * M_PI / 2.0;

    for (double yaw : yaw_candidates) {
      yaw = std::atan2(std::sin(yaw), std::cos(yaw));
      const double deg = yaw * 180.0 / M_PI;

      RCLCPP_INFO(
          LOGGER,
          "DUAL STACK L%d %s PLACE object=%s target=(%.3f %.3f %.4f) yaw=%.0f support=%s",
          level + 1,
          place_left ? "LEFT" : "RIGHT",
          object_name.c_str(), cx, cy, z, deg,
          support_object.empty() ? "table" : support_object.c_str());

      mtc::Task t;
      t.stages()->setName("dual_stack_place_" + object_name);
      t.loadRobotModel(node_);
      t.setProperty("group", arm_group);
      t.setProperty("eef", hand_group);
      t.setProperty("ik_frame", hand_frame);

      mtc::Stage* current_state_ptr = nullptr;
      auto current = std::make_unique<mtc::stages::CurrentState>("current attached state");
      current_state_ptr = current.get();
      t.add(std::move(current));

      mtc::Stage* place_monitor_ptr = current_state_ptr;
      if (!support_object.empty()) {
        const auto* hand_jmg = t.getRobotModel()->getJointModelGroup(hand_group);
        if (!hand_jmg) {
          RCLCPP_ERROR(LOGGER, "DUAL STACK PLACE: missing hand group %s", hand_group.c_str());
          return false;
        }

        const auto hand_collision_links =
            hand_jmg->getLinkModelNamesWithCollisionGeometry();

        auto allow_support_touch =
            std::make_unique<mtc::stages::ModifyPlanningScene>(
                "allow active gripper touch support");
        allow_support_touch->allowCollisions(
            support_object, hand_collision_links, true);
        place_monitor_ptr = allow_support_touch.get();
        t.add(std::move(allow_support_touch));
      }

      auto sampling = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
      sampling->setProperty("planning_pipeline", "ompl");
      sampling->setPlannerId("RRTConnectkConfigDefault");

      auto interpolation = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

      {
        auto connect = std::make_unique<mtc::stages::Connect>(
            "move active arm to stack target",
            mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling}});
        connect->setTimeout(5.0);
        connect->properties().configureInitFrom(mtc::Stage::PARENT);
        t.add(std::move(connect));
      }

      {
        auto place = std::make_unique<mtc::SerialContainer>(
            "place one object while other arm waits");
        t.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
        place->properties().configureInitFrom(
            mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        auto gen = std::make_unique<mtc::stages::GeneratePlacePose>(
            "generate stack target pose");
        gen->properties().configureInitFrom(mtc::Stage::PARENT);
        gen->properties().set("marker_ns", "dual_stack_target");
        gen->setObject(object_name);

        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "world";
        target.pose.position.x = cx;
        target.pose.position.y = cy;
        target.pose.position.z = z;
        Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        target.pose.orientation.x = q.x();
        target.pose.orientation.y = q.y();
        target.pose.orientation.z = q.z();
        target.pose.orientation.w = q.w();
        gen->setPose(target);
        gen->setMonitoredStage(place_monitor_ptr);

        auto ik = std::make_unique<mtc::stages::ComputeIK>(
            "compute stack place IK", std::move(gen));
        ik->setMaxIKSolutions(16);
        ik->setMinSolutionDistance(0.2);
        ik->setIKFrame(object_name);
        ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        place->insert(std::move(ik));

        auto open = std::make_unique<mtc::stages::MoveTo>(
            "open active gripper", interpolation);
        open->setGroup(hand_group);
        open->setGoal("open");
        place->insert(std::move(open));

        auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "detach stacked object");
        detach->detachObject(object_name, hand_frame);
        place->insert(std::move(detach));

        if (!support_object.empty()) {
          const auto* hand_jmg = t.getRobotModel()->getJointModelGroup(hand_group);
          const auto hand_collision_links =
              hand_jmg->getLinkModelNamesWithCollisionGeometry();
          auto restore_support =
              std::make_unique<mtc::stages::ModifyPlanningScene>(
                  "restore support collision checking");
          restore_support->allowCollisions(
              support_object, hand_collision_links, false);
          place->insert(std::move(restore_support));
        }

        t.add(std::move(place));
      }

      try {
        t.init();
      } catch (const mtc::InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, "DUAL STACK PLACE init failed:\n" << e);
        continue;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "DUAL STACK PLACE init exception: %s", e.what());
        continue;
      }

      if (!t.plan(5) || t.solutions().empty()) {
        RCLCPP_WARN(
            LOGGER,
            "DUAL STACK L%d %s: no place solution at yaw %.0f",
            level + 1, place_left ? "LEFT" : "RIGHT", deg);
        continue;
      }

      t.introspection().publishSolution(*t.solutions().front());
      const auto result = t.execute(*t.solutions().front());
      if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR(
            LOGGER,
            "DUAL STACK L%d %s PLACE execution failed",
            level + 1, place_left ? "LEFT" : "RIGHT");
        return false;
      }

      selected_yaw = yaw;
      RCLCPP_INFO(
          LOGGER,
          "DUAL STACK L%d %s PLACE SUCCESS object=%s yaw=%.0f",
          level + 1, place_left ? "LEFT" : "RIGHT",
          object_name.c_str(), deg);
      return true;
    }

    RCLCPP_ERROR(
        LOGGER,
        "DUAL STACK L%d %s PLACE FAILED for all 4 AUTO yaw candidates",
        level + 1, place_left ? "LEFT" : "RIGHT");
    return false;
  }

  bool placeOneGenericStackLevel(
      moveit::planning_interface::MoveGroupInterface& dual_group,
      const moveit::core::RobotState& lift_reference,
      bool place_left,
      const std::string& object_name,
      double cx,
      double cy,
      double z,
      const std::string& support_object,
      int level)
  {
    double selected_yaw = 0.0;
    if (!placeAttachedObjectGenericStackMTC(
            place_left, object_name, cx, cy, z,
            support_object, level, selected_yaw))
      return false;

    if (!std::isfinite(stack_column_yaw_)) {
      stack_column_yaw_ = selected_yaw;
      RCLCPP_INFO(
          LOGGER,
          "DUAL STACK column yaw locked at %.0f deg",
          stack_column_yaw_ * 180.0 / M_PI);
    }

    // After PLACE, return this arm to the exact dual-LIFT state captured while
    // both objects were already in the air. The other arm keeps waiting there.
    return restoreOneArmFromReference(
        dual_group, lift_reference, place_left,
        "DUAL STACK L" + std::to_string(level + 1) + " RETURN TO WAIT POSE");
  }

  bool runDualStackPair(
      moveit::planning_interface::MoveGroupInterface& dual_group,
      const std::string& lower_object,
      const std::string& upper_object,
      int lower_level,
      int upper_level,
      double center_x,
      double center_y,
      const std::string& support_below_lower,
      int round_index)
  {
    const auto model = dual_group.getRobotModel();
    const double size = node_->get_parameter("stack_object_size").as_double();
    const double gap = node_->get_parameter("stack_gap").as_double();
    constexpr double BASE_CLEARANCE = 0.0005;

    const double lower_z =
        stack_table_top_ + 0.5 * size +
        static_cast<double>(lower_level) * (size + gap) + BASE_CLEARANCE;
    const double upper_z =
        stack_table_top_ + 0.5 * size +
        static_cast<double>(upper_level) * (size + gap) + BASE_CLEARANCE;

    struct PairCandidate
    {
      bool valid = false;
      std::string left_object;
      std::string right_object;
      int left_level = -1;
      int right_level = -1;
      DualTaskTargets targets;
      std::vector<std::string> left_hand_links;
      std::vector<std::string> right_hand_links;
    };

    auto evaluate_assignment = [&](const std::string& left_object,
                                   const std::string& right_object,
                                   int left_level,
                                   int right_level) {
      PairCandidate c;
      c.left_object = left_object;
      c.right_object = right_object;
      c.left_level = left_level;
      c.right_level = right_level;
      try {
        c.targets = preparePickTargetsFastForObjects(
            model, left_object, right_object,
            c.left_hand_links, c.right_hand_links);
        c.valid = true;
        RCLCPP_INFO(
            LOGGER,
            "DUAL STACK R%d assignment feasible: LEFT=%s(L%d) RIGHT=%s(L%d) pick_score=%.3f",
            round_index,
            left_object.c_str(), left_level + 1,
            right_object.c_str(), right_level + 1,
            c.targets.score);
      } catch (const std::exception& e) {
        RCLCPP_WARN(
            LOGGER,
            "DUAL STACK R%d assignment rejected before motion: LEFT=%s RIGHT=%s reason=%s",
            round_index, left_object.c_str(), right_object.c_str(), e.what());
      }
      return c;
    };

    // The names do NOT define the level or arm. Command order defines stack
    // order. Evaluate both arm assignments before either robot moves.
    PairCandidate a = evaluate_assignment(
        lower_object, upper_object, lower_level, upper_level);
    PairCandidate b = evaluate_assignment(
        upper_object, lower_object, upper_level, lower_level);

    PairCandidate* chosen = nullptr;
    if (a.valid && b.valid)
      chosen = (a.targets.score <= b.targets.score) ? &a : &b;
    else if (a.valid)
      chosen = &a;
    else if (b.valid)
      chosen = &b;

    if (!chosen) {
      RCLCPP_ERROR(
          LOGGER,
          "DUAL STACK R%d: neither LEFT/RIGHT assignment can dual-pick %s + %s",
          round_index, lower_object.c_str(), upper_object.c_str());
      return false;
    }

    const std::string left_object = chosen->left_object;
    const std::string right_object = chosen->right_object;
    const int left_level = chosen->left_level;
    const int right_level = chosen->right_level;
    const auto& targets = chosen->targets;

    auto home = dual_group.getCurrentState(5.0);
    if (!home)
      return false;

    moveit::core::RobotState pre(*home), grasp(*home), lift(*home);
    applyGoalToState(pre, targets.pregrasp);
    applyGoalToState(grasp, targets.grasp);
    applyGoalToState(lift, targets.lift);

    RCLCPP_INFO(LOGGER, "====================================================");
    RCLCPP_INFO(
        LOGGER,
        "DUAL STACK ROUND %d: simultaneous PICK LEFT=%s(L%d) RIGHT=%s(L%d)",
        round_index,
        left_object.c_str(), left_level + 1,
        right_object.c_str(), right_level + 1);
    RCLCPP_INFO(
        LOGGER,
        "After LIFT both arms WAIT; lower level is placed first, then upper level");
    RCLCPP_INFO(LOGGER, "====================================================");

    if (!executeDualGrippers(
            0.100, "DUAL STACK R" + std::to_string(round_index) + " OPEN BOTH"))
      return false;

    if (!planExecuteDualState(
            dual_group, *home, pre, HoldMode::NONE,
            "DUAL STACK R" + std::to_string(round_index) + " PARALLEL PREGRASP"))
      return false;

    auto s = dual_group.getCurrentState(5.0);
    if (!s)
      return false;
    if (!planExecuteDualState(
            dual_group, *s, grasp, HoldMode::NONE,
            "DUAL STACK R" + std::to_string(round_index) + " PARALLEL GRASP"))
      return false;

    auto attach_state = dual_group.getCurrentState(5.0);
    if (!attach_state)
      return false;
    attach_state->update();

    moveit::planning_interface::PlanningSceneInterface psi;
    const auto object_poses = psi.getObjectPoses({left_object, right_object});
    if (!object_poses.count(left_object) || !object_poses.count(right_object)) {
      RCLCPP_ERROR(LOGGER, "DUAL STACK R%d: cannot read both object poses", round_index);
      return false;
    }

    const bool la = dual_group.attachObject(
        left_object, "left_gripper_tcp", chosen->left_hand_links);
    const bool ra = dual_group.attachObject(
        right_object, "right_gripper_tcp", chosen->right_hand_links);
    if (!la || !ra) {
      RCLCPP_ERROR(
          LOGGER,
          "DUAL STACK R%d attach failed: LEFT=%s RIGHT=%s",
          round_index, la ? "OK" : "FAIL", ra ? "OK" : "FAIL");
      if (la) dual_group.detachObject(left_object);
      if (ra) dual_group.detachObject(right_object);
      return false;
    }

    if (!executeDualGrippers(
            0.0, "DUAL STACK R" + std::to_string(round_index) + " CLOSE BOTH")) {
      dual_group.detachObject(left_object);
      dual_group.detachObject(right_object);
      return false;
    }

    s = dual_group.getCurrentState(5.0);
    if (!s)
      return false;
    if (!planExecuteDualState(
            dual_group, *s, lift, HoldMode::NONE,
            "DUAL STACK R" + std::to_string(round_index) + " PARALLEL LIFT BOTH"))
      return false;

    auto lift_ref_ptr = dual_group.getCurrentState(5.0);
    if (!lift_ref_ptr)
      return false;
    moveit::core::RobotState lift_ref(*lift_ref_ptr);

    RCLCPP_INFO(
        LOGGER,
        "DUAL STACK R%d: BOTH OBJECTS ARE UP -> both arms now wait at lift pose",
        round_index);

    auto place_by_level = [&](bool place_left,
                              const std::string& object_name,
                              int level) {
      const bool is_lower = (level == lower_level);
      const double z = is_lower ? lower_z : upper_z;
      const std::string support = is_lower ? support_below_lower : lower_object;

      RCLCPP_INFO(
          LOGGER,
          "DUAL STACK R%d: placing %s with %s at L%d; the other arm WAITS",
          round_index,
          object_name.c_str(), place_left ? "LEFT" : "RIGHT", level + 1);

      return placeOneGenericStackLevel(
          dual_group, lift_ref, place_left, object_name,
          center_x, center_y, z, support, level);
    };

    // Command order is stack order. Whichever arm holds the lower object places
    // first. The other arm remains at the saved lift/wait pose holding its object.
    if (left_level < right_level) {
      if (!place_by_level(true, left_object, left_level))
        return false;
      if (!place_by_level(false, right_object, right_level))
        return false;
    } else {
      if (!place_by_level(false, right_object, right_level))
        return false;
      if (!place_by_level(true, left_object, left_level))
        return false;
    }

    auto now = dual_group.getCurrentState(5.0);
    if (!now)
      return false;
    if (!planExecuteDualState(
            dual_group, *now, *home, HoldMode::NONE,
            "DUAL STACK R" + std::to_string(round_index) + " PARALLEL RETURN"))
      return false;

    RCLCPP_INFO(LOGGER, "DUAL STACK ROUND %d SUCCESS", round_index);
    return true;
  }

  bool stackObjects(
      const std::vector<std::string>& objects,
      double center_x,
      double center_y)
  {
    if (!node_->get_parameter("execute").as_bool()) {
      RCLCPP_ERROR(LOGGER, "STACK command requires execute:=true");
      return false;
    }
    if (left_held_.valid || right_held_.valid) {
      RCLCPP_ERROR(LOGGER, "STACK requires both arms free at start");
      return false;
    }
    if (objects.empty()) {
      RCLCPP_ERROR(LOGGER, "STACK requires at least one object");
      return false;
    }

    stack_column_yaw_ = std::numeric_limits<double>::quiet_NaN();

    moveit::planning_interface::MoveGroupInterface dual_group(node_, "dual_arms");

    std::size_t level = 0;
    int round_index = 1;

    // Main behavior: two arms pick two consecutive command objects together,
    // lift together, wait together, then place them one at a time in command order.
    for (; level + 1 < objects.size(); level += 2, ++round_index) {
      const std::string support_below_lower =
          (level == 0) ? std::string{} : objects[level - 1];

      if (!runDualStackPair(
              dual_group,
              objects[level],       // lower object by COMMAND ORDER
              objects[level + 1],   // next object above it
              static_cast<int>(level),
              static_cast<int>(level + 1),
              center_x,
              center_y,
              support_below_lower,
              round_index))
        return false;
    }

    // Odd number of objects: only the final object has no partner, so handle
    // that one with the existing single-object PICK -> PLACE path.
    if (level < objects.size()) {
      const std::string& object_name = objects[level];
      const double size = node_->get_parameter("stack_object_size").as_double();
      const double gap = node_->get_parameter("stack_gap").as_double();
      constexpr double BASE_CLEARANCE = 0.0005;
      const double z =
          stack_table_top_ + 0.5 * size +
          static_cast<double>(level) * (size + gap) + BASE_CLEARANCE;

      Eigen::Isometry3d source_pose = Eigen::Isometry3d::Identity();
      if (!getObjectWorldPose(object_name, source_pose)) {
        RCLCPP_ERROR(LOGGER, "STACK: cannot read final object pose for %s", object_name.c_str());
        return false;
      }
      const double source_yaw = std::atan2(
          source_pose.linear()(1, 0), source_pose.linear()(0, 0));

      RCLCPP_INFO(
          LOGGER,
          "DUAL STACK: odd final object %s -> single PICK/PLACE at L%zu",
          object_name.c_str(), level + 1);

      if (!pickObject(object_name, ArmSide::AUTO))
        return false;

      const std::string support_object =
          (level == 0) ? std::string{} : objects[level - 1];
      if (!placeStackLevel(
              object_name, center_x, center_y, z, source_yaw,
              support_object, static_cast<int>(level)))
        return false;
    }

    RCLCPP_INFO(
        LOGGER,
        "DUAL STACK SUCCESS: %zu objects stacked at (%.3f, %.3f)",
        objects.size(), center_x, center_y);
    return true;
  }

  bool placeOneSwapTarget(
      moveit::planning_interface::MoveGroupInterface& dual_group,
      const moveit::core::RobotState& lift_reference,
      bool place_left,
      const std::string& object_name,
      const Eigen::Isometry3d& target_pose,
      double preferred_yaw,
      const std::string& phase_name)
  {
    const ArmSide arm = place_left ? ArmSide::LEFT : ArmSide::RIGHT;
    const std::array<double, 4> offsets = {
        0.0, M_PI / 2.0, M_PI, 3.0 * M_PI / 2.0};

    for (double offset : offsets) {
      double yaw = preferred_yaw + offset;
      yaw = std::atan2(std::sin(yaw), std::cos(yaw));

      RCLCPP_INFO(
          LOGGER,
          "%s: %s placing %s -> (%.3f %.3f %.4f), try yaw=%.0f deg; other arm WAITS",
          phase_name.c_str(), place_left ? "LEFT" : "RIGHT",
          object_name.c_str(),
          target_pose.translation().x(),
          target_pose.translation().y(),
          target_pose.translation().z(),
          yaw * 180.0 / M_PI);

      bool execution_started = false;
      if (!tryPlaceAttachedObjectMTC(
              arm,
              object_name,
              target_pose.translation().x(),
              target_pose.translation().y(),
              target_pose.translation().z(),
              yaw,
              "",
              execution_started)) {
        // If execution started, the current state is no longer guaranteed to
        // match the next yaw candidate. Stop immediately.
        if (execution_started) {
          RCLCPP_ERROR(LOGGER, "%s: execution failed", phase_name.c_str());
          return false;
        }
        continue;
      }

      // MTC opened the active gripper and detached this object.
      heldForArm(arm) = HeldObject{};
      rclcpp::sleep_for(std::chrono::milliseconds(200));

      // Create a small clearance before returning to the exact two-arm LIFT
      // state. The opposite arm remains fixed and continues holding its object.
      const bool retreat_ok = retreatReleasedArmUp(
          arm, phase_name + " RETREAT");
      if (!retreat_ok) {
        RCLCPP_WARN(
            LOGGER,
            "%s: object is placed, but short retreat was unavailable; trying WAIT return",
            phase_name.c_str());
      }

      if (!restoreOneArmFromReference(
              dual_group,
              lift_reference,
              place_left,
              phase_name + " RETURN TO WAIT")) {
        RCLCPP_WARN(
            LOGGER,
            "%s: object placed successfully but return-to-wait failed",
            phase_name.c_str());
        // Placement is already complete. Do not undo a successful swap half.
      }

      RCLCPP_INFO(
          LOGGER,
          "%s SUCCESS object=%s",
          phase_name.c_str(), object_name.c_str());
      return true;
    }

    RCLCPP_ERROR(
        LOGGER,
        "%s FAILED: no feasible yaw among 4 quarter-turn candidates for %s",
        phase_name.c_str(), object_name.c_str());
    return false;
  }

  bool swapObjects(const std::string& object_a, const std::string& object_b)
  {
    if (!node_->get_parameter("execute").as_bool()) {
      RCLCPP_ERROR(LOGGER, "SWAP command requires execute:=true");
      return false;
    }
    if (left_held_.valid || right_held_.valid) {
      RCLCPP_ERROR(LOGGER, "SWAP requires both arms free at start");
      return false;
    }
    if (object_a == object_b) {
      RCLCPP_ERROR(LOGGER, "SWAP requires two different objects");
      return false;
    }

    Eigen::Isometry3d pose_a = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d pose_b = Eigen::Isometry3d::Identity();
    if (!getObjectWorldPose(object_a, pose_a) ||
        !getObjectWorldPose(object_b, pose_b)) {
      RCLCPP_ERROR(LOGGER, "SWAP: cannot read both source object poses");
      return false;
    }

    const double yaw_a = std::atan2(
        pose_a.linear()(1, 0), pose_a.linear()(0, 0));
    const double yaw_b = std::atan2(
        pose_b.linear()(1, 0), pose_b.linear()(0, 0));

    moveit::planning_interface::MoveGroupInterface dual_group(node_, "dual_arms");
    const auto model = dual_group.getRobotModel();

    struct SwapCandidate
    {
      bool valid = false;
      std::string left_object;
      std::string right_object;
      DualTaskTargets targets;
      std::vector<std::string> left_hand_links;
      std::vector<std::string> right_hand_links;
    };

    auto evaluate_assignment = [&](const std::string& left_object,
                                   const std::string& right_object) {
      SwapCandidate c;
      c.left_object = left_object;
      c.right_object = right_object;
      try {
        c.targets = preparePickTargetsFastForObjects(
            model,
            left_object,
            right_object,
            c.left_hand_links,
            c.right_hand_links);
        c.valid = true;
        RCLCPP_INFO(
            LOGGER,
            "DUAL SWAP assignment feasible: LEFT=%s RIGHT=%s pick_score=%.3f",
            left_object.c_str(), right_object.c_str(), c.targets.score);
      } catch (const std::exception& e) {
        RCLCPP_WARN(
            LOGGER,
            "DUAL SWAP assignment rejected: LEFT=%s RIGHT=%s reason=%s",
            left_object.c_str(), right_object.c_str(), e.what());
      }
      return c;
    };

    SwapCandidate first = evaluate_assignment(object_a, object_b);
    SwapCandidate second = evaluate_assignment(object_b, object_a);

    SwapCandidate* chosen = nullptr;
    if (first.valid && second.valid)
      chosen = (first.targets.score <= second.targets.score) ? &first : &second;
    else if (first.valid)
      chosen = &first;
    else if (second.valid)
      chosen = &second;

    if (!chosen) {
      RCLCPP_ERROR(
          LOGGER,
          "DUAL SWAP: neither LEFT/RIGHT assignment can pick %s + %s together",
          object_a.c_str(), object_b.c_str());
      return false;
    }

    const std::string left_object = chosen->left_object;
    const std::string right_object = chosen->right_object;
    const auto& targets = chosen->targets;

    auto home = dual_group.getCurrentState(5.0);
    if (!home) {
      RCLCPP_ERROR(LOGGER, "DUAL SWAP: cannot read current robot state");
      return false;
    }

    moveit::core::RobotState pre(*home), grasp(*home), lift(*home);
    applyGoalToState(pre, targets.pregrasp);
    applyGoalToState(grasp, targets.grasp);
    applyGoalToState(lift, targets.lift);

    RCLCPP_INFO(LOGGER, "====================================================");
    RCLCPP_INFO(
        LOGGER,
        "DUAL SWAP START: %s <-> %s | LEFT=%s RIGHT=%s",
        object_a.c_str(), object_b.c_str(),
        left_object.c_str(), right_object.c_str());
    RCLCPP_INFO(
        LOGGER,
        "Both arms PICK together -> LIFT together -> one PLACE while the other WAITS -> second PLACE");
    RCLCPP_INFO(LOGGER, "====================================================");

    // 1) Both grippers open, both arms approach/grasp at the same time.
    if (!executeDualGrippers(0.100, "DUAL SWAP OPEN BOTH"))
      return false;

    if (!planExecuteDualState(
            dual_group, *home, pre, HoldMode::NONE,
            "DUAL SWAP PARALLEL PREGRASP"))
      return false;

    auto current = dual_group.getCurrentState(5.0);
    if (!current)
      return false;
    if (!planExecuteDualState(
            dual_group, *current, grasp, HoldMode::NONE,
            "DUAL SWAP PARALLEL GRASP"))
      return false;

    // 2) Record the two hand->object transforms, then attach both objects.
    auto attach_state = dual_group.getCurrentState(5.0);
    if (!attach_state)
      return false;
    attach_state->update();

    const Eigen::Isometry3d left_world_object =
        (left_object == object_a) ? pose_a : pose_b;
    const Eigen::Isometry3d right_world_object =
        (right_object == object_a) ? pose_a : pose_b;

    const Eigen::Isometry3d left_hand_to_object =
        attach_state->getGlobalLinkTransform("left_gripper_tcp").inverse() *
        left_world_object;
    const Eigen::Isometry3d right_hand_to_object =
        attach_state->getGlobalLinkTransform("right_gripper_tcp").inverse() *
        right_world_object;

    const bool left_attached = dual_group.attachObject(
        left_object, "left_gripper_tcp", chosen->left_hand_links);
    const bool right_attached = dual_group.attachObject(
        right_object, "right_gripper_tcp", chosen->right_hand_links);

    if (!left_attached || !right_attached) {
      RCLCPP_ERROR(
          LOGGER,
          "DUAL SWAP attach failed: LEFT=%s RIGHT=%s",
          left_attached ? "OK" : "FAIL",
          right_attached ? "OK" : "FAIL");
      if (left_attached) dual_group.detachObject(left_object);
      if (right_attached) dual_group.detachObject(right_object);
      return false;
    }

    if (!executeDualGrippers(0.0, "DUAL SWAP CLOSE BOTH")) {
      dual_group.detachObject(left_object);
      dual_group.detachObject(right_object);
      return false;
    }

    left_held_.valid = true;
    left_held_.object_name = left_object;
    left_held_.arm = ArmSide::LEFT;
    left_held_.hand_to_object = left_hand_to_object;

    right_held_.valid = true;
    right_held_.object_name = right_object;
    right_held_.arm = ArmSide::RIGHT;
    right_held_.hand_to_object = right_hand_to_object;

    // 3) Both arms lift at the same time and save this as the WAIT pose.
    current = dual_group.getCurrentState(5.0);
    if (!current)
      return false;
    if (!planExecuteDualState(
            dual_group, *current, lift, HoldMode::NONE,
            "DUAL SWAP PARALLEL LIFT BOTH"))
      return false;

    auto lift_ref_ptr = dual_group.getCurrentState(5.0);
    if (!lift_ref_ptr)
      return false;
    moveit::core::RobotState lift_reference(*lift_ref_ptr);

    RCLCPP_INFO(
        LOGGER,
        "DUAL SWAP: BOTH OBJECTS ARE UP -> both arms waiting at LIFT pose");

    // 4) Swap positions. The object A goes to B's original XYZ and object B
    // goes to A's original XYZ. Each object keeps its own orientation first;
    // quarter-turn alternatives are allowed only when necessary for IK.
    auto place_object_to_target = [&](const std::string& object_name,
                                      const Eigen::Isometry3d& target_pose,
                                      double preferred_yaw,
                                      const std::string& phase) {
      const bool place_left = (object_name == left_object);
      return placeOneSwapTarget(
          dual_group,
          lift_reference,
          place_left,
          object_name,
          target_pose,
          preferred_yaw,
          phase);
    };

    // Place A first while the arm holding B waits at the common lift pose.
    if (!place_object_to_target(
            object_a, pose_b, yaw_a, "DUAL SWAP PLACE A->B")) {
      RCLCPP_ERROR(LOGGER, "DUAL SWAP failed while placing %s", object_a.c_str());
      return false;
    }

    // Then place B into A's old position.
    if (!place_object_to_target(
            object_b, pose_a, yaw_b, "DUAL SWAP PLACE B->A")) {
      RCLCPP_ERROR(LOGGER, "DUAL SWAP failed while placing %s", object_b.c_str());
      return false;
    }

    // 5) Both objects are released. Return both arms together to the state from
    // which the SWAP command started.
    auto now = dual_group.getCurrentState(5.0);
    if (!now)
      return false;
    if (!planExecuteDualState(
            dual_group, *now, *home, HoldMode::NONE,
            "DUAL SWAP PARALLEL RETURN")) {
      RCLCPP_WARN(
          LOGGER,
          "DUAL SWAP objects exchanged successfully, but final arm return failed");
      return true;
    }

    RCLCPP_INFO(
        LOGGER,
        "DUAL SWAP SUCCESS: %s <-> %s",
        object_a.c_str(), object_b.c_str());
    return true;
  }

  enum class HoldMode
  {
    NONE,
    HOLD_LEFT,
    HOLD_RIGHT
  };


  bool buildDualPoseTargetIKOnly(
      const moveit::core::RobotState& current,
      const Eigen::Isometry3d* left_pose,
      const Eigen::Isometry3d* right_pose,
      moveit::core::RobotState& target)
  {
    const auto model = current.getRobotModel();
    const auto* left_jmg = model->getJointModelGroup("left_ur_onrobot_manipulator");
    const auto* right_jmg = model->getJointModelGroup("right_ur_onrobot_manipulator");
    const auto* dual_jmg = model->getJointModelGroup("dual_arms");
    if (!left_jmg || !right_jmg || !dual_jmg)
      return false;

    target = current;

    auto solve_one = [&](const moveit::core::JointModelGroup* jmg,
                         const Eigen::Isometry3d& pose,
                         const std::string& tip) {
      for (int attempt = 0; attempt < 12; ++attempt) {
        moveit::core::RobotState trial(target);
        if (attempt > 0)
          trial.setToRandomPositions(jmg);

        if (!trial.setFromIK(jmg, pose, tip, 0.15))
          continue;

        trial.update();
        if (!trial.satisfiesBounds(jmg))
          continue;

        std::vector<double> q;
        trial.copyJointGroupPositions(jmg, q);
        normalizeGroupNearState(model, jmg, current, q);
        target.setJointGroupPositions(jmg, q);
        target.update();
        return true;
      }
      return false;
    };

    if (left_pose && !solve_one(left_jmg, *left_pose, "left_gripper_tcp"))
      return false;
    if (right_pose && !solve_one(right_jmg, *right_pose, "right_gripper_tcp"))
      return false;

    return target.satisfiesBounds(dual_jmg);
  }

  moveit_msgs::msg::Constraints buildHoldConstraints(
      const moveit::core::RobotState& state,
      const moveit::core::JointModelGroup* hold_jmg)
  {
    moveit_msgs::msg::Constraints constraints;
    std::vector<double> q;
    state.copyJointGroupPositions(hold_jmg, q);
    const auto& names = hold_jmg->getVariableNames();

    for (std::size_t i = 0; i < names.size(); ++i) {
      moveit_msgs::msg::JointConstraint jc;
      jc.joint_name = names[i];
      jc.position = q[i];
      jc.tolerance_above = 0.005;
      jc.tolerance_below = 0.005;
      jc.weight = 1.0;
      constraints.joint_constraints.push_back(jc);
    }
    return constraints;
  }

  bool buildDualPoseTarget(
      const planning_scene::PlanningScenePtr& scene,
      const moveit::core::RobotState& current,
      const Eigen::Isometry3d* left_pose,
      const Eigen::Isometry3d* right_pose,
      moveit::core::RobotState& target)
  {
    const auto model = scene->getRobotModel();
    const auto* left_jmg = model->getJointModelGroup("left_ur_onrobot_manipulator");
    const auto* right_jmg = model->getJointModelGroup("right_ur_onrobot_manipulator");
    const auto* dual_jmg = model->getJointModelGroup("dual_arms");
    if (!left_jmg || !right_jmg || !dual_jmg)
      return false;

    const ArmConfig left{"left_ur_onrobot_manipulator", "left_ur_onrobot_gripper", "left_gripper_tcp", "left_object"};
    const ArmConfig right{"right_ur_onrobot_manipulator", "right_ur_onrobot_gripper", "right_gripper_tcp", "right_object"};

    target = current;

    if (left_pose) {
      moveit::core::RobotState left_solution(model);
      if (!solveIKCollisionFree(scene, left, *left_pose, target, left_solution, 25))
        return false;
      std::vector<double> q;
      left_solution.copyJointGroupPositions(left_jmg, q);
      normalizeGroupNearState(model, left_jmg, current, q);
      target.setJointGroupPositions(left_jmg, q);
      target.update();
    }

    if (right_pose) {
      moveit::core::RobotState right_solution(model);
      if (!solveIKCollisionFree(scene, right, *right_pose, target, right_solution, 25))
        return false;
      std::vector<double> q;
      right_solution.copyJointGroupPositions(right_jmg, q);
      normalizeGroupNearState(model, right_jmg, current, q);
      target.setJointGroupPositions(right_jmg, q);
      target.update();
    }

    if (!target.satisfiesBounds(dual_jmg))
      return false;
    return !scene->isStateColliding(target, "", false);
  }

  bool planExecuteDualState(
      moveit::planning_interface::MoveGroupInterface& dual_group,
      const moveit::core::RobotState& start,
      const moveit::core::RobotState& target,
      HoldMode hold_mode,
      const std::string& phase_name)
  {
    const auto model = dual_group.getRobotModel();
    const auto* dual_jmg = model->getJointModelGroup("dual_arms");
    const auto* left_jmg = model->getJointModelGroup("left_ur_onrobot_manipulator");
    const auto* right_jmg = model->getJointModelGroup("right_ur_onrobot_manipulator");
    if (!dual_jmg || !left_jmg || !right_jmg)
      return false;

    std::vector<double> q;
    target.copyJointGroupPositions(dual_jmg, q);

    dual_group.setPlannerId("RRTConnectkConfigDefault");
    dual_group.setPlanningTime(10.0);
    dual_group.setNumPlanningAttempts(5);
    dual_group.setMaxVelocityScalingFactor(0.30);
    dual_group.setMaxAccelerationScalingFactor(0.25);
    dual_group.setStartState(start);
    dual_group.setJointValueTarget(q);

    if (hold_mode == HoldMode::HOLD_LEFT)
      dual_group.setPathConstraints(buildHoldConstraints(start, left_jmg));
    else if (hold_mode == HoldMode::HOLD_RIGHT)
      dual_group.setPathConstraints(buildHoldConstraints(start, right_jmg));
    else
      dual_group.clearPathConstraints();

    RCLCPP_INFO(LOGGER, "[%s] planning dual_arms 12DOF...", phase_name.c_str());
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto result = dual_group.plan(plan);
    dual_group.clearPathConstraints();

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "[%s] PLAN FAILED", phase_name.c_str());
      return false;
    }

    RCLCPP_INFO(LOGGER, "[%s] PLAN SUCCESS: joints=%zu points=%zu", phase_name.c_str(),
                plan.trajectory_.joint_trajectory.joint_names.size(),
                plan.trajectory_.joint_trajectory.points.size());

    const bool execute = node_->get_parameter("execute").as_bool();
    if (!execute) {
      RCLCPP_INFO(LOGGER, "[%s] plan-only; execute:=true de robot chay.", phase_name.c_str());
      return true;
    }

    const auto exec_result = dual_group.execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "[%s] EXECUTION FAILED", phase_name.c_str());
      return false;
    }
    RCLCPP_INFO(LOGGER, "[%s] EXECUTION SUCCESS", phase_name.c_str());
    return true;
  }

  bool parallelMove(
      moveit::planning_interface::MoveGroupInterface& dual_group,
      const planning_scene::PlanningScenePtr& scene,
      const Eigen::Isometry3d& left_target,
      const Eigen::Isometry3d& right_target,
      const std::string& name)
  {
    auto current = dual_group.getCurrentState(5.0);
    if (!current)
      return false;
    scene->setCurrentState(*current);
    moveit::core::RobotState target(*current);
    if (!buildDualPoseTarget(scene, *current, &left_target, &right_target, target)) {
      RCLCPP_ERROR(LOGGER, "[%s] Khong tao duoc dual target collision-free", name.c_str());
      return false;
    }
    return planExecuteDualState(dual_group, *current, target, HoldMode::NONE, name);
  }

  bool leftHoldRightMove(
      moveit::planning_interface::MoveGroupInterface& dual_group,
      const planning_scene::PlanningScenePtr& scene,
      const Eigen::Isometry3d& right_target,
      const std::string& name)
  {
    auto current = dual_group.getCurrentState(5.0);
    if (!current)
      return false;
    scene->setCurrentState(*current);
    moveit::core::RobotState target(*current);
    if (!buildDualPoseTarget(scene, *current, nullptr, &right_target, target)) {
      RCLCPP_ERROR(LOGGER, "[%s] Khong tao duoc RIGHT target", name.c_str());
      return false;
    }
    return planExecuteDualState(dual_group, *current, target, HoldMode::HOLD_LEFT, name);
  }

  bool rightHoldLeftMove(
      moveit::planning_interface::MoveGroupInterface& dual_group,
      const planning_scene::PlanningScenePtr& scene,
      const Eigen::Isometry3d& left_target,
      const std::string& name)
  {
    auto current = dual_group.getCurrentState(5.0);
    if (!current)
      return false;
    scene->setCurrentState(*current);
    moveit::core::RobotState target(*current);
    if (!buildDualPoseTarget(scene, *current, &left_target, nullptr, target)) {
      RCLCPP_ERROR(LOGGER, "[%s] Khong tao duoc LEFT target", name.c_str());
      return false;
    }
    return planExecuteDualState(dual_group, *current, target, HoldMode::HOLD_RIGHT, name);
  }

  bool findNearbyJointTarget(
      const planning_scene::PlanningScenePtr& scene,
      const moveit::core::RobotState& current,
      bool move_left,
      bool move_right,
      double requested_delta,
      moveit::core::RobotState& target)
  {
    const auto model = scene->getRobotModel();
    const auto* left_jmg = model->getJointModelGroup("left_ur_onrobot_manipulator");
    const auto* right_jmg = model->getJointModelGroup("right_ur_onrobot_manipulator");
    const auto* dual_jmg = model->getJointModelGroup("dual_arms");

    if (!left_jmg || !right_jmg || !dual_jmg)
      return false;

    std::vector<double> left_current;
    std::vector<double> right_current;
    current.copyJointGroupPositions(left_jmg, left_current);
    current.copyJointGroupPositions(right_jmg, right_current);

    struct Change
    {
      int index;
      double delta;
    };

    std::vector<Change> left_changes;
    std::vector<Change> right_changes;

    if (move_left) {
      for (std::size_t i = 0; i < left_current.size(); ++i) {
        left_changes.push_back({static_cast<int>(i), requested_delta});
        left_changes.push_back({static_cast<int>(i), -requested_delta});
        left_changes.push_back({static_cast<int>(i), 0.5 * requested_delta});
        left_changes.push_back({static_cast<int>(i), -0.5 * requested_delta});
      }
    }
    else {
      left_changes.push_back({-1, 0.0});
    }

    if (move_right) {
      for (std::size_t i = 0; i < right_current.size(); ++i) {
        right_changes.push_back({static_cast<int>(i), requested_delta});
        right_changes.push_back({static_cast<int>(i), -requested_delta});
        right_changes.push_back({static_cast<int>(i), 0.5 * requested_delta});
        right_changes.push_back({static_cast<int>(i), -0.5 * requested_delta});
      }
    }
    else {
      right_changes.push_back({-1, 0.0});
    }

    for (const auto& lc : left_changes) {
      for (const auto& rc : right_changes) {
        std::vector<double> lq = left_current;
        std::vector<double> rq = right_current;

        if (lc.index >= 0)
          lq[static_cast<std::size_t>(lc.index)] += lc.delta;
        if (rc.index >= 0)
          rq[static_cast<std::size_t>(rc.index)] += rc.delta;

        moveit::core::RobotState trial(current);
        trial.setJointGroupPositions(left_jmg, lq);
        trial.setJointGroupPositions(right_jmg, rq);
        trial.update();

        if (!trial.satisfiesBounds(dual_jmg))
          continue;

        if (scene->isStateColliding(trial, "", false))
          continue;

        target = trial;

        RCLCPP_INFO(
            LOGGER,
            "Nearby target FOUND: left_joint=%d d=%.3f rad, right_joint=%d d=%.3f rad",
            lc.index, lc.delta, rc.index, rc.delta);

        return true;
      }
    }

    return false;
  }

  void runPrimitiveDemo()
  {
    moveit::planning_interface::MoveGroupInterface dual_group(node_, "dual_arms");
    const auto model = dual_group.getRobotModel();
    const auto* dual_jmg = model->getJointModelGroup("dual_arms");

    if (!dual_jmg) {
      RCLCPP_ERROR(LOGGER, "primitive_demo: khong tim thay dual_arms");
      return;
    }

    auto home = dual_group.getCurrentState(5.0);
    if (!home) {
      RCLCPP_ERROR(LOGGER, "primitive_demo: khong doc duoc current state");
      return;
    }

    moveit::planning_interface::PlanningSceneInterface psi;
    auto scene = std::make_shared<planning_scene::PlanningScene>(model);
    scene->setCurrentState(*home);

    // Demo primitive chi can vat can la table.
    // Khong dua left_object/right_object vao local IK/collision test de tranh
    // lam bai test scheduler bi phu thuoc vao vi tri vat the.
    const auto table_objects = psi.getObjects({"table"});
    for (const auto& kv : table_objects)
      scene->processCollisionObjectMsg(kv.second);

    if (scene->isStateColliding(*home, "", false)) {
      RCLCPP_ERROR(LOGGER, "primitive_demo: HOME dang collision trong local scene");
      return;
    }

    const double joint_delta =
        node_->get_parameter("demo_joint_delta").as_double();

    RCLCPP_INFO(LOGGER, "==========================================");
    RCLCPP_INFO(LOGGER, " PRIMITIVE DEMO V2 - dual_arms 12DOF");
    RCLCPP_INFO(LOGGER, " P1 PARALLEL -> P2 LEFT HOLD -> P3 RIGHT HOLD -> P4 HOME");
    RCLCPP_INFO(LOGGER, " Demo nay dung nearby JOINT targets de test kien truc, khong phu thuoc IK Cartesian.");
    RCLCPP_INFO(LOGGER, "==========================================");

    // PHASE 1: ca hai arm cung thay doi joint trong mot target 12DOF.
    auto s0 = dual_group.getCurrentState(5.0);
    if (!s0)
      return;

    scene->setCurrentState(*s0);
    moveit::core::RobotState p1(*s0);

    if (!findNearbyJointTarget(scene, *s0, true, true, joint_delta, p1)) {
      RCLCPP_ERROR(LOGGER, "[PHASE1] Khong tim duoc nearby collision-free dual joint target");
      return;
    }

    if (!planExecuteDualState(
            dual_group, *s0, p1, HoldMode::NONE,
            "PHASE1 PARALLEL LEFT+RIGHT"))
      return;

    if (!node_->get_parameter("execute").as_bool()) {
      RCLCPP_WARN(
          LOGGER,
          "primitive_demo plan-only chi test PHASE1. Dung execute:=true de test du 4 phase.");
      return;
    }

    // PHASE 2: LEFT giu nguyen, RIGHT thay doi.
    auto s1 = dual_group.getCurrentState(5.0);
    if (!s1)
      return;

    scene->setCurrentState(*s1);
    moveit::core::RobotState p2(*s1);

    if (!findNearbyJointTarget(scene, *s1, false, true, joint_delta, p2)) {
      RCLCPP_ERROR(LOGGER, "[PHASE2] Khong tim duoc nearby RIGHT target");
      return;
    }

    if (!planExecuteDualState(
            dual_group, *s1, p2, HoldMode::HOLD_LEFT,
            "PHASE2 LEFT HOLD + RIGHT MOVE"))
      return;

    // PHASE 3: RIGHT giu nguyen, LEFT thay doi.
    auto s2 = dual_group.getCurrentState(5.0);
    if (!s2)
      return;

    scene->setCurrentState(*s2);
    moveit::core::RobotState p3(*s2);

    if (!findNearbyJointTarget(scene, *s2, true, false, joint_delta, p3)) {
      RCLCPP_ERROR(LOGGER, "[PHASE3] Khong tim duoc nearby LEFT target");
      return;
    }

    if (!planExecuteDualState(
            dual_group, *s2, p3, HoldMode::HOLD_RIGHT,
            "PHASE3 RIGHT HOLD + LEFT MOVE"))
      return;

    // PHASE 4: ca hai cung tro ve state ban dau.
    auto current = dual_group.getCurrentState(5.0);
    if (!current)
      return;

    scene->setCurrentState(*current);

    if (!planExecuteDualState(
            dual_group, *current, *home, HoldMode::NONE,
            "PHASE4 PARALLEL RETURN HOME"))
      return;

    RCLCPP_INFO(LOGGER, "==========================================");
    RCLCPP_INFO(LOGGER, " PRIMITIVE DEMO V2 4 PHASE SUCCESS");
    RCLCPP_INFO(LOGGER, "==========================================");
  }

  void runPoseDemo()
  {
    moveit::planning_interface::MoveGroupInterface dual_group(node_, "dual_arms");
    const auto model = dual_group.getRobotModel();
    const auto* left_jmg = model->getJointModelGroup("left_ur_onrobot_manipulator");
    const auto* right_jmg = model->getJointModelGroup("right_ur_onrobot_manipulator");
    const auto* dual_jmg = model->getJointModelGroup("dual_arms");

    if (!left_jmg || !right_jmg || !dual_jmg) {
      RCLCPP_ERROR(LOGGER, "POSE FAST: thieu JointModelGroup");
      return;
    }

    std::vector<std::string> left_hand_links;
    std::vector<std::string> right_hand_links;
    DualTaskTargets targets;

    try {
      RCLCPP_INFO(LOGGER, "POSE FAST: tim PREGRASP/GRASP nhanh...");
      targets = preparePickTargetsFast(model, left_hand_links, right_hand_links);
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(LOGGER, "POSE FAST target search failed: %s", e.what());
      return;
    }

    auto home = dual_group.getCurrentState(5.0);
    if (!home) {
      RCLCPP_ERROR(LOGGER, "POSE FAST: khong doc duoc current state");
      return;
    }

    auto applyGoal = [](moveit::core::RobotState& state,
                        const std::map<std::string, double>& goal) {
      for (const auto& kv : goal)
        state.setVariablePosition(kv.first, kv.second);
      state.update();
    };

    moveit::core::RobotState pre(*home);
    moveit::core::RobotState grasp(*home);
    applyGoal(pre, targets.pregrasp);
    applyGoal(grasp, targets.grasp);

    std::vector<double> left_pre_q, right_pre_q;
    std::vector<double> left_grasp_q, right_grasp_q;
    pre.copyJointGroupPositions(left_jmg, left_pre_q);
    pre.copyJointGroupPositions(right_jmg, right_pre_q);
    grasp.copyJointGroupPositions(left_jmg, left_grasp_q);
    grasp.copyJointGroupPositions(right_jmg, right_grasp_q);

    // Move only part way from pregrasp to grasp so this test DOES NOT touch the objects.
    constexpr double NEAR_FRACTION = 0.45;
    std::vector<double> left_near_q(left_pre_q.size());
    std::vector<double> right_near_q(right_pre_q.size());

    for (std::size_t i = 0; i < left_pre_q.size(); ++i)
      left_near_q[i] = left_pre_q[i] + NEAR_FRACTION * (left_grasp_q[i] - left_pre_q[i]);
    for (std::size_t i = 0; i < right_pre_q.size(); ++i)
      right_near_q[i] = right_pre_q[i] + NEAR_FRACTION * (right_grasp_q[i] - right_pre_q[i]);

    double left_delta = 0.0;
    double right_delta = 0.0;
    std::vector<double> left_home_q, right_home_q;
    home->copyJointGroupPositions(left_jmg, left_home_q);
    home->copyJointGroupPositions(right_jmg, right_home_q);
    for (std::size_t i = 0; i < left_pre_q.size(); ++i)
      left_delta = std::max(left_delta, std::abs(left_pre_q[i] - left_home_q[i]));
    for (std::size_t i = 0; i < right_pre_q.size(); ++i)
      right_delta = std::max(right_delta, std::abs(right_pre_q[i] - right_home_q[i]));

    RCLCPP_INFO(LOGGER,
                "POSE FAST target ready: LEFT angle=%.0f RIGHT angle=%.0f; max joint delta to pregrasp LEFT=%.3f RIGHT=%.3f rad",
                targets.left_angle * 180.0 / M_PI,
                targets.right_angle * 180.0 / M_PI,
                left_delta, right_delta);

    RCLCPP_INFO(LOGGER, "==========================================");
    RCLCPP_INFO(LOGGER, " POSE FAST DEMO START");
    RCLCPP_INFO(LOGGER, " P1 BOTH -> PREGRASP");
    RCLCPP_INFO(LOGGER, " P2 LEFT HOLD + RIGHT approach 45%%");
    RCLCPP_INFO(LOGGER, " P3 RIGHT HOLD + LEFT approach 45%%");
    RCLCPP_INFO(LOGGER, " P4 BOTH retreat -> PREGRASP");
    RCLCPP_INFO(LOGGER, " P5 BOTH -> HOME");
    RCLCPP_INFO(LOGGER, "==========================================");

    if (!planExecuteDualState(dual_group, *home, pre, HoldMode::NONE,
                              "POSE FAST P1 PARALLEL -> PREGRASP"))
      return;

    if (!node_->get_parameter("execute").as_bool()) {
      RCLCPP_WARN(LOGGER, "execute:=false, dung sau P1 plan-only");
      return;
    }

    auto s1 = dual_group.getCurrentState(5.0);
    if (!s1)
      return;
    moveit::core::RobotState p2(*s1);
    p2.setJointGroupPositions(right_jmg, right_near_q);
    p2.update();
    if (!planExecuteDualState(dual_group, *s1, p2, HoldMode::HOLD_LEFT,
                              "POSE FAST P2 LEFT HOLD + RIGHT APPROACH"))
      return;

    auto s2 = dual_group.getCurrentState(5.0);
    if (!s2)
      return;
    moveit::core::RobotState p3(*s2);
    p3.setJointGroupPositions(left_jmg, left_near_q);
    p3.update();
    if (!planExecuteDualState(dual_group, *s2, p3, HoldMode::HOLD_RIGHT,
                              "POSE FAST P3 RIGHT HOLD + LEFT APPROACH"))
      return;

    auto s3 = dual_group.getCurrentState(5.0);
    if (!s3)
      return;
    if (!planExecuteDualState(dual_group, *s3, pre, HoldMode::NONE,
                              "POSE FAST P4 PARALLEL RETREAT"))
      return;

    auto s4 = dual_group.getCurrentState(5.0);
    if (!s4)
      return;
    if (!planExecuteDualState(dual_group, *s4, *home, HoldMode::NONE,
                              "POSE FAST P5 PARALLEL HOME"))
      return;

    RCLCPP_INFO(LOGGER, "==========================================");
    RCLCPP_INFO(LOGGER, " POSE FAST DEMO SUCCESS");
    RCLCPP_INFO(LOGGER, "==========================================");
  }

  bool executeOneGripper(bool left, double width, const std::string& phase_name)
  {
    const std::string group = left ? "left_ur_onrobot_gripper" : "right_ur_onrobot_gripper";
    const std::string joint = left ? "left_finger_width" : "right_finger_width";
    moveit::planning_interface::MoveGroupInterface gripper(node_, group);
    gripper.setStartStateToCurrentState();
    gripper.setMaxVelocityScalingFactor(0.30);
    gripper.setMaxAccelerationScalingFactor(0.25);
    if (!gripper.setJointValueTarget(std::map<std::string, double>{{joint, width}})) {
      RCLCPP_ERROR(LOGGER, "[%s] set one-gripper target failed", phase_name.c_str());
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (gripper.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "[%s] one-gripper PLAN FAILED", phase_name.c_str());
      return false;
    }
    if (!node_->get_parameter("execute").as_bool())
      return true;
    if (gripper.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "[%s] one-gripper EXECUTION FAILED", phase_name.c_str());
      return false;
    }
    RCLCPP_INFO(LOGGER, "[%s] one-gripper EXECUTION SUCCESS", phase_name.c_str());
    return true;
  }

  double setupStack4Scene()
  {
    moveit::planning_interface::PlanningSceneInterface psi;

    const double table_top =
        node_->get_parameter("stack_table_top").as_double();
    const double table_thickness =
        node_->get_parameter("stack_table_thickness").as_double();
    const double size =
        node_->get_parameter("stack_object_size").as_double();

    if (table_thickness <= 0.0 || size <= 0.0)
      throw std::runtime_error(
          "STACK4: stack_table_thickness and stack_object_size must be > 0");

    // IMPORTANT:
    // Remove any stale/old collision object named "table" first. Previously
    // setupStack4Scene() read whatever "table" happened to be in PlanningScene;
    // if another node had published an older table at center_z=0, the inferred
    // top became +0.05 and all cubes floated ~5.3 cm above the real tabletop.
    psi.removeCollisionObjects(
        {"table", "left_object", "right_object",
         "stack_A", "stack_B", "stack_C", "stack_D"});
    rclcpp::sleep_for(std::chrono::milliseconds(300));

    // Re-create the table used by STACK4 from ONE source of truth.
    // For the current environment:
    //   top = -0.003 m
    //   thickness = 0.10 m
    // => center_z = -0.053 m
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = "world";
    table.id = "table";

    shape_msgs::msg::SolidPrimitive table_box;
    table_box.type = shape_msgs::msg::SolidPrimitive::BOX;
    table_box.dimensions = {1.0, 1.0, table_thickness};

    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.0;
    table_pose.position.y = 0.0;
    table_pose.position.z = table_top - 0.5 * table_thickness;

    table.primitives.push_back(table_box);
    table.primitive_poses.push_back(table_pose);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;

    if (!psi.applyCollisionObject(table))
      throw std::runtime_error("STACK4: failed to apply corrected table");

    constexpr double SPAWN_CLEARANCE = 0.0005;
    const double cube_z =
        table_top + 0.5 * size + SPAWN_CLEARANCE;

    RCLCPP_INFO(
        LOGGER,
        "STACK4 TABLE FORCED: top=%.4f thickness=%.4f center_z=%.4f",
        table_top, table_thickness, table_pose.position.z);
    RCLCPP_INFO(
        LOGGER,
        "STACK4 CUBE SPAWN: size=%.4f center_z=%.4f bottom=%.4f "
        "(table_top=%.4f clearance=%.4f)",
        size, cube_z, cube_z - 0.5 * size,
        table_top, SPAWN_CLEARANCE);

    struct ObjDef { std::string id; double x; double y; };
    const std::vector<ObjDef> defs = {
      {"stack_A", node_->get_parameter("stack_a_x").as_double(),
                  node_->get_parameter("stack_a_y").as_double()},
      {"stack_B", node_->get_parameter("stack_b_x").as_double(),
                  node_->get_parameter("stack_b_y").as_double()},
      {"stack_C", node_->get_parameter("stack_c_x").as_double(),
                  node_->get_parameter("stack_c_y").as_double()},
      {"stack_D", node_->get_parameter("stack_d_x").as_double(),
                  node_->get_parameter("stack_d_y").as_double()},
    };

    std::vector<moveit_msgs::msg::CollisionObject> add;
    for (const auto& d : defs) {
      moveit_msgs::msg::CollisionObject obj;
      obj.header.frame_id = "world";
      obj.id = d.id;

      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::BOX;
      box.dimensions = {size, size, size};

      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.x = d.x;
      pose.position.y = d.y;
      pose.position.z = cube_z;

      obj.primitives.push_back(box);
      obj.primitive_poses.push_back(pose);
      obj.operation = moveit_msgs::msg::CollisionObject::ADD;
      add.push_back(obj);

      RCLCPP_INFO(
          LOGGER,
          "STACK4 SPAWN %s: (%.3f, %.3f, %.4f), bottom=%.4f",
          d.id.c_str(), d.x, d.y, cube_z, cube_z - 0.5 * size);
    }

    if (!psi.applyCollisionObjects(add))
      throw std::runtime_error("STACK4: failed to apply stack cubes");

    rclcpp::sleep_for(std::chrono::milliseconds(300));
    return table_top;
  }

  void applyGoalToState(
      moveit::core::RobotState& state,
      const std::map<std::string, double>& goal)
  {
    for (const auto& kv : goal)
      state.setVariablePosition(kv.first, kv.second);
    state.update();
  }

  moveit::core::RobotState makeContactState(
      const moveit::core::RobotState& pre,
      const moveit::core::RobotState& grasp,
      const moveit::core::JointModelGroup* left_jmg,
      const moveit::core::JointModelGroup* right_jmg)
  {
    constexpr double CONTACT_FRACTION = 0.55;
    std::vector<double> lp, rp, lg, rg;
    pre.copyJointGroupPositions(left_jmg, lp);
    pre.copyJointGroupPositions(right_jmg, rp);
    grasp.copyJointGroupPositions(left_jmg, lg);
    grasp.copyJointGroupPositions(right_jmg, rg);

    for (std::size_t i = 0; i < lp.size(); ++i)
      lp[i] += CONTACT_FRACTION * (lg[i] - lp[i]);
    for (std::size_t i = 0; i < rp.size(); ++i)
      rp[i] += CONTACT_FRACTION * (rg[i] - rp[i]);

    moveit::core::RobotState out(pre);
    out.setJointGroupPositions(left_jmg, lp);
    out.setJointGroupPositions(right_jmg, rp);
    out.update();
    return out;
  }

  bool makeOneArmStackTargetRigid(
      const moveit::core::RobotState& current,
      bool move_left,
      const Eigen::Isometry3d& hand_to_object,
      double object_x,
      double object_y,
      double object_z,
      double object_yaw,
      moveit::core::RobotState& target)
  {
    Eigen::Isometry3d world_object = Eigen::Isometry3d::Identity();
    world_object.translation() = Eigen::Vector3d(object_x, object_y, object_z);
    world_object.linear() =
        Eigen::AngleAxisd(object_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // Preserve the ACTUAL rigid transform captured at attachment:
    // T_world_object = T_world_hand * T_hand_object
    // => T_world_hand = T_world_object * inverse(T_hand_object)
    const Eigen::Isometry3d world_hand =
        world_object * hand_to_object.inverse();

    const bool solved = move_left
        ? buildDualPoseTargetIKOnly(current, &world_hand, nullptr, target)
        : buildDualPoseTargetIKOnly(current, nullptr, &world_hand, target);
    if (!solved) {
      RCLCPP_WARN(LOGGER,
          "STACK %s IK failed: object=(%.3f, %.3f, %.3f) yaw=%.0f deg "
          "TCP=(%.3f, %.3f, %.3f)",
          move_left ? "LEFT" : "RIGHT", object_x, object_y, object_z,
          object_yaw * 180.0 / M_PI, world_hand.translation().x(),
          world_hand.translation().y(), world_hand.translation().z());
    }
    return solved;
  }

  double activeJointDistance(
      const moveit::core::RobotState& a,
      const moveit::core::RobotState& b,
      bool left) const
  {
    const auto model = a.getRobotModel();
    const auto* jmg = model->getJointModelGroup(
        left ? "left_ur_onrobot_manipulator"
             : "right_ur_onrobot_manipulator");
    if (!jmg)
      return 1e9;

    std::vector<double> qa, qb;
    a.copyJointGroupPositions(jmg, qa);
    b.copyJointGroupPositions(jmg, qb);

    double d = 0.0;
    for (std::size_t i = 0; i < qa.size() && i < qb.size(); ++i)
      d += std::abs(qb[i] - qa[i]);
    return d;
  }

  bool findBestCenterRoute(
      const moveit::core::RobotState& current,
      bool move_left,
      const Eigen::Isometry3d& hand_to_object,
      double cx,
      double cy,
      double place_z,
      double pre_dz,
      double transit_z,
      double entry_offset,
      double& selected_object_yaw,
      double& selected_entry_x,
      double& selected_entry_y)
  {
    (void)entry_offset;

    // Reconstruct the object's ACTUAL current pose from the rigid grasp.
    //
    // IMPORTANT: RobotState may have dirty link transforms after joint values
    // were changed by IK/normalization. Calling getGlobalLinkTransform() on a
    // dirty state triggers MoveIt's checkLinkTransforms() assertion in Humble.
    // Work on a local copy and explicitly update all transforms first.
    moveit::core::RobotState current_clean(current);
    current_clean.update();

    const std::string tip =
        move_left ? "left_gripper_tcp" : "right_gripper_tcp";
    const Eigen::Isometry3d world_hand =
        current_clean.getGlobalLinkTransform(tip);
    const Eigen::Isometry3d world_object_now =
        world_hand * hand_to_object;

    const double ox = world_object_now.translation().x();
    const double oy = world_object_now.translation().y();

    // For level 1, search the complete 360 deg because the first cube can
    // define the orientation of the whole column.
    // After level 1, keep the same square orientation; +90/+180/+270 are
    // geometrically equivalent for a square cube.
    std::vector<double> yaw_deg;
    if (std::isfinite(stack_column_yaw_)) {
      const double base_deg = stack_column_yaw_ * 180.0 / M_PI;
      for (int k = 0; k < 4; ++k) {
        double deg = std::fmod(base_deg + 90.0 * k, 360.0);
        if (deg < 0.0) deg += 360.0;
        yaw_deg.push_back(deg);
      }
    }
    else {
      yaw_deg.reserve(36);
      for (int i = 0; i < 36; ++i)
        yaw_deg.push_back(10.0 * i);
    }

    // Do NOT jump from the corner directly to x=center.
    // Search several radial entry points between the current object position
    // and the exact center. The final PLACE still remains exactly (cx, cy).
    static const std::vector<double> entry_fractions =
        {0.35, 0.50, 0.65, 0.80};

    bool found = false;
    double best_score = std::numeric_limits<double>::infinity();
    double best_yaw = 0.0;
    double best_entry_x = ox;
    double best_entry_y = oy;

    for (double deg : yaw_deg) {
      const double yaw = deg * M_PI / 180.0;

      // First prove that the EXACT CENTER final pose is actually reachable.
      moveit::core::RobotState place_check(current_clean);
      if (!makeOneArmStackTargetRigid(
              current_clean, move_left, hand_to_object,
              cx, cy, place_z, yaw, place_check)) {
        RCLCPP_WARN(
            LOGGER,
            "STACK %s exact CENTER PLACE unreachable, yaw=%.0f deg",
            move_left ? "LEFT" : "RIGHT", deg);
        continue;
      }

      for (double f : entry_fractions) {
        const double entry_x = ox + f * (cx - ox);
        const double entry_y = oy + f * (cy - oy);

        moveit::core::RobotState entry(current_clean);
        if (!makeOneArmStackTargetRigid(
                current_clean, move_left, hand_to_object,
                entry_x, entry_y, transit_z, yaw, entry))
          continue;

        moveit::core::RobotState center_high(entry);
        if (!makeOneArmStackTargetRigid(
                entry, move_left, hand_to_object,
                cx, cy, transit_z, yaw, center_high))
          continue;

        moveit::core::RobotState pre(center_high);
        if (!makeOneArmStackTargetRigid(
                center_high, move_left, hand_to_object,
                cx, cy, place_z + pre_dz, yaw, pre))
          continue;

        moveit::core::RobotState place(pre);
        if (!makeOneArmStackTargetRigid(
                pre, move_left, hand_to_object,
                cx, cy, place_z, yaw, place))
          continue;

        const double score =
            activeJointDistance(current_clean, entry, move_left) +
            activeJointDistance(entry, center_high, move_left) +
            activeJointDistance(center_high, pre, move_left) +
            activeJointDistance(pre, place, move_left);

        RCLCPP_INFO(
            LOGGER,
            "STACK %s CENTER route candidate: yaw=%.0f entry=(%.3f,%.3f) f=%.2f score=%.3f",
            move_left ? "LEFT" : "RIGHT",
            deg, entry_x, entry_y, f, score);

        if (score < best_score) {
          best_score = score;
          best_yaw = yaw;
          best_entry_x = entry_x;
          best_entry_y = entry_y;
          found = true;
        }
      }
    }

    if (!found)
      return false;

    selected_object_yaw = best_yaw;
    selected_entry_x = best_entry_x;
    selected_entry_y = best_entry_y;

    RCLCPP_INFO(
        LOGGER,
        "STACK %s CENTER route BEST: yaw=%.0f entry=(%.3f,%.3f) score=%.3f",
        move_left ? "LEFT" : "RIGHT",
        selected_object_yaw * 180.0 / M_PI,
        selected_entry_x, selected_entry_y,
        best_score);

    return true;
  }

  bool moveOneArmObjectPose(
      moveit::planning_interface::MoveGroupInterface& dual_group,
      bool move_left,
      const Eigen::Isometry3d& hand_to_object,
      double x,
      double y,
      double z,
      double object_yaw,
      const std::string& phase_name)
  {
    auto current = dual_group.getCurrentState(5.0);
    if (!current)
      return false;

    moveit::core::RobotState target(*current);
    if (!makeOneArmStackTargetRigid(
            *current, move_left, hand_to_object,
            x, y, z, object_yaw, target)) {
      RCLCPP_ERROR(
          LOGGER, "[%s] IK failed at object pose (%.3f, %.3f, %.3f)",
          phase_name.c_str(), x, y, z);
      return false;
    }

    return planExecuteDualState(
        dual_group, *current, target,
        move_left ? HoldMode::HOLD_RIGHT : HoldMode::HOLD_LEFT,
        phase_name);
  }

  bool restoreOneArmFromReference(
      moveit::planning_interface::MoveGroupInterface& dual_group,
      const moveit::core::RobotState& reference,
      bool restore_left,
      const std::string& phase_name)
  {
    // The reference state is captured immediately BEFORE the PLACE motion.
    // After open + detach (+ a short vertical retreat), restore the active arm
    // to exactly those saved joint values.  Use the single-arm planning group
    // instead of planning all 12 DOF, which is both simpler and more robust.
    (void)dual_group;

    const ArmSide arm = restore_left ? ArmSide::LEFT : ArmSide::RIGHT;
    const ArmConfig cfg = armConfig(arm, "");

    moveit::planning_interface::MoveGroupInterface arm_group(node_, cfg.arm_group);
    const auto model = arm_group.getRobotModel();
    const auto* jmg = model->getJointModelGroup(cfg.arm_group);
    if (!jmg) {
      RCLCPP_ERROR(LOGGER, "[%s] joint model group %s not found",
                   phase_name.c_str(), cfg.arm_group.c_str());
      return false;
    }

    std::vector<double> q_reference;
    reference.copyJointGroupPositions(jmg, q_reference);

    arm_group.setStartStateToCurrentState();
    arm_group.setJointValueTarget(q_reference);
    arm_group.setPlannerId("RRTConnectkConfigDefault");
    arm_group.setPlanningTime(5.0);
    arm_group.setNumPlanningAttempts(5);
    arm_group.setMaxVelocityScalingFactor(0.25);
    arm_group.setMaxAccelerationScalingFactor(0.25);

    RCLCPP_INFO(
        LOGGER,
        "[%s] returning %s arm to exact PRE-PLACE joint pose",
        phase_name.c_str(), restore_left ? "LEFT" : "RIGHT");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = arm_group.plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "[%s] PRE-PLACE RETURN PLAN FAILED",
                   phase_name.c_str());
      return false;
    }

    if (!node_->get_parameter("execute").as_bool()) {
      RCLCPP_INFO(LOGGER, "[%s] PRE-PLACE RETURN PLAN SUCCESS",
                  phase_name.c_str());
      return true;
    }

    const auto exec_result = arm_group.execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "[%s] PRE-PLACE RETURN EXECUTION FAILED",
                   phase_name.c_str());
      return false;
    }

    RCLCPP_INFO(LOGGER, "[%s] PRE-PLACE RETURN SUCCESS",
                phase_name.c_str());
    return true;
  }


  bool placeAttachedObjectAtCenterMTC(
      bool place_left,
      const std::string& object_name,
      int level,
      double& selected_yaw)
  {
    const std::string arm_group =
        place_left ? "left_ur_onrobot_manipulator" : "right_ur_onrobot_manipulator";
    const std::string hand_group =
        place_left ? "left_ur_onrobot_gripper" : "right_ur_onrobot_gripper";
    const std::string hand_frame =
        place_left ? "left_gripper_tcp" : "right_gripper_tcp";

    const double cx = node_->get_parameter("stack_center_x").as_double();
    const double cy = node_->get_parameter("stack_center_y").as_double();
    const double size = node_->get_parameter("stack_object_size").as_double();
    const double gap = node_->get_parameter("stack_gap").as_double();
    constexpr double STACK_BASE_CLEARANCE = 0.0005;
    const double z =
        stack_table_top_ + 0.5 * size +
        level * (size + gap) + STACK_BASE_CLEARANCE;

    // Prefer the column orientation selected by level 1. If that exact
    // square orientation is not reachable for the other arm, fall back to the
    // remaining 10-degree yaw candidates instead of making the arm stand still.
    std::vector<double> yaw_deg;
    auto add_yaw_unique = [&yaw_deg](double deg) {
      deg = std::fmod(deg, 360.0);
      if (deg < 0.0) deg += 360.0;
      for (double x : yaw_deg)
        if (std::abs(x - deg) < 1e-6)
          return;
      yaw_deg.push_back(deg);
    };

    if (std::isfinite(stack_column_yaw_)) {
      const double base_deg = stack_column_yaw_ * 180.0 / M_PI;
      for (int k = 0; k < 4; ++k)
        add_yaw_unique(base_deg + 90.0 * k);

      // Fallback only after the four aligned square orientations.
      for (int i = 0; i < 36; ++i)
        add_yaw_unique(10.0 * i);
    } else {
      // First cube: try square-aligned orientations first, then all yaw.
      add_yaw_unique(0.0);
      add_yaw_unique(90.0);
      add_yaw_unique(180.0);
      add_yaw_unique(270.0);
      for (int i = 0; i < 36; ++i)
        add_yaw_unique(10.0 * i);
    }

    // Cube directly below the one currently being placed.
    // The active gripper can touch this support cube while lowering the held
    // cube onto the stack. Without this ACM exception, ComputeIK may reject
    // every otherwise-valid level-2+ pose because the fingers touch the cube
    // below.
    std::string support_object;
    if (level == 1) support_object = "stack_C";
    else if (level == 2) support_object = "stack_A";
    else if (level == 3) support_object = "stack_D";

    for (double deg : yaw_deg) {
      const double yaw = deg * M_PI / 180.0;

      RCLCPP_INFO(
          LOGGER,
          "STACK MTC PLACE L%d %s: OBJECT target=(%.3f, %.3f, %.3f), yaw=%.0f",
          level + 1, place_left ? "LEFT" : "RIGHT", cx, cy, z, deg);

      mtc::Task t;
      t.stages()->setName("stack_mtc_place_" + object_name);
      t.loadRobotModel(node_);
      t.setProperty("group", arm_group);
      t.setProperty("eef", hand_group);
      t.setProperty("ik_frame", hand_frame);

      mtc::Stage* current_state_ptr = nullptr;
      auto current = std::make_unique<mtc::stages::CurrentState>("current attached state");
      current_state_ptr = current.get();
      t.add(std::move(current));

      // Keep the OLD fast pick algorithm unchanged.
      // Only fix the PLACE scene chain: GeneratePlacePose must monitor the
      // scene after support-contact permission has been applied.
      mtc::Stage* place_monitor_ptr = current_state_ptr;

      if (!support_object.empty()) {
        const auto* hand_jmg =
            t.getRobotModel()->getJointModelGroup(hand_group);
        if (!hand_jmg) {
          RCLCPP_ERROR(LOGGER, "STACK MTC PLACE: missing hand group %s",
                       hand_group.c_str());
          return false;
        }

        const auto hand_collision_links =
            hand_jmg->getLinkModelNamesWithCollisionGeometry();

        auto allow_support_touch =
            std::make_unique<mtc::stages::ModifyPlanningScene>(
                "allow active gripper touch support cube");
        allow_support_touch->allowCollisions(
            support_object, hand_collision_links, true);

        place_monitor_ptr = allow_support_touch.get();
        t.add(std::move(allow_support_touch));

        RCLCPP_INFO(
            LOGGER,
            "STACK MTC PLACE L%d %s: allow gripper/support contact with %s; "
            "GeneratePlacePose monitors POST-ACM stage",
            level + 1, place_left ? "LEFT" : "RIGHT",
            support_object.c_str());
      }

      auto sampling = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
      sampling->setProperty("planning_pipeline", "ompl");
      sampling->setPlannerId("RRTConnectkConfigDefault");

      auto interpolation = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

      auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
      cartesian->setMaxVelocityScalingFactor(0.25);
      cartesian->setMaxAccelerationScalingFactor(0.20);
      cartesian->setStepSize(0.005);

      {
        auto connect = std::make_unique<mtc::stages::Connect>(
            "move active arm to place",
            mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling}});
        connect->setTimeout(8.0);
        connect->properties().configureInitFrom(mtc::Stage::PARENT);
        t.add(std::move(connect));
      }

      {
        auto place = std::make_unique<mtc::SerialContainer>(
            "place attached object at exact center");
        t.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
        place->properties().configureInitFrom(
            mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        {
          auto gen = std::make_unique<mtc::stages::GeneratePlacePose>(
              "generate exact center object pose");
          gen->properties().configureInitFrom(mtc::Stage::PARENT);
          gen->properties().set("marker_ns", "stack_center_object_pose");
          gen->setObject(object_name);

          geometry_msgs::msg::PoseStamped target;
          target.header.frame_id = "world";
          target.pose.position.x = cx;
          target.pose.position.y = cy;
          target.pose.position.z = z;

          Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
          target.pose.orientation.x = q.x();
          target.pose.orientation.y = q.y();
          target.pose.orientation.z = q.z();
          target.pose.orientation.w = q.w();

          gen->setPose(target);
          gen->setMonitoredStage(place_monitor_ptr);

          auto ik = std::make_unique<mtc::stages::ComputeIK>(
              "compute IK using attached object frame", std::move(gen));
          ik->setMaxIKSolutions(16);
          ik->setMinSolutionDistance(0.2);
          ik->setIKFrame(object_name);
          ik->properties().configureInitFrom(
              mtc::Stage::PARENT, {"eef", "group"});
          ik->properties().configureInitFrom(
              mtc::Stage::INTERFACE, {"target_pose"});
          place->insert(std::move(ik));
        }

        {
          auto open = std::make_unique<mtc::stages::MoveTo>(
              "open active gripper", interpolation);
          open->setGroup(hand_group);
          open->setGoal("open");
          place->insert(std::move(open));
        }

        {
          auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>(
              "detach stacked object");
          detach->detachObject(object_name, hand_frame);
          place->insert(std::move(detach));
        }

        if (!support_object.empty()) {
          const auto* hand_jmg =
              t.getRobotModel()->getJointModelGroup(hand_group);
          const auto hand_collision_links =
              hand_jmg->getLinkModelNamesWithCollisionGeometry();

          auto forbid_support_touch =
              std::make_unique<mtc::stages::ModifyPlanningScene>(
                  "restore gripper/support collision checking");
          forbid_support_touch->allowCollisions(
              support_object, hand_collision_links, false);
          place->insert(std::move(forbid_support_touch));
        }

        // Do NOT put a long Cartesian retreat inside this MTC place task.
        // At exact center the RIGHT arm can place/detach successfully, but the
        // old 6-10 cm vertical Cartesian retreat only achieved ~2.86 cm and
        // therefore invalidated every otherwise-valid place solution.
        //
        // After this task executes, placeOneStackLevel() already calls
        // restoreOneArmFromReference(), which moves the active arm safely back
        // to its saved lift state while the other arm remains held.

        t.add(std::move(place));
      }

      try {
        t.init();
      } catch (const mtc::InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, "STACK MTC PLACE init failed:\n" << e);
        continue;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "STACK MTC PLACE init exception: %s", e.what());
        continue;
      }

      if (!t.plan(5) || t.solutions().empty()) {
        RCLCPP_WARN(
            LOGGER,
            "STACK MTC PLACE L%d %s: no solution at yaw %.0f",
            level + 1, place_left ? "LEFT" : "RIGHT", deg);
        continue;
      }

      RCLCPP_INFO(
          LOGGER,
          "STACK MTC PLACE L%d %s PLAN SUCCESS at exact center, yaw=%.0f",
          level + 1, place_left ? "LEFT" : "RIGHT", deg);

      t.introspection().publishSolution(*t.solutions().front());

      if (node_->get_parameter("execute").as_bool()) {
        const auto result = t.execute(*t.solutions().front());
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          RCLCPP_ERROR(
              LOGGER,
              "STACK MTC PLACE L%d %s EXECUTION FAILED",
              level + 1, place_left ? "LEFT" : "RIGHT");
          return false;
        }
      }

      selected_yaw = yaw;
      RCLCPP_INFO(
          LOGGER,
          "STACK MTC PLACE L%d %s SUCCESS",
          level + 1, place_left ? "LEFT" : "RIGHT");
      return true;
    }

    RCLCPP_ERROR(
        LOGGER,
        "STACK MTC PLACE L%d %s FAILED for all object yaw candidates",
        level + 1, place_left ? "LEFT" : "RIGHT");
    return false;
  }

  bool placeOneStackLevel(
      moveit::planning_interface::MoveGroupInterface& dual_group,
      const moveit::core::RobotState& lift_reference,
      bool place_left,
      const std::string& object_name,
      const Eigen::Isometry3d& hand_to_object,
      int level)
  {
    (void)hand_to_object;

    double selected_yaw = 0.0;
    if (!placeAttachedObjectAtCenterMTC(
            place_left, object_name, level, selected_yaw))
      return false;

    if (!std::isfinite(stack_column_yaw_)) {
      stack_column_yaw_ = selected_yaw;
      RCLCPP_INFO(
          LOGGER,
          "STACK COLUMN yaw LOCKED by MTC at %.0f deg",
          stack_column_yaw_ * 180.0 / M_PI);
    }

    return restoreOneArmFromReference(
        dual_group, lift_reference, place_left,
        "STACK level " + std::to_string(level + 1) +
        " RETURN TO LIFT AFTER MTC PLACE");
  }

  bool runStackRound(
      moveit::planning_interface::MoveGroupInterface& dual_group,
      const std::string& left_object,
      const std::string& right_object,
      int right_level,
      int left_level,
      int round_index)
  {
    const auto model = dual_group.getRobotModel();
    const auto* left_jmg = model->getJointModelGroup("left_ur_onrobot_manipulator");
    const auto* right_jmg = model->getJointModelGroup("right_ur_onrobot_manipulator");
    if (!left_jmg || !right_jmg)
      return false;

    std::vector<std::string> left_hand_links, right_hand_links;
    DualTaskTargets targets;
    try {
      targets = preparePickTargetsFastForObjects(
          model, left_object, right_object, left_hand_links, right_hand_links);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(LOGGER, "STACK ROUND %d target search failed: %s", round_index, e.what());
      return false;
    }

    auto home = dual_group.getCurrentState(5.0);
    if (!home)
      return false;
    moveit::core::RobotState pre(*home), grasp(*home), lift(*home);
    applyGoalToState(pre, targets.pregrasp);
    applyGoalToState(grasp, targets.grasp);
    applyGoalToState(lift, targets.lift);

    // STACK4 must reach the REAL top-down grasp pose before closing/attaching.
    // Do not use the old 55% contact interpolation here, otherwise the object
    // is attached while the gripper is still several centimeters away.
    moveit::core::RobotState contact(grasp);

    RCLCPP_INFO(LOGGER, "========== STACK ROUND %d: LEFT=%s RIGHT=%s ==========",
                round_index, left_object.c_str(), right_object.c_str());

    if (!executeDualGrippers(0.100, "STACK R" + std::to_string(round_index) + " OPEN BOTH")) return false;
    if (!planExecuteDualState(dual_group, *home, pre, HoldMode::NONE,
                              "STACK R" + std::to_string(round_index) + " PARALLEL PREGRASP")) return false;
    auto s = dual_group.getCurrentState(5.0); if (!s) return false;
    if (!planExecuteDualState(
            dual_group, *s, contact, HoldMode::NONE,
            "STACK R" + std::to_string(round_index) + " PARALLEL REAL GRASP"))
      return false;

    RCLCPP_INFO(
        LOGGER,
        "STACK R%d reached exact TOP-DOWN GRASP state",
        round_index);
    rclcpp::sleep_for(std::chrono::milliseconds(250));

    // IMPORTANT:
    // At the exact grasp pose the open fingers are already touching/overlapping
    // the collision geometry of the objects. If we ask MoveIt to CLOSE while
    // the objects are still world collision objects, the gripper plan is rejected.
    //
    // Capture the real hand<->object transform FIRST, then attach the objects
    // with the gripper links as touch_links. This changes only the collision
    // model: the grippers are then allowed to close onto their own objects.
    auto attach_state = dual_group.getCurrentState(5.0);
    if (!attach_state)
      return false;
    attach_state->update();

    moveit::planning_interface::PlanningSceneInterface psi;
    const auto object_poses = psi.getObjectPoses({left_object, right_object});
    if (!object_poses.count(left_object) || !object_poses.count(right_object)) {
      RCLCPP_ERROR(LOGGER, "STACK R%d cannot read object poses at REAL GRASP", round_index);
      return false;
    }

    const Eigen::Isometry3d world_left_object =
        poseToEigen(object_poses.at(left_object));
    const Eigen::Isometry3d world_right_object =
        poseToEigen(object_poses.at(right_object));

    const Eigen::Isometry3d left_hand_to_object =
        attach_state->getGlobalLinkTransform("left_gripper_tcp").inverse() *
        world_left_object;
    const Eigen::Isometry3d right_hand_to_object =
        attach_state->getGlobalLinkTransform("right_gripper_tcp").inverse() *
        world_right_object;

    const bool la =
        dual_group.attachObject(left_object, "left_gripper_tcp", left_hand_links);
    const bool ra =
        dual_group.attachObject(right_object, "right_gripper_tcp", right_hand_links);

    if (!la || !ra) {
      RCLCPP_ERROR(LOGGER, "STACK R%d PRE-CLOSE ATTACH failed: L=%s R=%s",
                   round_index, la ? "OK" : "FAIL", ra ? "OK" : "FAIL");
      if (la) dual_group.detachObject(left_object);
      if (ra) dual_group.detachObject(right_object);
      return false;
    }

    RCLCPP_INFO(
        LOGGER,
        "STACK R%d contact objects attached with touch_links; now CLOSE BOTH",
        round_index);
    rclcpp::sleep_for(std::chrono::milliseconds(300));

    if (!executeDualGrippers(
            0.0, "STACK R" + std::to_string(round_index) + " CLOSE BOTH")) {
      RCLCPP_ERROR(
          LOGGER,
          "STACK R%d CLOSE failed even after touch-link attach; detaching for clean recovery",
          round_index);
      dual_group.detachObject(left_object);
      dual_group.detachObject(right_object);
      return false;
    }

    RCLCPP_INFO(
        LOGGER,
        "STACK R%d REAL GRASP COMPLETE: exact pose + closed grippers + attached objects",
        round_index);

    s = dual_group.getCurrentState(5.0); if (!s) return false;
    if (!planExecuteDualState(dual_group, *s, lift, HoldMode::NONE,
                              "STACK R" + std::to_string(round_index) + " PARALLEL LIFT")) return false;
    auto lift_ref_ptr = dual_group.getCurrentState(5.0); if (!lift_ref_ptr) return false;
    moveit::core::RobotState lift_ref(*lift_ref_ptr);

    // RIGHT places first while LEFT keeps holding its object high.
    if (!placeOneStackLevel(
            dual_group, lift_ref, false, right_object,
            right_hand_to_object, right_level))
      return false;

    // Then LEFT places on the next level while RIGHT stays parked.
    if (!placeOneStackLevel(
            dual_group, lift_ref, true, left_object,
            left_hand_to_object, left_level))
      return false;

    auto now = dual_group.getCurrentState(5.0); if (!now) return false;
    if (!planExecuteDualState(dual_group, *now, *home, HoldMode::NONE,
                              "STACK R" + std::to_string(round_index) + " PARALLEL HOME")) return false;
    return true;
  }

  void runStack4Demo()
  {
    stack_column_yaw_ = std::numeric_limits<double>::quiet_NaN();

    RCLCPP_INFO(LOGGER, "==========================================");
    RCLCPP_INFO(LOGGER, " STACK4 TOP-DOWN: 4 CORNERS -> EXACT CENTER COLUMN");
    RCLCPP_INFO(LOGGER, " ROUND1: LEFT=A RIGHT=C; RIGHT->L1, LEFT->L2");
    RCLCPP_INFO(LOGGER, " ROUND2: LEFT=B RIGHT=D; RIGHT->L3, LEFT->L4");
    RCLCPP_INFO(LOGGER, " CENTER target: x=%.3f y=%.3f",
                node_->get_parameter("stack_center_x").as_double(),
                node_->get_parameter("stack_center_y").as_double());
    RCLCPP_INFO(LOGGER, "==========================================");

    try {
      stack_table_top_ = setupStack4Scene();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(LOGGER, "STACK4 scene setup failed: %s", e.what());
      return;
    }

    moveit::planning_interface::MoveGroupInterface dual_group(node_, "dual_arms");
    if (!runStackRound(dual_group, "stack_A", "stack_C", 0, 1, 1))
      return;
    rclcpp::sleep_for(std::chrono::milliseconds(800));
    if (!runStackRound(dual_group, "stack_B", "stack_D", 2, 3, 2))
      return;

    RCLCPP_INFO(LOGGER, "==========================================");
    RCLCPP_INFO(LOGGER, " STACK4 DEMO SUCCESS: A/C/B/D -> 4 LEVEL COLUMN");
    RCLCPP_INFO(LOGGER, "==========================================");
  }

  DualTaskTargets preparePickTargetsFastForObjects(
      const moveit::core::RobotModelConstPtr& robot_model,
      const std::string& left_object_name,
      const std::string& right_object_name,
      std::vector<std::string>& left_hand_links,
      std::vector<std::string>& right_hand_links)
  {
    const ArmConfig left{
        "left_ur_onrobot_manipulator",
        "left_ur_onrobot_gripper",
        "left_gripper_tcp",
        left_object_name};

    const ArmConfig right{
        "right_ur_onrobot_manipulator",
        "right_ur_onrobot_gripper",
        "right_gripper_tcp",
        right_object_name};

    moveit::planning_interface::MoveGroupInterface dual_group(node_, "dual_arms");
    auto current_state = dual_group.getCurrentState(5.0);

    if (!current_state)
      throw std::runtime_error("FAST PICK: khong doc duoc current state");

    moveit::planning_interface::PlanningSceneInterface psi;
    std::map<std::string, moveit_msgs::msg::CollisionObject> objects;

    for (int i = 0; i < 40; ++i) {
      objects = psi.getObjects();
      if (objects.count("table") &&
          objects.count(left.object_name) &&
          objects.count(right.object_name))
        break;
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (!objects.count("table") ||
        !objects.count(left.object_name) ||
        !objects.count(right.object_name))
      throw std::runtime_error("FAST PICK: PlanningScene thieu table/object");

    const auto poses = psi.getObjectPoses({left.object_name, right.object_name});

    if (!poses.count(left.object_name) || !poses.count(right.object_name))
      throw std::runtime_error("FAST PICK: khong doc duoc pose object");

    const Eigen::Isometry3d world_left_object =
        poseToEigen(poses.at(left.object_name));
    const Eigen::Isometry3d world_right_object =
        poseToEigen(poses.at(right.object_name));

    RCLCPP_INFO(
        LOGGER,
        "FAST PICK [%s,%s] objects: LEFT(%.3f %.3f %.3f) RIGHT(%.3f %.3f %.3f)",
        left_object_name.c_str(), right_object_name.c_str(),
        world_left_object.translation().x(),
        world_left_object.translation().y(),
        world_left_object.translation().z(),
        world_right_object.translation().x(),
        world_right_object.translation().y(),
        world_right_object.translation().z());

    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    scene->setCurrentState(*current_state);

    for (const auto& kv : objects)
      scene->processCollisionObjectMsg(kv.second);

    const auto* left_jmg =
        robot_model->getJointModelGroup(left.arm_group);
    const auto* right_jmg =
        robot_model->getJointModelGroup(right.arm_group);
    const auto* dual_jmg =
        robot_model->getJointModelGroup("dual_arms");
    const auto* left_hand_jmg =
        robot_model->getJointModelGroup(left.hand_group);
    const auto* right_hand_jmg =
        robot_model->getJointModelGroup(right.hand_group);

    if (!left_jmg || !right_jmg || !dual_jmg ||
        !left_hand_jmg || !right_hand_jmg)
      throw std::runtime_error("FAST PICK: thieu JointModelGroup");

    left_hand_links =
        left_hand_jmg->getLinkModelNamesWithCollisionGeometry();
    right_hand_links =
        right_hand_jmg->getLinkModelNamesWithCollisionGeometry();

    auto& acm = scene->getAllowedCollisionMatrixNonConst();
    acm.setEntry(left.object_name, left_hand_links, true);
    acm.setEntry(right.object_name, right_hand_links, true);
    acm.setEntry(left_hand_links, left_hand_links, true);
    acm.setEntry(right_hand_links, right_hand_links, true);

    constexpr double APPROACH_DISTANCE = 0.07;
    constexpr double LIFT_DISTANCE = 0.10;

    // TOP-DOWN grasp: sweep the complete 0..350 deg yaw range.
    // The gripper approach axis stays vertical; only wrist yaw changes.
    std::vector<int> angle_order;
    angle_order.reserve(36);
    for (int idx = 0; idx < 36; ++idx)
      angle_order.push_back(idx);

    auto findLimitedCandidates =
        [&](const ArmConfig& cfg,
            const Eigen::Isometry3d& world_object,
            const moveit::core::RobotState& seed,
            std::size_t max_valid) {
          std::vector<Candidate> out;

          for (std::size_t order_i = 0;
               order_i < angle_order.size() && out.size() < max_valid;
               ++order_i) {
            const int idx = angle_order[order_i];
            const double angle = idx * M_PI / 18.0;

            const bool is_left =
                cfg.arm_group == "left_ur_onrobot_manipulator";
            const bool is_right_high_stack =
                cfg.arm_group == "right_ur_onrobot_manipulator" &&
                cfg.object_name == "stack_D";

            RCLCPP_INFO(
                LOGGER,
                "FAST PICK %s: try yaw %.0f deg (%zu/%zu)%s%s",
                cfg.arm_group.c_str(),
                angle * 180.0 / M_PI,
                order_i + 1,
                angle_order.size(),
                is_left ? " LEFT_FIXED_TILT_X=+10deg" : "",
                is_right_high_stack ? " RIGHT_L3_FIXED_TILT_X=-10deg" : "");

            // Keep the old fast yaw search. Only the rigid grasp orientation
            // needed by the high stack levels is changed:
            // LEFT keeps +10 deg (proved for L2), while RIGHT/stack_D uses
            // the mirrored -10 deg tilt so the wrist has a different IK
            // family at CENTER L3. RIGHT/stack_C for L1 stays unchanged.
            Eigen::Isometry3d world_hand_grasp;
            if (is_left)
              world_hand_grasp = leftStackHandPose(world_object, angle);
            else if (is_right_high_stack)
              world_hand_grasp = rightHighStackHandPose(world_object, angle);
            else
              world_hand_grasp = topDownHandPose(world_object, angle);

            // local +Z points down, therefore local -Z is straight upward
            const Eigen::Isometry3d world_hand_pregrasp =
                world_hand_grasp *
                Eigen::Translation3d(0.0, 0.0, -APPROACH_DISTANCE);

            moveit::core::RobotState grasp(robot_model);
            if (!solveIKCollisionFree(
                    scene, cfg, world_hand_grasp, seed, grasp, 4))
              continue;

            moveit::core::RobotState pregrasp(robot_model);
            if (!solveIKCollisionFree(
                    scene, cfg, world_hand_pregrasp, grasp, pregrasp, 4))
              continue;

            if (!validateApproach(
                    scene, cfg, world_hand_grasp, pregrasp))
              continue;

            Candidate c(robot_model);
            c.pregrasp = pregrasp;
            c.grasp = grasp;
            c.angle = angle;
            out.push_back(c);

            RCLCPP_INFO(
                LOGGER,
                "TOP-DOWN PICK %s: valid yaw %.0f deg",
                cfg.arm_group.c_str(),
                angle * 180.0 / M_PI);
          }

          return out;
        };

    RCLCPP_INFO(LOGGER, "FAST PICK: searching LEFT candidates...");
    auto left_candidates =
        findLimitedCandidates(
            left, world_left_object, *current_state, 8);

    RCLCPP_INFO(
        LOGGER,
        "FAST PICK: LEFT valid candidates=%zu",
        left_candidates.size());

    RCLCPP_INFO(LOGGER, "FAST PICK: searching RIGHT candidates...");
    auto right_candidates =
        findLimitedCandidates(
            right, world_right_object, *current_state, 8);

    RCLCPP_INFO(
        LOGGER,
        "FAST PICK: RIGHT valid candidates=%zu",
        right_candidates.size());

    if (left_candidates.empty() || right_candidates.empty())
      throw std::runtime_error(
          "FAST PICK: khong tim duoc grasp candidate");

    std::vector<double> left_current_q;
    std::vector<double> right_current_q;
    current_state->copyJointGroupPositions(left_jmg, left_current_q);
    current_state->copyJointGroupPositions(right_jmg, right_current_q);

    struct RankedPair
    {
      std::size_t li;
      std::size_t ri;
      double score;
    };

    std::vector<RankedPair> pairs;

    for (std::size_t li = 0; li < left_candidates.size(); ++li) {
      std::vector<double> lq;
      left_candidates[li].pregrasp.copyJointGroupPositions(left_jmg, lq);
      normalizeGroupNearState(
          robot_model, left_jmg, *current_state, lq);

      for (std::size_t ri = 0; ri < right_candidates.size(); ++ri) {
        std::vector<double> rq;
        right_candidates[ri].pregrasp.copyJointGroupPositions(right_jmg, rq);
        normalizeGroupNearState(
            robot_model, right_jmg, *current_state, rq);

        double max_delta = 0.0;
        const double score =
            jointTravelScore(
                lq, rq,
                left_current_q, right_current_q,
                max_delta);

        pairs.push_back({li, ri, score});
      }
    }

    std::sort(
        pairs.begin(), pairs.end(),
        [](const RankedPair& a, const RankedPair& b) {
          return a.score < b.score;
        });

    moveit::core::RobotState best_pre(*current_state);
    moveit::core::RobotState best_grasp(*current_state);
    moveit::core::RobotState best_lift(*current_state);

    bool found = false;
    double best_score = 0.0;
    double best_left_angle = 0.0;
    double best_right_angle = 0.0;

    for (std::size_t p = 0; p < pairs.size(); ++p) {
      const auto& lp = left_candidates[pairs[p].li];
      const auto& rp = right_candidates[pairs[p].ri];

      RCLCPP_INFO(
          LOGGER,
          "FAST PICK pair %zu/%zu: LEFT=%.0f RIGHT=%.0f score=%.3f",
          p + 1, pairs.size(),
          lp.angle * 180.0 / M_PI,
          rp.angle * 180.0 / M_PI,
          pairs[p].score);

      std::vector<double> left_pre_q;
      std::vector<double> right_pre_q;
      lp.pregrasp.copyJointGroupPositions(left_jmg, left_pre_q);
      rp.pregrasp.copyJointGroupPositions(right_jmg, right_pre_q);

      normalizeGroupNearState(
          robot_model, left_jmg, *current_state, left_pre_q);
      normalizeGroupNearState(
          robot_model, right_jmg, *current_state, right_pre_q);

      moveit::core::RobotState pre(*current_state);
      pre.setJointGroupPositions(left_jmg, left_pre_q);
      pre.setJointGroupPositions(right_jmg, right_pre_q);
      pre.update();

      if (!pre.satisfiesBounds(dual_jmg) ||
          scene->isStateColliding(pre, "", false))
        continue;

      std::vector<double> left_grasp_q;
      std::vector<double> right_grasp_q;
      lp.grasp.copyJointGroupPositions(left_jmg, left_grasp_q);
      rp.grasp.copyJointGroupPositions(right_jmg, right_grasp_q);

      normalizeGroupNearState(
          robot_model, left_jmg, pre, left_grasp_q);
      normalizeGroupNearState(
          robot_model, right_jmg, pre, right_grasp_q);

      moveit::core::RobotState grasp(pre);
      grasp.setJointGroupPositions(left_jmg, left_grasp_q);
      grasp.setJointGroupPositions(right_jmg, right_grasp_q);
      grasp.update();

      if (!grasp.satisfiesBounds(dual_jmg) ||
          scene->isStateColliding(grasp, "", false))
        continue;

      if (!validateDualInterpolation(
              scene, pre, grasp, dual_jmg, 20))
        continue;

      Eigen::Isometry3d world_left_hand_lift =
          leftStackHandPose(world_left_object, lp.angle);

      Eigen::Isometry3d world_right_hand_lift =
          (right_object_name == "stack_D")
              ? rightHighStackHandPose(world_right_object, rp.angle)
              : topDownHandPose(world_right_object, rp.angle);

      world_left_hand_lift.translation().z() += LIFT_DISTANCE;
      world_right_hand_lift.translation().z() += LIFT_DISTANCE;

      moveit::core::RobotState left_lift(robot_model);
      moveit::core::RobotState right_lift(robot_model);

      if (!solveIKCollisionFree(
              scene, left,
              world_left_hand_lift,
              grasp, left_lift, 10))
        continue;

      if (!solveIKCollisionFree(
              scene, right,
              world_right_hand_lift,
              grasp, right_lift, 10))
        continue;

      std::vector<double> left_lift_q;
      std::vector<double> right_lift_q;

      left_lift.copyJointGroupPositions(
          left_jmg, left_lift_q);
      right_lift.copyJointGroupPositions(
          right_jmg, right_lift_q);

      normalizeGroupNearState(
          robot_model, left_jmg, grasp, left_lift_q);
      normalizeGroupNearState(
          robot_model, right_jmg, grasp, right_lift_q);

      moveit::core::RobotState lift(grasp);
      lift.setJointGroupPositions(left_jmg, left_lift_q);
      lift.setJointGroupPositions(right_jmg, right_lift_q);
      lift.update();

      if (!lift.satisfiesBounds(dual_jmg) ||
          scene->isStateColliding(lift, "", false))
        continue;

      if (!validateDualInterpolation(
              scene, grasp, lift, dual_jmg, 20))
        continue;

      best_pre = pre;
      best_grasp = grasp;
      best_lift = lift;
      best_score = pairs[p].score;
      best_left_angle = lp.angle;
      best_right_angle = rp.angle;
      found = true;

      RCLCPP_INFO(
          LOGGER,
          "TOP-DOWN PICK ACCEPTED: LEFT yaw=%.0f RIGHT yaw=%.0f score=%.3f",
          best_left_angle * 180.0 / M_PI,
          best_right_angle * 180.0 / M_PI,
          best_score);

      break;
    }

    if (!found)
      throw std::runtime_error(
          "FAST PICK: khong tim duoc dual pair PREGRASP->GRASP->LIFT");

    DualTaskTargets targets;
    targets.pregrasp = groupGoal(best_pre, dual_jmg);
    targets.grasp = groupGoal(best_grasp, dual_jmg);
    targets.lift = groupGoal(best_lift, dual_jmg);
    targets.home = groupGoal(*current_state, dual_jmg);
    targets.left_angle = best_left_angle;
    targets.right_angle = best_right_angle;
    targets.score = best_score;

    return targets;
  }

  DualTaskTargets preparePickTargetsFast(
      const moveit::core::RobotModelConstPtr& robot_model,
      std::vector<std::string>& left_hand_links,
      std::vector<std::string>& right_hand_links)
  {
    const ArmConfig left{
        "left_ur_onrobot_manipulator",
        "left_ur_onrobot_gripper",
        "left_gripper_tcp",
        "left_object"};

    const ArmConfig right{
        "right_ur_onrobot_manipulator",
        "right_ur_onrobot_gripper",
        "right_gripper_tcp",
        "right_object"};

    moveit::planning_interface::MoveGroupInterface dual_group(node_, "dual_arms");
    auto current_state = dual_group.getCurrentState(5.0);

    if (!current_state)
      throw std::runtime_error("FAST PICK: khong doc duoc current state");

    moveit::planning_interface::PlanningSceneInterface psi;
    std::map<std::string, moveit_msgs::msg::CollisionObject> objects;

    for (int i = 0; i < 40; ++i) {
      objects = psi.getObjects({"table", left.object_name, right.object_name});
      if (objects.count("table") &&
          objects.count(left.object_name) &&
          objects.count(right.object_name))
        break;
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (!objects.count("table") ||
        !objects.count(left.object_name) ||
        !objects.count(right.object_name))
      throw std::runtime_error("FAST PICK: PlanningScene thieu table/object");

    const auto poses = psi.getObjectPoses({left.object_name, right.object_name});

    if (!poses.count(left.object_name) || !poses.count(right.object_name))
      throw std::runtime_error("FAST PICK: khong doc duoc pose object");

    const Eigen::Isometry3d world_left_object =
        poseToEigen(poses.at(left.object_name));
    const Eigen::Isometry3d world_right_object =
        poseToEigen(poses.at(right.object_name));

    RCLCPP_INFO(
        LOGGER,
        "FAST PICK objects: LEFT(%.3f %.3f %.3f) RIGHT(%.3f %.3f %.3f)",
        world_left_object.translation().x(),
        world_left_object.translation().y(),
        world_left_object.translation().z(),
        world_right_object.translation().x(),
        world_right_object.translation().y(),
        world_right_object.translation().z());

    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    scene->setCurrentState(*current_state);

    for (const auto& kv : objects)
      scene->processCollisionObjectMsg(kv.second);

    const auto* left_jmg =
        robot_model->getJointModelGroup(left.arm_group);
    const auto* right_jmg =
        robot_model->getJointModelGroup(right.arm_group);
    const auto* dual_jmg =
        robot_model->getJointModelGroup("dual_arms");
    const auto* left_hand_jmg =
        robot_model->getJointModelGroup(left.hand_group);
    const auto* right_hand_jmg =
        robot_model->getJointModelGroup(right.hand_group);

    if (!left_jmg || !right_jmg || !dual_jmg ||
        !left_hand_jmg || !right_hand_jmg)
      throw std::runtime_error("FAST PICK: thieu JointModelGroup");

    left_hand_links =
        left_hand_jmg->getLinkModelNamesWithCollisionGeometry();
    right_hand_links =
        right_hand_jmg->getLinkModelNamesWithCollisionGeometry();

    auto& acm = scene->getAllowedCollisionMatrixNonConst();
    acm.setEntry(left.object_name, left_hand_links, true);
    acm.setEntry(right.object_name, right_hand_links, true);
    acm.setEntry(left_hand_links, left_hand_links, true);
    acm.setEntry(right_hand_links, right_hand_links, true);

    const Eigen::Isometry3d hand_tf = graspFrameTransform();
    constexpr double APPROACH_DISTANCE = 0.05;
    constexpr double LIFT_DISTANCE = 0.08;

    // Search heuristic only: try around 260 deg first because it is good
    // for the current symmetric scene, but ALWAYS falls back around 360 deg.
    // Therefore 260 deg is not a fixed grasp requirement.
    std::vector<int> angle_order;
    angle_order.reserve(36);
    const int seed_idx = 26;  // 260 deg search seed only

    angle_order.push_back(seed_idx);
    for (int d = 1; d < 36; ++d) {
      const int plus = (seed_idx + d) % 36;
      const int minus = (seed_idx - d + 36) % 36;

      if (std::find(angle_order.begin(), angle_order.end(), plus) ==
          angle_order.end())
        angle_order.push_back(plus);

      if (angle_order.size() >= 36)
        break;

      if (std::find(angle_order.begin(), angle_order.end(), minus) ==
          angle_order.end())
        angle_order.push_back(minus);

      if (angle_order.size() >= 36)
        break;
    }

    auto findLimitedCandidates =
        [&](const ArmConfig& cfg,
            const Eigen::Isometry3d& world_object,
            const moveit::core::RobotState& seed,
            std::size_t max_valid) {
          std::vector<Candidate> out;

          for (std::size_t order_i = 0;
               order_i < angle_order.size() && out.size() < max_valid;
               ++order_i) {
            const int idx = angle_order[order_i];
            const double angle = idx * M_PI / 18.0;

            RCLCPP_INFO(
                LOGGER,
                "FAST PICK %s: try angle %.0f deg (%zu/%zu)",
                cfg.arm_group.c_str(),
                angle * 180.0 / M_PI,
                order_i + 1,
                angle_order.size());

            const Eigen::Isometry3d object_grasp =
                Eigen::Isometry3d(
                    Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));

            const Eigen::Isometry3d world_hand_grasp =
                world_object * object_grasp * hand_tf.inverse();

            const Eigen::Isometry3d world_hand_pregrasp =
                world_hand_grasp *
                Eigen::Translation3d(0.0, 0.0, -APPROACH_DISTANCE);

            moveit::core::RobotState grasp(robot_model);
            if (!solveIKCollisionFree(
                    scene, cfg, world_hand_grasp, seed, grasp, 6))
              continue;

            moveit::core::RobotState pregrasp(robot_model);
            if (!solveIKCollisionFree(
                    scene, cfg, world_hand_pregrasp, grasp, pregrasp, 6))
              continue;

            if (!validateApproach(
                    scene, cfg, world_hand_grasp, pregrasp))
              continue;

            Candidate c(robot_model);
            c.pregrasp = pregrasp;
            c.grasp = grasp;
            c.angle = angle;
            out.push_back(c);

            RCLCPP_INFO(
                LOGGER,
                "FAST PICK %s: valid angle %.0f deg",
                cfg.arm_group.c_str(),
                angle * 180.0 / M_PI);
          }

          return out;
        };

    RCLCPP_INFO(LOGGER, "FAST PICK: searching LEFT candidates...");
    auto left_candidates =
        findLimitedCandidates(
            left, world_left_object, *current_state, 4);

    RCLCPP_INFO(
        LOGGER,
        "FAST PICK: LEFT valid candidates=%zu",
        left_candidates.size());

    RCLCPP_INFO(LOGGER, "FAST PICK: searching RIGHT candidates...");
    auto right_candidates =
        findLimitedCandidates(
            right, world_right_object, *current_state, 4);

    RCLCPP_INFO(
        LOGGER,
        "FAST PICK: RIGHT valid candidates=%zu",
        right_candidates.size());

    if (left_candidates.empty() || right_candidates.empty())
      throw std::runtime_error(
          "FAST PICK: khong tim duoc grasp candidate");

    std::vector<double> left_current_q;
    std::vector<double> right_current_q;
    current_state->copyJointGroupPositions(left_jmg, left_current_q);
    current_state->copyJointGroupPositions(right_jmg, right_current_q);

    struct RankedPair
    {
      std::size_t li;
      std::size_t ri;
      double score;
    };

    std::vector<RankedPair> pairs;

    for (std::size_t li = 0; li < left_candidates.size(); ++li) {
      std::vector<double> lq;
      left_candidates[li].pregrasp.copyJointGroupPositions(left_jmg, lq);
      normalizeGroupNearState(
          robot_model, left_jmg, *current_state, lq);

      for (std::size_t ri = 0; ri < right_candidates.size(); ++ri) {
        std::vector<double> rq;
        right_candidates[ri].pregrasp.copyJointGroupPositions(right_jmg, rq);
        normalizeGroupNearState(
            robot_model, right_jmg, *current_state, rq);

        double max_delta = 0.0;
        const double score =
            jointTravelScore(
                lq, rq,
                left_current_q, right_current_q,
                max_delta);

        pairs.push_back({li, ri, score});
      }
    }

    std::sort(
        pairs.begin(), pairs.end(),
        [](const RankedPair& a, const RankedPair& b) {
          return a.score < b.score;
        });

    moveit::core::RobotState best_pre(*current_state);
    moveit::core::RobotState best_grasp(*current_state);
    moveit::core::RobotState best_lift(*current_state);

    bool found = false;
    double best_score = 0.0;
    double best_left_angle = 0.0;
    double best_right_angle = 0.0;

    for (std::size_t p = 0; p < pairs.size(); ++p) {
      const auto& lp = left_candidates[pairs[p].li];
      const auto& rp = right_candidates[pairs[p].ri];

      RCLCPP_INFO(
          LOGGER,
          "FAST PICK pair %zu/%zu: LEFT=%.0f RIGHT=%.0f score=%.3f",
          p + 1, pairs.size(),
          lp.angle * 180.0 / M_PI,
          rp.angle * 180.0 / M_PI,
          pairs[p].score);

      std::vector<double> left_pre_q;
      std::vector<double> right_pre_q;
      lp.pregrasp.copyJointGroupPositions(left_jmg, left_pre_q);
      rp.pregrasp.copyJointGroupPositions(right_jmg, right_pre_q);

      normalizeGroupNearState(
          robot_model, left_jmg, *current_state, left_pre_q);
      normalizeGroupNearState(
          robot_model, right_jmg, *current_state, right_pre_q);

      moveit::core::RobotState pre(*current_state);
      pre.setJointGroupPositions(left_jmg, left_pre_q);
      pre.setJointGroupPositions(right_jmg, right_pre_q);
      pre.update();

      if (!pre.satisfiesBounds(dual_jmg) ||
          scene->isStateColliding(pre, "", false))
        continue;

      std::vector<double> left_grasp_q;
      std::vector<double> right_grasp_q;
      lp.grasp.copyJointGroupPositions(left_jmg, left_grasp_q);
      rp.grasp.copyJointGroupPositions(right_jmg, right_grasp_q);

      normalizeGroupNearState(
          robot_model, left_jmg, pre, left_grasp_q);
      normalizeGroupNearState(
          robot_model, right_jmg, pre, right_grasp_q);

      moveit::core::RobotState grasp(pre);
      grasp.setJointGroupPositions(left_jmg, left_grasp_q);
      grasp.setJointGroupPositions(right_jmg, right_grasp_q);
      grasp.update();

      if (!grasp.satisfiesBounds(dual_jmg) ||
          scene->isStateColliding(grasp, "", false))
        continue;

      if (!validateDualInterpolation(
              scene, pre, grasp, dual_jmg, 20))
        continue;

      const Eigen::Isometry3d left_object_grasp =
          Eigen::Isometry3d(
              Eigen::AngleAxisd(
                  lp.angle, Eigen::Vector3d::UnitZ()));

      const Eigen::Isometry3d right_object_grasp =
          Eigen::Isometry3d(
              Eigen::AngleAxisd(
                  rp.angle, Eigen::Vector3d::UnitZ()));

      Eigen::Isometry3d world_left_hand_lift =
          world_left_object *
          left_object_grasp *
          hand_tf.inverse();

      Eigen::Isometry3d world_right_hand_lift =
          world_right_object *
          right_object_grasp *
          hand_tf.inverse();

      world_left_hand_lift.translation().z() += LIFT_DISTANCE;
      world_right_hand_lift.translation().z() += LIFT_DISTANCE;

      moveit::core::RobotState left_lift(robot_model);
      moveit::core::RobotState right_lift(robot_model);

      if (!solveIKCollisionFree(
              scene, left,
              world_left_hand_lift,
              grasp, left_lift, 10))
        continue;

      if (!solveIKCollisionFree(
              scene, right,
              world_right_hand_lift,
              grasp, right_lift, 10))
        continue;

      std::vector<double> left_lift_q;
      std::vector<double> right_lift_q;

      left_lift.copyJointGroupPositions(
          left_jmg, left_lift_q);
      right_lift.copyJointGroupPositions(
          right_jmg, right_lift_q);

      normalizeGroupNearState(
          robot_model, left_jmg, grasp, left_lift_q);
      normalizeGroupNearState(
          robot_model, right_jmg, grasp, right_lift_q);

      moveit::core::RobotState lift(grasp);
      lift.setJointGroupPositions(left_jmg, left_lift_q);
      lift.setJointGroupPositions(right_jmg, right_lift_q);
      lift.update();

      if (!lift.satisfiesBounds(dual_jmg) ||
          scene->isStateColliding(lift, "", false))
        continue;

      if (!validateDualInterpolation(
              scene, grasp, lift, dual_jmg, 20))
        continue;

      best_pre = pre;
      best_grasp = grasp;
      best_lift = lift;
      best_score = pairs[p].score;
      best_left_angle = lp.angle;
      best_right_angle = rp.angle;
      found = true;

      RCLCPP_INFO(
          LOGGER,
          "FAST PICK ACCEPTED: LEFT=%.0f RIGHT=%.0f score=%.3f",
          best_left_angle * 180.0 / M_PI,
          best_right_angle * 180.0 / M_PI,
          best_score);

      break;
    }

    if (!found)
      throw std::runtime_error(
          "FAST PICK: khong tim duoc dual pair PREGRASP->GRASP->LIFT");

    DualTaskTargets targets;
    targets.pregrasp = groupGoal(best_pre, dual_jmg);
    targets.grasp = groupGoal(best_grasp, dual_jmg);
    targets.lift = groupGoal(best_lift, dual_jmg);
    targets.home = groupGoal(*current_state, dual_jmg);
    targets.left_angle = best_left_angle;
    targets.right_angle = best_right_angle;
    targets.score = best_score;

    return targets;
  }

  DualTaskTargets prepareTargets(
      const moveit::core::RobotModelConstPtr& robot_model,
      std::vector<std::string>& left_hand_links,
      std::vector<std::string>& right_hand_links)
  {
    const ArmConfig left{
        "left_ur_onrobot_manipulator",
        "left_ur_onrobot_gripper",
        "left_gripper_tcp",
        "left_object"};

    const ArmConfig right{
        "right_ur_onrobot_manipulator",
        "right_ur_onrobot_gripper",
        "right_gripper_tcp",
        "right_object"};

    moveit::planning_interface::MoveGroupInterface dual_group(node_, "dual_arms");
    auto current_state = dual_group.getCurrentState(5.0);

    if (!current_state)
      throw std::runtime_error("Khong doc duoc current state");

    moveit::planning_interface::PlanningSceneInterface psi;
    std::map<std::string, moveit_msgs::msg::CollisionObject> objects;

    for (int i = 0; i < 40; ++i) {
      objects = psi.getObjects({"table", left.object_name, right.object_name});

      if (objects.count("table") &&
          objects.count(left.object_name) &&
          objects.count(right.object_name))
        break;

      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (!objects.count("table") ||
        !objects.count(left.object_name) ||
        !objects.count(right.object_name))
      throw std::runtime_error("PlanningScene thieu table/object");

    const auto poses = psi.getObjectPoses({left.object_name, right.object_name});

    if (!poses.count(left.object_name) || !poses.count(right.object_name))
      throw std::runtime_error("Khong doc duoc world pose cua 2 object");

    const Eigen::Isometry3d world_left_object = poseToEigen(poses.at(left.object_name));
    const Eigen::Isometry3d world_right_object = poseToEigen(poses.at(right.object_name));

    RCLCPP_INFO(
        LOGGER,
        "Objects: LEFT(%.3f %.3f %.3f) RIGHT(%.3f %.3f %.3f)",
        world_left_object.translation().x(),
        world_left_object.translation().y(),
        world_left_object.translation().z(),
        world_right_object.translation().x(),
        world_right_object.translation().y(),
        world_right_object.translation().z());

    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    scene->setCurrentState(*current_state);

    for (const auto& kv : objects)
      scene->processCollisionObjectMsg(kv.second);

    const auto* left_jmg = robot_model->getJointModelGroup(left.arm_group);
    const auto* right_jmg = robot_model->getJointModelGroup(right.arm_group);
    const auto* dual_jmg = robot_model->getJointModelGroup("dual_arms");
    const auto* left_hand_jmg = robot_model->getJointModelGroup(left.hand_group);
    const auto* right_hand_jmg = robot_model->getJointModelGroup(right.hand_group);

    if (!left_jmg || !right_jmg || !dual_jmg || !left_hand_jmg || !right_hand_jmg)
      throw std::runtime_error("Thieu JointModelGroup");

    left_hand_links = left_hand_jmg->getLinkModelNamesWithCollisionGeometry();
    right_hand_links = right_hand_jmg->getLinkModelNamesWithCollisionGeometry();

    auto& acm = scene->getAllowedCollisionMatrixNonConst();
    acm.setEntry(left.object_name, left_hand_links, true);
    acm.setEntry(right.object_name, right_hand_links, true);
    acm.setEntry(left_hand_links, left_hand_links, true);
    acm.setEntry(right_hand_links, right_hand_links, true);

    auto left_candidates =
        findCandidates(scene, left, world_left_object, *current_state);
    auto right_candidates =
        findCandidates(scene, right, world_right_object, *current_state);

    RCLCPP_INFO(
        LOGGER,
        "Grasp candidates: LEFT=%zu RIGHT=%zu",
        left_candidates.size(),
        right_candidates.size());

    if (left_candidates.empty() || right_candidates.empty())
      throw std::runtime_error("Khong tim duoc grasp candidate");

    std::vector<double> left_current_q;
    std::vector<double> right_current_q;
    current_state->copyJointGroupPositions(left_jmg, left_current_q);
    current_state->copyJointGroupPositions(right_jmg, right_current_q);

    // Rank tung candidate bang quang duong joint re: CURRENT -> PREGRASP -> GRASP.
    // Chi giu TOP_K moi tay de khong lam no so cap full-cycle.
    constexpr std::size_t TOP_K_PER_ARM = 12;

    auto rankArmCandidates = [&](std::vector<Candidate>& candidates,
                                 const moveit::core::JointModelGroup* jmg,
                                 const std::vector<double>& current_q) {
      for (auto& c : candidates) {
        std::vector<double> pre_q;
        std::vector<double> grasp_q;
        c.pregrasp.copyJointGroupPositions(jmg, pre_q);
        normalizeGroupNearState(robot_model, jmg, *current_state, pre_q);

        moveit::core::RobotState pre(*current_state);
        pre.setJointGroupPositions(jmg, pre_q);
        pre.update();

        c.grasp.copyJointGroupPositions(jmg, grasp_q);
        normalizeGroupNearState(robot_model, jmg, pre, grasp_q);

        double sum = 0.0;
        double max_delta = 0.0;
        for (std::size_t i = 0; i < pre_q.size(); ++i) {
          const double d0 = std::abs(pre_q[i] - current_q[i]);
          const double d1 = std::abs(grasp_q[i] - pre_q[i]);
          sum += d0 + d1;
          max_delta = std::max(max_delta, d0);
        }
        c.rank_score = sum + 2.0 * max_delta;
      }

      std::sort(candidates.begin(), candidates.end(),
                [](const Candidate& a, const Candidate& b) {
                  return a.rank_score < b.rank_score;
                });

      if (candidates.size() > TOP_K_PER_ARM)
        candidates.erase(candidates.begin() + TOP_K_PER_ARM, candidates.end());
    };

    rankArmCandidates(left_candidates, left_jmg, left_current_q);
    rankArmCandidates(right_candidates, right_jmg, right_current_q);

    RCLCPP_INFO(LOGGER,
                "Ranked candidates kept: LEFT=%zu RIGHT=%zu; best LEFT=%.0f deg cost=%.3f; best RIGHT=%.0f deg cost=%.3f",
                left_candidates.size(), right_candidates.size(),
                left_candidates.front().angle * 180.0 / M_PI, left_candidates.front().rank_score,
                right_candidates.front().angle * 180.0 / M_PI, right_candidates.front().rank_score);

    const Eigen::Isometry3d hand_tf = graspFrameTransform();

    const double left_place_x = node_->get_parameter("left_place_x").as_double();
    const double left_place_y = node_->get_parameter("left_place_y").as_double();
    const double left_place_z = node_->get_parameter("left_place_z").as_double();

    const double right_place_x = node_->get_parameter("right_place_x").as_double();
    const double right_place_y = node_->get_parameter("right_place_y").as_double();
    const double right_place_z = node_->get_parameter("right_place_z").as_double();

    const double left_roll =
        node_->get_parameter("left_place_roll_deg").as_double() * M_PI / 180.0;
    const double left_pitch =
        node_->get_parameter("left_place_pitch_deg").as_double() * M_PI / 180.0;
    const double left_yaw =
        node_->get_parameter("left_place_yaw_deg").as_double() * M_PI / 180.0;

    const double right_roll =
        node_->get_parameter("right_place_roll_deg").as_double() * M_PI / 180.0;
    const double right_pitch =
        node_->get_parameter("right_place_pitch_deg").as_double() * M_PI / 180.0;
    const double right_yaw =
        node_->get_parameter("right_place_yaw_deg").as_double() * M_PI / 180.0;

    Eigen::Isometry3d world_left_place_object = world_left_object;
    Eigen::Isometry3d world_right_place_object = world_right_object;

    world_left_place_object.translation() =
        Eigen::Vector3d(left_place_x, left_place_y, left_place_z);
    world_right_place_object.translation() =
        Eigen::Vector3d(right_place_x, right_place_y, right_place_z);

    const Eigen::Matrix3d left_offset =
        (Eigen::AngleAxisd(left_yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(left_pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(left_roll, Eigen::Vector3d::UnitX())).toRotationMatrix();

    const Eigen::Matrix3d right_offset =
        (Eigen::AngleAxisd(right_yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(right_pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(right_roll, Eigen::Vector3d::UnitX())).toRotationMatrix();

    world_left_place_object.linear() =
        left_offset * world_left_object.linear();
    world_right_place_object.linear() =
        right_offset * world_right_object.linear();

    RCLCPP_INFO(
        LOGGER,
        "Requested PLACE: LEFT(%.3f %.3f %.3f) RPYoff=(%.1f %.1f %.1f deg) "
        "RIGHT(%.3f %.3f %.3f) RPYoff=(%.1f %.1f %.1f deg)",
        left_place_x, left_place_y, left_place_z,
        left_roll * 180.0 / M_PI,
        left_pitch * 180.0 / M_PI,
        left_yaw * 180.0 / M_PI,
        right_place_x, right_place_y, right_place_z,
        right_roll * 180.0 / M_PI,
        right_pitch * 180.0 / M_PI,
        right_yaw * 180.0 / M_PI);

    const double place_distance =
        std::sqrt(
            std::pow(left_place_x - right_place_x, 2.0) +
            std::pow(left_place_y - right_place_y, 2.0) +
            std::pow(left_place_z - right_place_z, 2.0));

    RCLCPP_INFO(LOGGER, "Distance between place targets: %.3f m", place_distance);

    if (place_distance < 0.06) {
      RCLCPP_WARN(
          LOGGER,
          "Hai vi tri place rat gan nhau (%.3f m). Planner co the reject do object/gripper collision.",
          place_distance);
    }

    moveit::core::RobotState best_pre(*current_state);
    moveit::core::RobotState best_grasp(*current_state);
    moveit::core::RobotState best_lift(*current_state);
    moveit::core::RobotState best_preplace(*current_state);
    moveit::core::RobotState best_place(*current_state);

    bool found_full_pair = false;
    double best_score = std::numeric_limits<double>::infinity();
    double best_max_delta = 0.0;
    double left_angle = 0.0;
    double right_angle = 0.0;

    int pair_count = 0;
    int grasp_ok = 0;
    int lift_ok = 0;
    int preplace_ok = 0;
    int place_ok = 0;

    auto stateTravel = [&](const moveit::core::RobotState& a,
                           const moveit::core::RobotState& b) {
      std::vector<double> qa;
      std::vector<double> qb;
      a.copyJointGroupPositions(dual_jmg, qa);
      b.copyJointGroupPositions(dual_jmg, qb);

      double sum = 0.0;
      for (std::size_t i = 0; i < qa.size(); ++i)
        sum += std::abs(qb[i] - qa[i]);

      return sum;
    };

    struct RankedPair
    {
      std::size_t left_idx;
      std::size_t right_idx;
      double quick_score;
    };

    std::vector<RankedPair> ranked_pairs;
    ranked_pairs.reserve(left_candidates.size() * right_candidates.size());

    for (std::size_t li = 0; li < left_candidates.size(); ++li) {
      for (std::size_t ri = 0; ri < right_candidates.size(); ++ri) {
        ranked_pairs.push_back(
            RankedPair{li, ri,
                       left_candidates[li].rank_score +
                       right_candidates[ri].rank_score});
      }
    }

    std::sort(ranked_pairs.begin(), ranked_pairs.end(),
              [](const RankedPair& a, const RankedPair& b) {
                return a.quick_score < b.quick_score;
              });

    RCLCPP_INFO(LOGGER,
                "Ranked %zu dual pairs by cheap joint cost. Testing FULL cycle from cheapest...",
                ranked_pairs.size());

    for (const auto& rp : ranked_pairs) {
      const auto& lc = left_candidates[rp.left_idx];
      const auto& rc = right_candidates[rp.right_idx];
      ++pair_count;

      RCLCPP_INFO(LOGGER,
                  "FULL test %d/%zu: LEFT=%.0f RIGHT=%.0f quick_cost=%.3f",
                  pair_count, ranked_pairs.size(),
                  lc.angle * 180.0 / M_PI,
                  rc.angle * 180.0 / M_PI,
                  rp.quick_score);

      std::vector<double> left_pre_q;
      std::vector<double> right_pre_q;
      lc.pregrasp.copyJointGroupPositions(left_jmg, left_pre_q);
      rc.pregrasp.copyJointGroupPositions(right_jmg, right_pre_q);
      normalizeGroupNearState(robot_model, left_jmg, *current_state, left_pre_q);
      normalizeGroupNearState(robot_model, right_jmg, *current_state, right_pre_q);

      moveit::core::RobotState pre(*current_state);
      pre.setJointGroupPositions(left_jmg, left_pre_q);
      pre.setJointGroupPositions(right_jmg, right_pre_q);
      pre.update();

      if (!pre.satisfiesBounds(dual_jmg) ||
          scene->isStateColliding(pre, "", false))
        continue;

      std::vector<double> left_grasp_q;
      std::vector<double> right_grasp_q;
      lc.grasp.copyJointGroupPositions(left_jmg, left_grasp_q);
      rc.grasp.copyJointGroupPositions(right_jmg, right_grasp_q);
      normalizeGroupNearState(robot_model, left_jmg, pre, left_grasp_q);
      normalizeGroupNearState(robot_model, right_jmg, pre, right_grasp_q);

      moveit::core::RobotState grasp(pre);
      grasp.setJointGroupPositions(left_jmg, left_grasp_q);
      grasp.setJointGroupPositions(right_jmg, right_grasp_q);
      grasp.update();

      if (!grasp.satisfiesBounds(dual_jmg) ||
          scene->isStateColliding(grasp, "", false))
        continue;

      if (!validateDualInterpolation(scene, pre, grasp, dual_jmg))
        continue;

      ++grasp_ok;

      const Eigen::Isometry3d left_object_grasp =
          Eigen::Isometry3d(
              Eigen::AngleAxisd(lc.angle, Eigen::Vector3d::UnitZ()));
      const Eigen::Isometry3d right_object_grasp =
          Eigen::Isometry3d(
              Eigen::AngleAxisd(rc.angle, Eigen::Vector3d::UnitZ()));

      const Eigen::Isometry3d world_left_hand_grasp =
          world_left_object * left_object_grasp * hand_tf.inverse();
      const Eigen::Isometry3d world_right_hand_grasp =
          world_right_object * right_object_grasp * hand_tf.inverse();

      Eigen::Isometry3d world_left_hand_lift = world_left_hand_grasp;
      Eigen::Isometry3d world_right_hand_lift = world_right_hand_grasp;
      world_left_hand_lift.translation().z() += 0.08;
      world_right_hand_lift.translation().z() += 0.08;

      moveit::core::RobotState left_lift(robot_model);
      moveit::core::RobotState right_lift(robot_model);

      if (!solveIKCollisionFree(
              scene, left, world_left_hand_lift, grasp, left_lift, 25))
        continue;
      if (!solveIKCollisionFree(
              scene, right, world_right_hand_lift, grasp, right_lift, 25))
        continue;

      std::vector<double> left_lift_q;
      std::vector<double> right_lift_q;
      left_lift.copyJointGroupPositions(left_jmg, left_lift_q);
      right_lift.copyJointGroupPositions(right_jmg, right_lift_q);
      normalizeGroupNearState(robot_model, left_jmg, grasp, left_lift_q);
      normalizeGroupNearState(robot_model, right_jmg, grasp, right_lift_q);

      moveit::core::RobotState combined_lift(grasp);
      combined_lift.setJointGroupPositions(left_jmg, left_lift_q);
      combined_lift.setJointGroupPositions(right_jmg, right_lift_q);
      combined_lift.update();

      if (!combined_lift.satisfiesBounds(dual_jmg) ||
          scene->isStateColliding(combined_lift, "", false))
        continue;
      if (!validateDualInterpolation(scene, grasp, combined_lift, dual_jmg, 30))
        continue;

      ++lift_ok;

      const Eigen::Isometry3d world_left_hand_place =
          world_left_place_object * left_object_grasp * hand_tf.inverse();
      const Eigen::Isometry3d world_right_hand_place =
          world_right_place_object * right_object_grasp * hand_tf.inverse();

      Eigen::Isometry3d world_left_hand_preplace = world_left_hand_place;
      Eigen::Isometry3d world_right_hand_preplace = world_right_hand_place;
      world_left_hand_preplace.translation().z() += 0.08;
      world_right_hand_preplace.translation().z() += 0.08;

      moveit::core::RobotState left_preplace(robot_model);
      moveit::core::RobotState right_preplace(robot_model);

      if (!solveIKCollisionFree(
              scene, left, world_left_hand_preplace,
              combined_lift, left_preplace, 20))
        continue;
      if (!solveIKCollisionFree(
              scene, right, world_right_hand_preplace,
              combined_lift, right_preplace, 20))
        continue;

      std::vector<double> left_preplace_q;
      std::vector<double> right_preplace_q;
      left_preplace.copyJointGroupPositions(left_jmg, left_preplace_q);
      right_preplace.copyJointGroupPositions(right_jmg, right_preplace_q);
      normalizeGroupNearState(robot_model, left_jmg, combined_lift, left_preplace_q);
      normalizeGroupNearState(robot_model, right_jmg, combined_lift, right_preplace_q);

      moveit::core::RobotState combined_preplace(combined_lift);
      combined_preplace.setJointGroupPositions(left_jmg, left_preplace_q);
      combined_preplace.setJointGroupPositions(right_jmg, right_preplace_q);
      combined_preplace.update();

      if (!combined_preplace.satisfiesBounds(dual_jmg) ||
          scene->isStateColliding(combined_preplace, "", false))
        continue;

      ++preplace_ok;

      moveit::core::RobotState left_place(robot_model);
      moveit::core::RobotState right_place(robot_model);

      if (!solveIKCollisionFree(
              scene, left, world_left_hand_place,
              combined_preplace, left_place, 20))
        continue;
      if (!solveIKCollisionFree(
              scene, right, world_right_hand_place,
              combined_preplace, right_place, 20))
        continue;

      std::vector<double> left_place_q;
      std::vector<double> right_place_q;
      left_place.copyJointGroupPositions(left_jmg, left_place_q);
      right_place.copyJointGroupPositions(right_jmg, right_place_q);
      normalizeGroupNearState(robot_model, left_jmg, combined_preplace, left_place_q);
      normalizeGroupNearState(robot_model, right_jmg, combined_preplace, right_place_q);

      moveit::core::RobotState combined_place(combined_preplace);
      combined_place.setJointGroupPositions(left_jmg, left_place_q);
      combined_place.setJointGroupPositions(right_jmg, right_place_q);

      if (!combined_place.setToDefaultValues(left_hand_jmg, "open") ||
          !combined_place.setToDefaultValues(right_hand_jmg, "open"))
        throw std::runtime_error("Khong tim thay named state open cua gripper");

      combined_place.update();

      if (!combined_place.satisfiesBounds(dual_jmg) ||
          scene->isStateColliding(combined_place, "", false))
        continue;
      if (!validateDualInterpolation(
              scene, combined_preplace, combined_place, dual_jmg, 30))
        continue;

      ++place_ok;

      double max_delta = 0.0;
      const double pick_score = jointTravelScore(
          left_pre_q, right_pre_q,
          left_current_q, right_current_q,
          max_delta);

      const double full_score =
          pick_score +
          stateTravel(pre, grasp) +
          stateTravel(grasp, combined_lift) +
          stateTravel(combined_lift, combined_preplace) +
          stateTravel(combined_preplace, combined_place);

      best_score = full_score;
      best_max_delta = max_delta;
      best_pre = pre;
      best_grasp = grasp;
      best_lift = combined_lift;
      best_preplace = combined_preplace;
      best_place = combined_place;
      left_angle = lc.angle;
      right_angle = rc.angle;
      found_full_pair = true;

      RCLCPP_INFO(LOGGER,
                  "RANKED FULL pair ACCEPTED: LEFT=%.0f RIGHT=%.0f quick=%.3f full=%.3f max_delta=%.3f",
                  lc.angle * 180.0 / M_PI,
                  rc.angle * 180.0 / M_PI,
                  rp.quick_score,
                  full_score,
                  max_delta);
      break;
    }

    RCLCPP_INFO(
        LOGGER,
        "Pair search: total=%d grasp_ok=%d lift_ok=%d preplace_ok=%d place_ok=%d",
        pair_count,
        grasp_ok,
        lift_ok,
        preplace_ok,
        place_ok);

    if (!found_full_pair)
      throw std::runtime_error(
          "Khong tim duoc cap grasp nao di duoc tron PICK -> LIFT -> PLACE");

    RCLCPP_INFO(
        LOGGER,
        "Selected RANKED FULL-CYCLE pair: LEFT=%.0f RIGHT=%.0f score=%.3f max_delta=%.3f",
        left_angle * 180.0 / M_PI,
        right_angle * 180.0 / M_PI,
        best_score,
        best_max_delta);

    RCLCPP_INFO(
        LOGGER,
        "Release check: grippers OPEN collision-free; object-table contact allowed only during PLACE");

    DualTaskTargets targets;
    targets.pregrasp = groupGoal(best_pre, dual_jmg);
    targets.grasp = groupGoal(best_grasp, dual_jmg);
    targets.lift = groupGoal(best_lift, dual_jmg);
    targets.preplace = groupGoal(best_preplace, dual_jmg);
    targets.place = groupGoal(best_place, dual_jmg);
    targets.home = groupGoal(*current_state, dual_jmg);
    targets.left_angle = left_angle;
    targets.right_angle = right_angle;
    targets.score = best_score;
    return targets;
  }

  mtc::Task createDualPickDemoTask()
  {
    mtc::Task task;
    task.stages()->setName("dual_pick_parallel_cycle");
    task.loadRobotModel(node_);

    std::vector<std::string> left_hand_links;
    std::vector<std::string> right_hand_links;

    // Reuse the already proven object-aware target generator.
    // It returns synchronized 12DOF pregrasp/grasp/lift states.
    const auto targets =
        preparePickTargetsFast(task.getRobotModel(), left_hand_links, right_hand_links);

    auto sampling =
        std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    sampling->setProperty("planning_pipeline", "ompl");
    sampling->setPlannerId("RRTConnectkConfigDefault");

    auto interpolation =
        std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

    {
      auto allow =
          std::make_unique<mtc::stages::ModifyPlanningScene>(
              "allow grasp collisions");
      allow->allowCollisions("left_object", left_hand_links, true);
      allow->allowCollisions("right_object", right_hand_links, true);
      allow->allowCollisions(left_hand_links, left_hand_links, true);
      allow->allowCollisions(right_hand_links, right_hand_links, true);
      task.add(std::move(allow));
    }

    {
      auto open =
          std::make_unique<mtc::stages::MoveTo>(
              "open both grippers", interpolation);
      open->setGroup("dual_grippers");
      open->setGoal(std::map<std::string, double>{
          {"left_finger_width", 0.100},
          {"right_finger_width", 0.100}});
      task.add(std::move(open));
    }

    {
      auto move =
          std::make_unique<mtc::stages::MoveTo>(
              "P1 parallel move to dual pregrasp", sampling);
      move->setGroup("dual_arms");
      move->setGoal(targets.pregrasp);
      move->setTimeout(15.0);
      task.add(std::move(move));
    }

    {
      auto approach =
          std::make_unique<mtc::stages::MoveTo>(
              "P2 synchronized dual approach", interpolation);
      approach->setGroup("dual_arms");
      approach->setGoal(targets.grasp);
      task.add(std::move(approach));
    }

    {
      auto close =
          std::make_unique<mtc::stages::MoveTo>(
              "P3 close both grippers", interpolation);
      close->setGroup("dual_grippers");
      close->setGoal(std::map<std::string, double>{
          {"left_finger_width", 0.0},
          {"right_finger_width", 0.0}});
      task.add(std::move(close));
    }

    {
      auto attach_left =
          std::make_unique<mtc::stages::ModifyPlanningScene>(
              "P4 attach left object");
      attach_left->attachObject("left_object", "left_gripper_tcp");
      task.add(std::move(attach_left));

      auto attach_right =
          std::make_unique<mtc::stages::ModifyPlanningScene>(
              "P4 attach right object");
      attach_right->attachObject("right_object", "right_gripper_tcp");
      task.add(std::move(attach_right));
    }

    {
      auto lift =
          std::make_unique<mtc::stages::MoveTo>(
              "P5 synchronized dual lift", interpolation);
      lift->setGroup("dual_arms");
      lift->setGoal(targets.lift);
      task.add(std::move(lift));
    }

    // Put-back portion keeps the demo repeatable and leaves the scene clean.
    {
      auto allow_table =
          std::make_unique<mtc::stages::ModifyPlanningScene>(
              "allow objects contact table for put-back");
      allow_table->allowCollisions("left_object", "table", true);
      allow_table->allowCollisions("right_object", "table", true);
      task.add(std::move(allow_table));
    }

    {
      auto lower =
          std::make_unique<mtc::stages::MoveTo>(
              "P6 synchronized put-back", interpolation);
      lower->setGroup("dual_arms");
      lower->setGoal(targets.grasp);
      task.add(std::move(lower));
    }

    {
      auto open =
          std::make_unique<mtc::stages::MoveTo>(
              "P7 open both grippers", interpolation);
      open->setGroup("dual_grippers");
      open->setGoal(std::map<std::string, double>{
          {"left_finger_width", 0.100},
          {"right_finger_width", 0.100}});
      task.add(std::move(open));
    }

    {
      auto detach_left =
          std::make_unique<mtc::stages::ModifyPlanningScene>(
              "P8 detach left object");
      detach_left->detachObject("left_object", "left_gripper_tcp");
      task.add(std::move(detach_left));

      auto detach_right =
          std::make_unique<mtc::stages::ModifyPlanningScene>(
              "P8 detach right object");
      detach_right->detachObject("right_object", "right_gripper_tcp");
      task.add(std::move(detach_right));
    }

    {
      auto restore =
          std::make_unique<mtc::stages::ModifyPlanningScene>(
              "restore collision rules");
      restore->allowCollisions("left_object", left_hand_links, false);
      restore->allowCollisions("right_object", right_hand_links, false);
      restore->allowCollisions("left_object", "table", false);
      restore->allowCollisions("right_object", "table", false);
      task.add(std::move(restore));
    }

    {
      auto retreat =
          std::make_unique<mtc::stages::MoveTo>(
              "P9 synchronized retreat", interpolation);
      retreat->setGroup("dual_arms");
      retreat->setGoal(targets.pregrasp);
      task.add(std::move(retreat));
    }

    {
      auto home =
          std::make_unique<mtc::stages::MoveTo>(
              "P10 parallel return home", sampling);
      home->setGroup("dual_arms");
      home->setGoal(targets.home);
      home->setTimeout(15.0);
      task.add(std::move(home));
    }

    return task;
  }

  mtc::Task createTask()
  {
    mtc::Task task;
    task.stages()->setName("dual_arms_12dof_full_pick_place");
    task.loadRobotModel(node_);

    std::vector<std::string> left_hand_links;
    std::vector<std::string> right_hand_links;

    const auto targets =
        prepareTargets(task.getRobotModel(), left_hand_links, right_hand_links);

    auto sampling = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    sampling->setProperty("planning_pipeline", "ompl");
    sampling->setPlannerId("RRTConnectkConfigDefault");

    auto interpolation =
        std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

    {
      auto allow =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow grasp collisions");
      allow->allowCollisions("left_object", left_hand_links, true);
      allow->allowCollisions("right_object", right_hand_links, true);
      allow->allowCollisions(left_hand_links, left_hand_links, true);
      allow->allowCollisions(right_hand_links, right_hand_links, true);
      task.add(std::move(allow));
    }

    {
      auto open = std::make_unique<mtc::stages::MoveTo>(
          "open both grippers", interpolation);
      open->setGroup("dual_grippers");
      open->setGoal(std::map<std::string, double>{
          {"left_finger_width", 0.100},
          {"right_finger_width", 0.100}});
      task.add(std::move(open));
    }

    {
      auto move = std::make_unique<mtc::stages::MoveTo>(
          "dual move to pre-grasp", sampling);
      move->setGroup("dual_arms");
      move->setGoal(targets.pregrasp);
      move->setTimeout(15.0);
      task.add(std::move(move));
    }

    {
      auto approach = std::make_unique<mtc::stages::MoveTo>(
          "dual synchronized approach to grasp", interpolation);
      approach->setGroup("dual_arms");
      approach->setGoal(targets.grasp);
      task.add(std::move(approach));
    }

    {
      auto close = std::make_unique<mtc::stages::MoveTo>(
          "close both grippers", interpolation);
      close->setGroup("dual_grippers");
      close->setGoal(std::map<std::string, double>{
          {"left_finger_width", 0.0},
          {"right_finger_width", 0.0}});
      task.add(std::move(close));
    }

    {
      auto attach_left =
          std::make_unique<mtc::stages::ModifyPlanningScene>("attach left object");
      attach_left->attachObject("left_object", "left_gripper_tcp");
      task.add(std::move(attach_left));

      auto attach_right =
          std::make_unique<mtc::stages::ModifyPlanningScene>("attach right object");
      attach_right->attachObject("right_object", "right_gripper_tcp");
      task.add(std::move(attach_right));
    }

    {
      auto lift = std::make_unique<mtc::stages::MoveTo>(
          "dual synchronized lift", interpolation);
      lift->setGroup("dual_arms");
      lift->setGoal(targets.lift);
      task.add(std::move(lift));
    }

    {
      // Khi dat vat len ban, attached object duoc phep tiep xuc voi mat ban.
      // Chi allow object <-> table, KHONG allow arm/gripper <-> table.
      auto allow_place_contact =
          std::make_unique<mtc::stages::ModifyPlanningScene>(
              "allow objects contact table at place");
      allow_place_contact->allowCollisions("left_object", "table", true);
      allow_place_contact->allowCollisions("right_object", "table", true);
      task.add(std::move(allow_place_contact));
    }

    {
      auto move = std::make_unique<mtc::stages::MoveTo>(
          "dual move to pre-place", sampling);
      move->setGroup("dual_arms");
      move->setGoal(targets.preplace);
      move->setTimeout(15.0);
      task.add(std::move(move));
    }

    {
      auto descend = std::make_unique<mtc::stages::MoveTo>(
          "dual synchronized descend to place", interpolation);
      descend->setGroup("dual_arms");
      descend->setGoal(targets.place);
      task.add(std::move(descend));
    }

    {
      auto open = std::make_unique<mtc::stages::MoveTo>(
          "open both grippers at place", interpolation);
      open->setGroup("dual_grippers");
      open->setGoal(std::map<std::string, double>{
          {"left_finger_width", 0.100},
          {"right_finger_width", 0.100}});
      task.add(std::move(open));
    }

    {
      auto detach_left =
          std::make_unique<mtc::stages::ModifyPlanningScene>(
              "detach left object");
      detach_left->detachObject("left_object", "left_gripper_tcp");
      task.add(std::move(detach_left));

      auto detach_right =
          std::make_unique<mtc::stages::ModifyPlanningScene>(
              "detach right object");
      detach_right->detachObject("right_object", "right_gripper_tcp");
      task.add(std::move(detach_right));
    }

    {
      auto forbid =
          std::make_unique<mtc::stages::ModifyPlanningScene>(
              "restore object collision rules");
      forbid->allowCollisions("left_object", left_hand_links, false);
      forbid->allowCollisions("right_object", right_hand_links, false);
      forbid->allowCollisions("left_object", "table", false);
      forbid->allowCollisions("right_object", "table", false);
      task.add(std::move(forbid));
    }

    {
      auto retreat = std::make_unique<mtc::stages::MoveTo>(
          "dual synchronized retreat", interpolation);
      retreat->setGroup("dual_arms");
      retreat->setGoal(targets.preplace);
      task.add(std::move(retreat));
    }

    {
      auto home = std::make_unique<mtc::stages::MoveTo>(
          "dual return home", sampling);
      home->setGroup("dual_arms");
      home->setGoal(targets.home);
      home->setTimeout(15.0);
      task.add(std::move(home));
    }

    return task;
  }

public:
  rclcpp::Node::SharedPtr apiNode() const
  {
    return node_;
  }

  bool apiInitializeScene()
  {
    try {
      stack_column_yaw_ = std::numeric_limits<double>::quiet_NaN();
      if (node_->get_parameter("command_initialize_stack_scene").as_bool())
        stack_table_top_ = setupStack4Scene();
      else
        stack_table_top_ = node_->get_parameter("stack_table_top").as_double();
      return true;
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(LOGGER, "initializeScene failed: %s", e.what());
      return false;
    }
  }

  bool apiResetScene()
  {
    if (left_held_.valid || right_held_.valid) {
      RCLCPP_ERROR(LOGGER, "reset_scene refused: an object is currently held");
      return false;
    }
    try {
      stack_column_yaw_ = std::numeric_limits<double>::quiet_NaN();
      stack_table_top_ = setupStack4Scene();
      return true;
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(LOGGER, "reset_scene failed: %s", e.what());
      return false;
    }
  }

  void apiPrintStatus()
  {
    printSystemStatus();
  }

  bool apiExecuteEnabled() const
  {
    return node_->get_parameter("execute").as_bool();
  }

  ArmScore apiEvaluateArm(const std::string& object_name, ArmSide arm)
  {
    const auto eval = evaluateArmForObject(normalizeObjectName(object_name), arm);
    ArmScore out;
    out.arm = eval.arm;
    out.ik_ok = eval.ik_ok;
    out.collision_free = eval.collision_free;
    out.approach_ok = eval.approach_ok;
    out.plan_ok = eval.plan_ok;
    out.reachable = eval.reachable;
    out.cartesian_distance = eval.cartesian_distance;
    out.joint_cost = eval.joint_cost;
    out.max_joint_delta = eval.max_joint_delta;
    out.total_cost = eval.total_cost;
    out.grasp_yaw = eval.grasp_yaw;
    return out;
  }

  ArmSide apiSelectBestArm(const std::string& object_name)
  {
    const std::string object = normalizeObjectName(object_name);
    const auto left = evaluateArmForObject(object, ArmSide::LEFT);
    const auto right = evaluateArmForObject(object, ArmSide::RIGHT);
    return selectBestArm(left, right);
  }

  bool apiPick(const std::string& object_name, ArmSide requested_arm)
  {
    return pickObject(normalizeObjectName(object_name), requested_arm);
  }

  bool apiPlace(const std::string& object_name,
                double x, double y, double z, double yaw_rad)
  {
    return placeObject(normalizeObjectName(object_name), x, y, z, yaw_rad);
  }

  bool apiMove(const std::string& object_name,
               double x, double y, double z, double yaw_rad)
  {
    return moveObject(normalizeObjectName(object_name), x, y, z, yaw_rad);
  }

  bool apiGetObjectPose(const std::string& object_name,
                        double& x, double& y, double& z)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    const std::string object = normalizeObjectName(object_name);
    if (!getObjectWorldPose(object, pose))
      return false;

    x = pose.translation().x();
    y = pose.translation().y();
    z = pose.translation().z();
    return true;
  }

  bool apiStack(const std::vector<std::string>& objects,
                double center_x, double center_y)
  {
    std::vector<std::string> normalized;
    normalized.reserve(objects.size());
    for (const auto& object : objects)
      normalized.push_back(normalizeObjectName(object));
    return stackObjects(normalized, center_x, center_y);
  }

  bool apiSwap(const std::string& object_a, const std::string& object_b)
  {
    return swapObjects(normalizeObjectName(object_a), normalizeObjectName(object_b));
  }

  double stack_table_top_ = -0.003;
  double stack_column_yaw_ = std::numeric_limits<double>::quiet_NaN();

  HeldObject left_held_;
  HeldObject right_held_;

  std::deque<std::string> command_queue_;
  std::mutex command_queue_mutex_;
  std::condition_variable command_cv_;
  std::thread command_worker_;
  bool command_worker_stop_ = false;
  std::atomic<bool> command_server_ready_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;
};


DualArmInterface::DualArmInterface(const rclcpp::NodeOptions& options)
  : impl_(std::make_unique<Impl>(options))
{
}

DualArmInterface::~DualArmInterface() = default;

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
DualArmInterface::getNodeBaseInterface()
{
  return impl_->getNodeBaseInterface();
}

rclcpp::Node::SharedPtr DualArmInterface::node() const
{
  return impl_->apiNode();
}

bool DualArmInterface::initializeScene()
{
  return impl_->apiInitializeScene();
}

bool DualArmInterface::resetScene()
{
  return impl_->apiResetScene();
}

void DualArmInterface::printStatus()
{
  impl_->apiPrintStatus();
}

bool DualArmInterface::executeEnabled() const
{
  return impl_->apiExecuteEnabled();
}

ArmScore DualArmInterface::evaluateArm(const std::string& object_name, ArmSide arm)
{
  return impl_->apiEvaluateArm(object_name, arm);
}

ArmSide DualArmInterface::selectBestArm(const std::string& object_name)
{
  return impl_->apiSelectBestArm(object_name);
}

bool DualArmInterface::pick(const std::string& object_name, ArmSide requested_arm)
{
  return impl_->apiPick(object_name, requested_arm);
}

bool DualArmInterface::place(const std::string& object_name,
                             double x, double y, double z,
                             double yaw_rad)
{
  return impl_->apiPlace(object_name, x, y, z, yaw_rad);
}

bool DualArmInterface::move(const std::string& object_name,
                            double x, double y, double z,
                            double yaw_rad)
{
  return impl_->apiMove(object_name, x, y, z, yaw_rad);
}

bool DualArmInterface::getObjectPose(const std::string& object_name,
                                     double& x, double& y, double& z)
{
  return impl_->apiGetObjectPose(object_name, x, y, z);
}

bool DualArmInterface::stack(const std::vector<std::string>& objects,
                             double center_x, double center_y)
{
  return impl_->apiStack(objects, center_x, center_y);
}

bool DualArmInterface::swap(const std::string& object_a, const std::string& object_b)
{
  return impl_->apiSwap(object_a, object_b);
}

std::string DualArmInterface::normalizeObjectName(const std::string& name)
{
  if (name == "A" || name == "a") return "stack_A";
  if (name == "B" || name == "b") return "stack_B";
  if (name == "C" || name == "c") return "stack_C";
  if (name == "D" || name == "d") return "stack_D";
  return name;
}

ArmSide DualArmInterface::parseArmSide(const std::string& text)
{
  if (text == "left" || text == "LEFT" || text == "l") return ArmSide::LEFT;
  if (text == "right" || text == "RIGHT" || text == "r") return ArmSide::RIGHT;
  if (text == "auto" || text == "AUTO" || text.empty()) return ArmSide::AUTO;
  return ArmSide::NONE;
}

const char* DualArmInterface::armSideName(ArmSide arm)
{
  switch (arm) {
    case ArmSide::LEFT: return "LEFT";
    case ArmSide::RIGHT: return "RIGHT";
    case ArmSide::AUTO: return "AUTO";
    default: return "NONE";
  }
}

}  // namespace ur_onrobot_mtc
