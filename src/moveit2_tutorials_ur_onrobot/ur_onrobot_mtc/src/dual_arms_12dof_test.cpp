#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <limits>
#include <algorithm>

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

#include <Eigen/Geometry>

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

  constexpr int PREFERRED_INDEX = 26;
  constexpr double APPROACH_DISTANCE = 0.05;

  for (int i = 0; i < 36; ++i) {
    const int idx = (PREFERRED_INDEX + i) % 36;
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

    if (out.size() >= 6)
      break;
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

class DualArmMTCNode
{
public:
  explicit DualArmMTCNode(const rclcpp::NodeOptions& options)
  {
    node_ = std::make_shared<rclcpp::Node>("dual_arm_mtc_node", options);
    if (!node_->has_parameter("execute"))
      node_->declare_parameter<bool>("execute", false);
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
  {
    return node_->get_node_base_interface();
  }

  void run()
  {
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

private:
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

    const auto left_candidates =
        findCandidates(scene, left, world_left_object, *current_state);
    const auto right_candidates =
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

    const Eigen::Isometry3d hand_tf = graspFrameTransform();

    Eigen::Isometry3d world_left_place_object = world_left_object;
    Eigen::Isometry3d world_right_place_object = world_right_object;
    // Hai de robot nam tai y = +/-0.50, nen dat vat xa khu vuc de.
    world_left_place_object.translation() = Eigen::Vector3d(-0.25, 0.20, 0.052);
    world_right_place_object.translation() = Eigen::Vector3d(-0.25, -0.20, 0.052);

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

    for (const auto& lc : left_candidates) {
      std::vector<double> left_pre_q;
      lc.pregrasp.copyJointGroupPositions(left_jmg, left_pre_q);
      normalizeGroupNearState(robot_model, left_jmg, *current_state, left_pre_q);

      for (const auto& rc : right_candidates) {
        ++pair_count;

        std::vector<double> right_pre_q;
        rc.pregrasp.copyJointGroupPositions(right_jmg, right_pre_q);
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
                scene, left, world_left_hand_lift, grasp, left_lift, 30))
          continue;

        if (!solveIKCollisionFree(
                scene, right, world_right_hand_lift, grasp, right_lift, 30))
          continue;

        std::vector<double> left_lift_q;
        std::vector<double> right_lift_q;
        left_lift.copyJointGroupPositions(left_jmg, left_lift_q);
        right_lift.copyJointGroupPositions(right_jmg, right_lift_q);

        normalizeGroupNearState(
            robot_model, left_jmg, grasp, left_lift_q);
        normalizeGroupNearState(
            robot_model, right_jmg, grasp, right_lift_q);

        moveit::core::RobotState combined_lift(grasp);
        combined_lift.setJointGroupPositions(left_jmg, left_lift_q);
        combined_lift.setJointGroupPositions(right_jmg, right_lift_q);
        combined_lift.update();

        if (!combined_lift.satisfiesBounds(dual_jmg) ||
            scene->isStateColliding(combined_lift, "", false))
          continue;

        if (!validateDualInterpolation(
                scene, grasp, combined_lift, dual_jmg, 30))
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
                combined_lift, left_preplace, 25))
          continue;

        if (!solveIKCollisionFree(
                scene, right, world_right_hand_preplace,
                combined_lift, right_preplace, 25))
          continue;

        std::vector<double> left_preplace_q;
        std::vector<double> right_preplace_q;
        left_preplace.copyJointGroupPositions(left_jmg, left_preplace_q);
        right_preplace.copyJointGroupPositions(right_jmg, right_preplace_q);

        normalizeGroupNearState(
            robot_model, left_jmg, combined_lift, left_preplace_q);
        normalizeGroupNearState(
            robot_model, right_jmg, combined_lift, right_preplace_q);

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
                combined_preplace, left_place, 25))
          continue;

        if (!solveIKCollisionFree(
                scene, right, world_right_hand_place,
                combined_preplace, right_place, 25))
          continue;

        std::vector<double> left_place_q;
        std::vector<double> right_place_q;
        left_place.copyJointGroupPositions(left_jmg, left_place_q);
        right_place.copyJointGroupPositions(right_jmg, right_place_q);

        normalizeGroupNearState(
            robot_model, left_jmg, combined_preplace, left_place_q);
        normalizeGroupNearState(
            robot_model, right_jmg, combined_preplace, right_place_q);

        moveit::core::RobotState combined_place(combined_preplace);
        combined_place.setJointGroupPositions(left_jmg, left_place_q);
        combined_place.setJointGroupPositions(right_jmg, right_place_q);

        // Kiem tra dung state cua stage ke tiep: hai gripper deu mo.
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
        const double pick_score =
            jointTravelScore(
                left_pre_q,
                right_pre_q,
                left_current_q,
                right_current_q,
                max_delta);

        const double full_score =
            pick_score +
            stateTravel(pre, grasp) +
            stateTravel(grasp, combined_lift) +
            stateTravel(combined_lift, combined_preplace) +
            stateTravel(combined_preplace, combined_place);

        if (full_score < best_score) {
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

          RCLCPP_INFO(
              LOGGER,
              "Feasible FULL pair FOUND: LEFT=%.0f RIGHT=%.0f score=%.3f -> ACCEPT NOW",
              lc.angle * 180.0 / M_PI,
              rc.angle * 180.0 / M_PI,
              full_score);

          // Candidate list da uu tien 260 deg truoc.
          // Gap cap FULL kha thi dau tien thi dung ngay de tranh quet 36 cap IK rat lau.
          break;
        }
      }

      if (found_full_pair)
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
        "Selected FIRST FEASIBLE FULL-CYCLE pair: LEFT=%.0f RIGHT=%.0f score=%.3f max_delta=%.3f",
        left_angle * 180.0 / M_PI,
        right_angle * 180.0 / M_PI,
        best_score,
        best_max_delta);

    RCLCPP_INFO(
        LOGGER,
        "Place targets: LEFT(-0.25, 0.20, 0.052) RIGHT(-0.25, -0.20, 0.052)");

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

  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<DualArmMTCNode>(options);

  auto base = node->getNodeBaseInterface();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(base);

  std::thread spin_thread([&executor]() { executor.spin(); });

  std::this_thread::sleep_for(std::chrono::seconds(1));
  node->run();

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
