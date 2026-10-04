#include "ur_onrobot_mtc/tasks/solder_task.hpp"
#include "ur_onrobot_mtc/pcb_geometry.hpp"

#include <Eigen/Geometry>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>
#include <moveit_msgs/msg/planning_scene.hpp>

namespace ur_onrobot_mtc
{
namespace
{
using Group = moveit::planning_interface::MoveGroupInterface;
using Scene = moveit::planning_interface::PlanningSceneInterface;
using Transform = Eigen::Isometry3d;
using namespace std::chrono_literals;

Transform transform(const geometry_msgs::msg::Pose& p)
{
  Transform t = Transform::Identity();
  const Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (!std::isfinite(q.norm()) || q.norm() < 1e-9)
    throw std::runtime_error("Invalid scene quaternion");
  t.linear() = q.normalized().toRotationMatrix();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  return t;
}

geometry_msgs::msg::Pose pose(const Transform& t)
{
  geometry_msgs::msg::Pose p;
  const Eigen::Quaterniond q(t.linear());
  p.position.x = t.translation().x(); p.position.y = t.translation().y(); p.position.z = t.translation().z();
  p.orientation.x = q.x(); p.orientation.y = q.y(); p.orientation.z = q.z(); p.orientation.w = q.w();
  return p;
}

Eigen::Vector3d vector(const std::array<double, 3>& v) { return {v[0], v[1], v[2]}; }
std::string prefix(ArmSide arm) { return arm == ArmSide::LEFT ? "left_" : "right_"; }

// Only allow each object's own gripper to touch it during pickup/release.
// Restore individual pairs against the latest ACM, preserving other changes.
class GraspContacts
{
public:
  explicit GraspContacts(rclcpp::Node::SharedPtr node) : node_(std::move(node))
  {
    client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  }
  ~GraspContacts()
  {
    try { restore(); }
    catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "SOLDER collision-rule restore failed: %s", e.what());
    }
  }

  void allow(const std::string& object, const std::vector<std::string>& links)
  {
    auto matrix = read();
    for (const auto& link : links) {
      collision_detection::AllowedCollision::Type type;
      const bool present = matrix.getEntry(object, link, type);
      if (present && type == collision_detection::AllowedCollision::CONDITIONAL)
        throw std::runtime_error("Conditional collision rule cannot be overridden");
      saved_.push_back({object, link, present, present && type == collision_detection::AllowedCollision::ALWAYS});
      matrix.setEntry(object, link, true);
    }
    write(matrix);
  }

  void restore()
  {
    if (saved_.empty()) return;
    auto matrix = read();
    for (auto it = saved_.rbegin(); it != saved_.rend(); ++it) {
      if (it->present) matrix.setEntry(it->object, it->link, it->allowed);
      else matrix.removeEntry(it->object, it->link);
    }
    write(matrix);
    saved_.clear();
  }

private:
  collision_detection::AllowedCollisionMatrix read()
  {
    if (!client_->wait_for_service(3s)) throw std::runtime_error("PlanningScene service unavailable");
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = client_->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready)
      throw std::runtime_error("PlanningScene request timed out");
    return collision_detection::AllowedCollisionMatrix(future.get()->scene.allowed_collision_matrix);
  }
  void write(const collision_detection::AllowedCollisionMatrix& matrix)
  {
    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    matrix.getMessage(diff.allowed_collision_matrix);
    if (!scene_.applyPlanningScene(diff)) throw std::runtime_error("Cannot update grasp collision rules");
  }
  struct Saved { std::string object, link; bool present, allowed; };
  std::vector<Saved> saved_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr client_;
  Scene scene_;
};
}  // namespace

class SolderTask::Impl
{
public:
  explicit Impl(std::shared_ptr<DualArmInterface> robot) : robot_(std::move(robot)), node_(robot_->node())
  {
    scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if (!node_->has_parameter("solder.holder_arm")) node_->declare_parameter<std::string>("solder.holder_arm", "left");
    if (!node_->has_parameter("solder.dwell_seconds")) node_->declare_parameter<double>("solder.dwell_seconds", 1.5);
    if (!node_->has_parameter("solder.approach_height")) node_->declare_parameter<double>("solder.approach_height", 0.06);
    status_pub_ = node_->create_publisher<std_msgs::msg::String>("/solder_status", rclcpp::QoS(1).transient_local());
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/solder_process_markers", rclcpp::QoS(1).transient_local());
  }

  bool execute(int component)
  {
    try {
      if (component < 0 || component > 3) throw std::runtime_error("Use solder [1|2|3|all]");
      if (!robot_->executeEnabled()) throw std::runtime_error("SOLDER needs execute:=true on fake hardware");
      const ArmSide holder = DualArmInterface::parseArmSide(node_->get_parameter("solder.holder_arm").as_string());
      if (holder != ArmSide::LEFT && holder != ArmSide::RIGHT) throw std::runtime_error("Invalid solder.holder_arm");
      const ArmSide solderer = holder == ArmSide::LEFT ? ArmSide::RIGHT : ArmSide::LEFT;
      dwell_ = node_->get_parameter("solder.dwell_seconds").as_double();
      clearance_ = node_->get_parameter("solder.approach_height").as_double();
      if (!std::isfinite(dwell_) || dwell_ < 0.0 || dwell_ > 30.0 ||
          !std::isfinite(clearance_) || clearance_ < 0.02 || clearance_ > 0.15)
        throw std::runtime_error("Invalid dwell_seconds (0..30) or approach_height (0.02..0.15)");
      if (!scene_.getAttachedObjects().empty())
        throw std::runtime_error("Start SOLDER with both hands free; inspect status after a failed task");

      Group holder_group(node_, prefix(holder) + "ur_onrobot_manipulator");
      Group tool_group(node_, prefix(solderer) + "ur_onrobot_manipulator");
      configure(holder_group, holder); configure(tool_group, solderer);
      const auto model = holder_group.getRobotModel();
      const auto* holder_hand = model->getJointModelGroup(prefix(holder) + "ur_onrobot_gripper");
      const auto* tool_hand = model->getJointModelGroup(prefix(solderer) + "ur_onrobot_gripper");
      if (!holder_hand || !tool_hand) throw std::runtime_error("Missing gripper groups");

      std::vector<int> selected;
      for (int i = 1; i <= 3; ++i) if (component == 0 || component == i) selected.push_back(i);
      // Verify all required geometry before either arm moves.
      const Transform board = worldPose("pcb");
      const Transform tool_source = worldPose("solder_tool");
      requireBox("pcb", 0, {pcb::board_x, pcb::board_y, pcb::board_z});
      requireBox("solder_tool", 0, {pcb::handle_length, pcb::handle_width, pcb::handle_height});
      requireBox("solder_tool", 1, {pcb::shaft_length, pcb::shaft_width, pcb::shaft_width});
      requireBox("solder_tool", 2, {pcb::tip_length, pcb::tip_width, pcb::tip_width});
      // Capture both the position AND axes of the tip before PICK removes the
      // tool from the world. The scene may store upright or horizontal boxes.
      const auto tool_geometry = scene_.getObjects({"solder_tool"}).at("solder_tool");
      if (tool_geometry.primitive_poses.size() != tool_geometry.primitives.size())
        throw std::runtime_error("Missing solder_tool primitive poses");
      const Transform tool_to_tip = transform(tool_geometry.primitive_poses.at(2)) *
          Eigen::Translation3d(pcb::tip_length / 2.0, 0, 0);
      RCLCPP_INFO(node_->get_logger(), "SOLDER PCB centre=(%.4f %.4f %.4f), tool tip offset=(%.4f %.4f %.4f)",
                  board.translation().x(), board.translation().y(), board.translation().z(),
                  tool_to_tip.translation().x(), tool_to_tip.translation().y(), tool_to_tip.translation().z());
      for (int i : selected) {

        if (completed_.count(i)) throw std::runtime_error("Component already soldered in this session");
        requireBox("component_" + std::to_string(i), 0, {pcb::component_x, pcb::component_y, pcb::component_z});
      }

      GraspContacts tool_contacts(node_);
      tool_contacts.allow("solder_tool", tool_hand->getLinkModelNamesWithCollisionGeometry());
      status("PICK_TOOL");
      if (!robot_->pick("solder_tool", solderer)) throw std::runtime_error("Tool pickup failed");
      const Transform hand_to_tool = heldTransform("solder_tool", solderer);
      const Transform hand_to_tip = hand_to_tool * tool_to_tip;

      // Stage the soldering arm farther behind the board while the other arm
      // picks and positions the component. Keep the iron attached; the pad
      // planner will approach it from this clear staging pose afterward.
      Transform tool_standby = tool_source;
      tool_standby.translation() += Eigen::Vector3d(0.0, -0.12, 0.05);
      status("MOVE_TOOL_CLEAR");
      move(tool_group, tool_standby * hand_to_tool.inverse());

      // Use the component-arm waiting pose specified by the RViz joint target.
      const auto* holder_joints = model->getJointModelGroup(prefix(holder) + "ur_onrobot_manipulator");
      if (!holder_joints) throw std::runtime_error("Missing holder manipulator group");
      const std::string holder_joint_prefix = prefix(holder);
      const std::map<std::string, double> holder_wait_by_name{
          {holder_joint_prefix + "shoulder_pan_joint", -23.0 * M_PI / 180.0},
          {holder_joint_prefix + "shoulder_lift_joint", -45.0 * M_PI / 180.0},
          {holder_joint_prefix + "elbow_joint", 45.0 * M_PI / 180.0},
          {holder_joint_prefix + "wrist_1_joint", -90.0 * M_PI / 180.0},
          {holder_joint_prefix + "wrist_2_joint", -90.0 * M_PI / 180.0},
          {holder_joint_prefix + "wrist_3_joint", 0.0}};
      std::vector<double> holder_ready_positions;
      for (const auto& variable : holder_joints->getVariableNames()) {
        const auto it = holder_wait_by_name.find(variable);
        if (it == holder_wait_by_name.end())
          throw std::runtime_error("Missing component-arm wait angle for joint: " + variable);
        holder_ready_positions.push_back(it->second);
      }
      if (holder_ready_positions.size() != holder_wait_by_name.size())
        throw std::runtime_error("Component-arm wait pose does not match the manipulator joint group");

      for (int i : selected) {
        const std::string object = "component_" + std::to_string(i);
        GraspContacts component_contacts(node_);
        component_contacts.allow(object, holder_hand->getLinkModelNamesWithCollisionGeometry());
        status("PICK_COMPONENT " + std::to_string(i));
        if (!robot_->pick(object, holder)) throw std::runtime_error("Component pickup failed");
        const Transform hand_to_component = heldTransform(object, holder);
        const Transform target = board * Eigen::Translation3d(vector(pcb::componentOffset(i - 1)));
        status("TRANSFER_COMPONENT " + std::to_string(i));
        const Transform above = positionComponent(holder_group, target, hand_to_component,
                                                   board.linear().col(2), i);
        // Set the component down, release it into the world scene, and clear
        // the holder arm before testing whether the solder arm can reach pads.
        status("RELEASE_COMPONENT " + std::to_string(i));
        if (!robot_->releaseHeldObject(object)) throw std::runtime_error("Component release failed");
        status("RETREAT_HOLDER " + std::to_string(i));
        cartesian(holder_group, above * hand_to_component.inverse());
        // Raise the empty holder first, then move it sideways off the PCB.
        // A single pose goal 20 cm sideways and 12 cm upward inherited the
        // placement quaternion and was outside the arm's solvable IK region.
        // Try short Cartesian segments first; after release, position-only
        // planning can change the empty gripper's orientation if needed.
        Transform holder_park = above * hand_to_component.inverse();
        Transform holder_park_high = holder_park;
        holder_park_high.translation() += board.linear().col(2) * 0.05;
        status("PARK_HOLDER_RAISE " + std::to_string(i));
        cartesianOrPlan(holder_group, holder_park_high, "PARK_HOLDER_RAISE");

        const double board_side = holder == ArmSide::LEFT ? 1.0 : -1.0;
        holder_park = holder_park_high;
        holder_park.translation() += board.linear().col(1) * (board_side * 0.17);
        RCLCPP_INFO(node_->get_logger(),
                    "SOLDER holder park TCP=(%.4f %.4f %.4f)",
                    holder_park.translation().x(), holder_park.translation().y(),
                    holder_park.translation().z());
        status("PARK_HOLDER " + std::to_string(i));
        // The lateral park move does not need a straight-line path. If the
        // Cartesian interpolation fails, the fallback keeps this position
        // goal but lets the empty gripper choose a reachable orientation.
        // Pad contact and retreat retain strict Cartesian checks.
        cartesianOrPlan(holder_group, holder_park, "PARK_HOLDER");
        status("HOLDER_CLEAR " + std::to_string(i));
        // Keep only the gripper/component contact allowance while the released
        // part is still near the fingers; restore the normal collision rules
        // once the holder has cleared the PCB.
        component_contacts.restore();

        for (std::size_t pad = 0; pad < 2; ++pad) {
          solderPad(tool_group, board, hand_to_tip, i, pad, solderer);
        }

        completed_.insert(i);
        // Return the soldering arm to its clear standby pose, then move the
        // component arm to its configured wait pose for the next pick.
        status("RESET_HOLDER_READY " + std::to_string(i));
        move(tool_group, tool_standby * hand_to_tool.inverse());
        moveJointTarget(holder_group, holder_ready_positions, "next component pick");
      }

      status("RETURN_TOOL");
      Transform tool_above = tool_source;
      tool_above.translation().z() += clearance_;
      move(tool_group, tool_above * hand_to_tool.inverse());
      cartesian(tool_group, tool_source * hand_to_tool.inverse());
      if (!robot_->releaseHeldObject("solder_tool")) throw std::runtime_error("Tool release failed");
      cartesian(tool_group, tool_above * hand_to_tool.inverse());
      tool_contacts.restore();
      status("DONE");
      return true;
    } catch (const std::exception& e) {
      status(std::string("FAILED: ") + e.what());
      RCLCPP_ERROR(node_->get_logger(), "SOLDER stopped: %s. Inspect status and attached objects before recovery.", e.what());
      return false;
    }
  }

private:
  void configure(Group& group, ArmSide arm)
  {
    group.setEndEffectorLink(prefix(arm) + "gripper_tcp");
    group.setPoseReferenceFrame("world");
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPlanningTime(8.0);
    group.setNumPlanningAttempts(4);
    group.setMaxVelocityScalingFactor(0.10);
    group.setMaxAccelerationScalingFactor(0.10);
  }

  Transform worldPose(const std::string& object)
  {
    const auto objects = scene_.getObjects({object});
    const auto it = objects.find(object);
    if (it == objects.end() || it->second.header.frame_id != "world")
      throw std::runtime_error("Missing world-frame scene object: " + object);
    const auto poses = scene_.getObjectPoses({object});
    if (!poses.count(object)) throw std::runtime_error("Missing object pose: " + object);
    return transform(poses.at(object));
  }

  void requireBox(const std::string& id, std::size_t index, const std::array<double, 3>& size)
  {
    const auto objects = scene_.getObjects({id});
    if (!objects.count(id)) throw std::runtime_error("Run pcb_solder.launch.py first: missing " + id);
    const auto& primitives = objects.at(id).primitives;
    if (index >= primitives.size() || primitives[index].type != shape_msgs::msg::SolidPrimitive::BOX ||
        primitives[index].dimensions.size() != 3)
      throw std::runtime_error("Unexpected PCB scene geometry: " + id);
    for (std::size_t j = 0; j < 3; ++j)
      if (std::abs(primitives[index].dimensions[j] - size[j]) > 1e-6)
        throw std::runtime_error("Scene dimensions differ from shared PCB geometry: " + id);
  }


  Transform heldTransform(const std::string& object, ArmSide expected)
  {
    ArmSide actual;
    geometry_msgs::msg::Pose p;
    if (!robot_->heldObjectTransform(object, actual, p) || actual != expected)
      throw std::runtime_error("Object tracking mismatch: " + object);
    for (int attempt = 0; attempt < 20; ++attempt) {
      if (scene_.getAttachedObjects({object}).count(object)) return transform(p);
      rclcpp::sleep_for(100ms);
    }
    throw std::runtime_error("Object attachment not confirmed: " + object);
  }

  bool plan(Group& group, const Transform& target, Group::Plan& result)
  {
    group.setStartStateToCurrentState();
    group.setPoseTarget(pose(target));
    const auto code = group.plan(result);
    group.clearPoseTargets();
    if (code == moveit::core::MoveItErrorCode::SUCCESS) return true;
    const Eigen::Quaterniond q(target.linear());
    RCLCPP_WARN(node_->get_logger(),
                "SOLDER pose plan failed: group=%s code=%d tcp=(%.4f %.4f %.4f) q=(%.4f %.4f %.4f %.4f)",
                group.getName().c_str(), code.val,
                target.translation().x(), target.translation().y(), target.translation().z(),
                q.x(), q.y(), q.z(), q.w());
    // PICK plans to joint targets. Try the same strategy here when the pose
    // constraint sampler fails, using the complete scene including both tools.
    return planJointTarget(group, target, result, code.val);
  }

  planning_scene::PlanningScenePtr snapshot(Group& group)
  {
    if (!scene_client_->wait_for_service(3s)) throw std::runtime_error("PlanningScene service unavailable");
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    using C = moveit_msgs::msg::PlanningSceneComponents;
    request->components.components = C::SCENE_SETTINGS | C::ROBOT_STATE |
        C::ROBOT_STATE_ATTACHED_OBJECTS | C::WORLD_OBJECT_NAMES | C::WORLD_OBJECT_GEOMETRY |
        C::OCTOMAP | C::TRANSFORMS | C::ALLOWED_COLLISION_MATRIX | C::LINK_PADDING_AND_SCALING;
    auto future = scene_client_->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready)
      throw std::runtime_error("PlanningScene snapshot timed out");
    auto result = std::make_shared<planning_scene::PlanningScene>(group.getRobotModel());
    if (!result->setPlanningSceneMsg(future.get()->scene))
      throw std::runtime_error("Cannot load PlanningScene snapshot");
    return result;
  }

  void logContacts(const planning_scene::PlanningScenePtr& scene,
                   moveit::core::RobotState& state, const char* label)
  {
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 12;
    collision_detection::CollisionResult result;
    scene->checkCollision(request, result, state);
    for (const auto& contact : result.contacts)
      RCLCPP_WARN(node_->get_logger(), "SOLDER %s collision: %s <-> %s", label,
                  contact.first.first.c_str(), contact.first.second.c_str());
  }

  bool planJointTarget(Group& group, const Transform& target, Group::Plan& result, int pose_error)
  {
    auto scene = snapshot(group);
    const auto* joints = group.getRobotModel()->getJointModelGroup(group.getName());
    if (!joints) throw std::runtime_error("Missing arm joint group");
    moveit::core::RobotState start(scene->getCurrentState());
    start.update();
    const bool bounds_ok = start.satisfiesBounds(joints);
    const bool start_colliding = scene->isStateColliding(start, "", false);
    if (!bounds_ok || start_colliding) {
      RCLCPP_ERROR(node_->get_logger(), "SOLDER START valid_bounds=%d collision=%d", bounds_ok, start_colliding);
      logContacts(scene, start, "START");
      last_plan_error_ = "Invalid start state; see SOLDER START collision/bounds (pose code=" +
          std::to_string(pose_error) + ")";
      return false;
    }
    int ik_count = 0, valid_count = 0, planned_count = 0;
    int last_code = pose_error;
    bool logged_goal_contacts = false;
    // Only randomize the active arm: the other arm and both attached objects
    // remain in the snapshot throughout IK and full-robot collision checking.
    for (int attempt = 0; attempt < 16 && planned_count < 4; ++attempt) {
      moveit::core::RobotState candidate(start);
      if (attempt) candidate.setToRandomPositions(joints);
      candidate.update();
      if (!candidate.setFromIK(joints, target, group.getEndEffectorLink(), 0.15)) continue;
      ++ik_count;
      candidate.update();
      if (!candidate.satisfiesBounds(joints)) continue;
      if (scene->isStateColliding(candidate, "", false)) {
        if (!logged_goal_contacts) logContacts(scene, candidate, "GOAL");
        logged_goal_contacts = true;
        continue;
      }
      ++valid_count;
      std::vector<double> values;
      candidate.copyJointGroupPositions(joints, values);
      group.setStartStateToCurrentState();
      if (!group.setJointValueTarget(values)) continue;
      ++planned_count;
      const auto code = group.plan(result);
      last_code = code.val;
      if (code == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "SOLDER joint-target plan succeeded: group=%s attempt=%d",
                    group.getName().c_str(), attempt + 1);
        return true;
      }
    }
    std::ostringstream message;
    message << "group=" << group.getName() << " pose_code=" << pose_error
            << " IK=" << ik_count << "/16 valid_goals=" << valid_count
            << " joint_plans=" << planned_count << " last_code=" << last_code;
    last_plan_error_ = message.str();
    RCLCPP_ERROR(node_->get_logger(), "SOLDER PLAN DIAGNOSTIC: %s", last_plan_error_.c_str());
    return false;
  }

  Transform positionComponent(Group& group, const Transform& target,
                               const Transform& hand_to_component,
                               const Eigen::Vector3d& normal, int component)
  {
    const Transform contact = target * hand_to_component.inverse();
    RCLCPP_INFO(node_->get_logger(), "SOLDER component_%d target centre=(%.4f %.4f %.4f)",
                component, target.translation().x(), target.translation().y(), target.translation().z());
    // A raised board can make the 6 cm preplace pose harder to reach. Test
    // shorter vertical approaches too, without altering the component pose.
    double previous_height = -1.0;
    for (double height : {clearance_, 0.04, 0.02}) {
      if (height > clearance_ + 1e-6 || std::abs(height - previous_height) < 1e-6) continue;
      previous_height = height;
      Transform above = target;
      above.translation() += normal * height;
      Group::Plan transfer, descend;
      RCLCPP_INFO(node_->get_logger(), "SOLDER preplace clearance=%.3f", height);
      if (!plan(group, above * hand_to_component.inverse(), transfer)) continue;
      moveit::core::RobotState start(group.getRobotModel());
      start.setToDefaultValues();
      moveit::core::robotStateMsgToRobotState(transfer.start_state_, start);
      robot_trajectory::RobotTrajectory path(group.getRobotModel(), group.getName());
      path.setRobotTrajectoryMsg(start, transfer.trajectory_);
      if (!path.getWayPointCount() || !planCartesian(group, contact, path.getLastWayPoint(), descend)) {
        last_plan_error_ = "Preplace planned but Cartesian descent incomplete";
        continue;
      }
      // Both segments must succeed in planning before moving the holder.
      run(group, transfer);
      status("LOWER_COMPONENT " + std::to_string(component));
      run(group, descend);
      return above;
    }
    throw std::runtime_error("Component transfer failed: " + last_plan_error_);
  }

  void run(Group& group, const Group::Plan& plan)
  {
    if (!rclcpp::ok() || group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Motion execution failed; no automatic retry");
  }

  void move(Group& group, const Transform& target)
  {
    Group::Plan result;
    if (!plan(group, target, result)) throw std::runtime_error("No collision-free transfer plan: " + last_plan_error_);
    run(group, result);
  }

  void moveJointTarget(Group& group, const std::vector<double>& target, const std::string& label)
  {
    group.setStartStateToCurrentState();
    group.clearPoseTargets();
    if (!group.setJointValueTarget(target))
      throw std::runtime_error("Cannot set ready joint target for " + label);
    Group::Plan result;
    const auto code = group.plan(result);
    group.clearPoseTargets();
    if (code != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("No collision-free ready posture for " + label +
                               " code=" + std::to_string(code.val));
    run(group, result);
  }

  bool planCartesian(Group& group, const Transform& target,
                     const moveit::core::RobotState& start, Group::Plan& result)
  {
    group.setStartState(start);
    moveit_msgs::msg::RobotTrajectory message;
    const double fraction = group.computeCartesianPath({pose(target)}, 0.002, 1.5, message, true);
    if (!std::isfinite(fraction) || fraction < 0.999 || message.joint_trajectory.points.empty()) {
      RCLCPP_WARN(node_->get_logger(), "SOLDER Cartesian plan incomplete: group=%s fraction=%.4f",
                  group.getName().c_str(), fraction);
      return false;
    }
    robot_trajectory::RobotTrajectory trajectory(group.getRobotModel(), group.getName());
    trajectory.setRobotTrajectoryMsg(start, message);
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    if (!timing.computeTimeStamps(trajectory, 0.10, 0.10)) return false;
    moveit::core::robotStateToRobotStateMsg(start, result.start_state_);
    trajectory.getRobotTrajectoryMsg(result.trajectory_);
    return true;
  }

  bool tryCartesian(Group& group, const Transform& target, double& fraction)
  {
    auto start = group.getCurrentState(3.0);
    if (!start) throw std::runtime_error("Missing current robot state");
    // Use a diff start state, retaining attached objects from the server scene.
    group.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory message;
    fraction = group.computeCartesianPath({pose(target)}, 0.002, 1.5, message, true);
    if (!std::isfinite(fraction) || fraction < 0.999 || message.joint_trajectory.points.empty())
      return false;
    robot_trajectory::RobotTrajectory trajectory(group.getRobotModel(), group.getName());
    trajectory.setRobotTrajectoryMsg(*start, message);
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    if (!timing.computeTimeStamps(trajectory, 0.10, 0.10))
      return false;
    Group::Plan result;
    trajectory.getRobotTrajectoryMsg(result.trajectory_);
    run(group, result);
    return true;
  }

  void cartesian(Group& group, const Transform& target)
  {
    double fraction = 0.0;
    if (!tryCartesian(group, target, fraction)) {
      throw std::runtime_error("Incomplete Cartesian contact/retreat path; fraction=" +
                               std::to_string(fraction) + "; refusing partial execution");
    }
  }

  void cartesianOrPlan(Group& group, const Transform& target, const std::string& label)
  {
    double fraction = 0.0;
    if (tryCartesian(group, target, fraction)) return;
    RCLCPP_WARN(node_->get_logger(),
                "SOLDER %s Cartesian incomplete (fraction=%.4f); trying position-only park plan",
                label.c_str(), fraction);
    // The holder has released the component, so its wrist orientation is no
    // longer constrained by the grasp. The exact inherited grasp quaternion
    // may have no IK at the park position; plan to the same clear position
    // while allowing MoveIt to choose a reachable orientation.
    group.setStartStateToCurrentState();
    group.clearPoseTargets();
    group.setPositionTarget(target.translation().x(), target.translation().y(),
                            target.translation().z(), group.getEndEffectorLink());
    Group::Plan result;
    const auto code = group.plan(result);
    group.clearPoseTargets();
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      throw std::runtime_error("No reachable position-only park plan: group=" + group.getName() +
                               " code=" + std::to_string(code.val));
    }
    RCLCPP_INFO(node_->get_logger(), "SOLDER %s position-only park plan succeeded", label.c_str());
    run(group, result);
  }

  struct PadSearchStats
  {
    int poses = 0, ik = 0, collision_free = 0, transfers = 0, descents = 0;
  };

  bool planPadApproach(Group& group, const planning_scene::PlanningScenePtr& scene,
                       const Transform& above, const Transform& contact,
                       Group::Plan& approach_plan, Group::Plan& contact_plan,
                       PadSearchStats& stats)
  {
    // Bound expensive OMPL calls. Unreachable orientations are filtered by IK
    // first instead of spending the full planning timeout on every one.
    constexpr int max_transfer_plans = 16;
    if (stats.transfers >= max_transfer_plans) return false;
    ++stats.poses;
    const auto* joints = group.getRobotModel()->getJointModelGroup(group.getName());
    if (!joints) throw std::runtime_error("Missing solder arm joint group");
    for (int attempt = 0; attempt < 3 && stats.transfers < max_transfer_plans; ++attempt) {
      moveit::core::RobotState candidate(scene->getCurrentState());
      if (attempt) candidate.setToRandomPositions(joints);
      candidate.update();
      if (!candidate.setFromIK(joints, above, group.getEndEffectorLink(), 0.05)) continue;
      ++stats.ik;
      candidate.update();
      if (!candidate.satisfiesBounds(joints)) continue;
      if (scene->isStateColliding(candidate, "", false)) {
        if (stats.ik == 1) logContacts(scene, candidate, "PAD APPROACH");
        continue;
      }
      ++stats.collision_free;

      // Verify the whole contact segment with both objects attached before
      // asking OMPL to transfer to this particular IK branch.
      if (!planCartesian(group, contact, candidate, contact_plan)) continue;
      ++stats.descents;
      std::vector<double> values;
      candidate.copyJointGroupPositions(joints, values);
      group.clearPoseTargets();
      group.setStartStateToCurrentState();
      if (!group.setJointValueTarget(values)) continue;
      ++stats.transfers;
      const double saved_time = group.getPlanningTime();
      group.setPlanningTime(4.0);
      const auto code = group.plan(approach_plan);
      group.setPlanningTime(saved_time);
      if (code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node_->get_logger(), "SOLDER PAD transfer failed: code=%d", code.val);
        continue;
      }
      // Recompute contact from the actual end of the returned transfer plan,
      // retaining attached objects and the stationary holder's joint state.
      moveit::core::RobotState start(scene->getCurrentState());
      moveit::core::robotStateMsgToRobotState(approach_plan.start_state_, start);
      robot_trajectory::RobotTrajectory transfer(group.getRobotModel(), group.getName());
      transfer.setRobotTrajectoryMsg(start, approach_plan.trajectory_);
      if (transfer.getWayPointCount() &&
          planCartesian(group, contact, transfer.getLastWayPoint(), contact_plan)) return true;
    }
    return false;
  }

  void solderPad(Group& group, const Transform& board, const Transform& hand_to_tip,
                  int component, std::size_t pad, ArmSide solderer)
  {
    // padOffset() returns the centre of the pad's top face: its Z coordinate
    // includes board half-thickness, pad height above the board, and half the
    // pad thickness. Target that surface directly so the modeled tip endpoint
    // makes contact instead of hovering above it.
    const Eigen::Vector3d pad_position = board * vector(pcb::padOffset(component - 1, pad));
    const Eigen::Vector3d normal = board.linear().col(2);
    const Eigen::Vector3d tip_position = pad_position;
    status("SEARCH_PAD " + std::to_string(component) + ":" + std::to_string(pad + 1));
    RCLCPP_INFO(node_->get_logger(), "SOLDER PAD CONTACT TARGET component=%d pad=%zu tip_world=(%.4f %.4f %.4f)",
                component, pad + 1, tip_position.x(), tip_position.y(), tip_position.z());
    auto scene = snapshot(group);
    moveit::core::RobotState start(scene->getCurrentState());
    start.update();
    if (!start.satisfiesBounds() || scene->isStateColliding(start, "", false)) {
      logContacts(scene, start, "PAD START");
      throw std::runtime_error("Invalid start state at pad search");
    }
    const std::string base_link = prefix(solderer) + "base_link";
    if (!group.getRobotModel()->getLinkModel(base_link))
      throw std::runtime_error("Missing solder arm base link: " + base_link);

    // Point from the handle toward the pad. Favour putting the handle toward
    // this arm's base, while keeping it outside the component on the pad side.
    const double sign = pad == 0 ? 1.0 : -1.0;
    Eigen::Vector3d preferred = board.linear().transpose() *
        (pad_position - start.getGlobalLinkTransform(base_link).translation());
    preferred.z() = 0.0;
    preferred.x() = sign * std::abs(preferred.x());
    if (preferred.norm() < 1e-6) preferred = Eigen::Vector3d(sign, 0, 0);
    preferred.normalize();
    std::vector<Eigen::Vector3d> directions{preferred};
    for (double azimuth_deg : {-75.0, -45.0, 0.0, 45.0, 75.0}) {
      const double a = azimuth_deg * M_PI / 180.0;
      const Eigen::Vector3d direction(sign * std::cos(a), std::sin(a), 0);
      if ((direction - preferred).norm() > 1e-6) directions.push_back(direction);
    }
    std::stable_sort(directions.begin(), directions.end(), [&](const auto& a, const auto& b) {
      return a.dot(preferred) > b.dot(preferred);
    });

    Group::Plan approach_plan, contact_plan;
    Transform contact = Transform::Identity(), above = Transform::Identity();
    bool found = false;
    PadSearchStats stats;
    double previous_height = -1.0;
    for (double height : {0.02, std::min(clearance_, 0.04), clearance_}) {
      if (std::abs(height - previous_height) < 1e-6) continue;
      previous_height = height;
      for (const auto& direction : directions) {
        const Eigen::Vector3d horizontal = board.linear() * direction;
        const Eigen::Vector3d across = normal.cross(horizontal).normalized();
        // Tip-frame +X points toward the pad. Tilt from the board normal,
        // rather than restricting every approach to the board's X-Z plane.
        for (double tilt_deg : {60.0, 45.0, 30.0}) {
          const double angle = tilt_deg * M_PI / 180.0;
          const Eigen::Vector3d axis = horizontal * std::sin(angle) - normal * std::cos(angle);
          Transform tip = Transform::Identity();
          tip.linear().col(0) = axis;
          tip.linear().col(1) = across;
          tip.linear().col(2) = axis.cross(across);
          tip.translation() = tip_position;
          for (int roll = 0; roll < 4; ++roll) {
            Transform oriented = tip;
            oriented.linear() = tip.linear() *
                Eigen::AngleAxisd(roll * M_PI / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
            contact = oriented * hand_to_tip.inverse();
            above = contact;
            above.translation() += normal * height;
            if (planPadApproach(group, scene, above, contact, approach_plan, contact_plan, stats)) {
              RCLCPP_INFO(node_->get_logger(),
                          "SOLDER PAD plan selected: tilt=%.0f roll=%d clearance=%.3f direction=(%.3f %.3f)",
                          tilt_deg, roll * 90, height, direction.x(), direction.y());
              found = true;
              break;
            }
          }
          if (found) break;
        }
        if (found) break;
      }
      if (found) break;
    }
    RCLCPP_INFO(node_->get_logger(),
                "SOLDER PAD SEARCH poses=%d IK=%d collision_free=%d descents=%d transfer_plans=%d success=%d",
                stats.poses, stats.ik, stats.collision_free, stats.descents, stats.transfers, found);
    if (!found) throw std::runtime_error("No complete solder approach/contact plan; see SOLDER PAD SEARCH");
    status("APPROACH_PAD " + std::to_string(component) + ":" + std::to_string(pad + 1));
    run(group, approach_plan);
    run(group, contact_plan);
    status("DWELL_PAD " + std::to_string(component) + ":" + std::to_string(pad + 1));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(dwell_);
    while (std::chrono::steady_clock::now() < deadline) {
      if (!rclcpp::ok()) throw std::runtime_error("Solder dwell interrupted");
      rclcpp::sleep_for(50ms);
    }
    cartesian(group, above);
    markPad(component, pad, pad_position);
  }

  void markPad(int component, std::size_t pad, const Eigen::Vector3d& at)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "world"; m.header.stamp = node_->now();
    m.ns = "soldered_pads"; m.id = 2 * (component - 1) + static_cast<int>(pad);
    m.type = visualization_msgs::msg::Marker::SPHERE; m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.pose.position.x = at.x(); m.pose.position.y = at.y(); m.pose.position.z = at.z();
    m.scale.x = 0.004; m.scale.y = 0.004; m.scale.z = 0.002;
    m.color.r = 0.1f; m.color.g = 1.0f; m.color.b = 0.2f; m.color.a = 1.0f;
    // Publish the full set so late RViz subscribers see every completed pad.
    marks_.markers.push_back(m);
    marker_pub_->publish(marks_);
  }

  void status(const std::string& text)
  {
    std_msgs::msg::String msg; msg.data = text; status_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "SOLDER %s", text.c_str());
  }

  std::shared_ptr<DualArmInterface> robot_;
  rclcpp::Node::SharedPtr node_;
  Scene scene_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  std::string last_plan_error_;
  double dwell_ = 1.5, clearance_ = 0.06;
  std::set<int> completed_;
  visualization_msgs::msg::MarkerArray marks_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

SolderTask::SolderTask(std::shared_ptr<DualArmInterface> robot)
  : TaskBase(robot), impl_(std::make_unique<Impl>(std::move(robot))) {}
SolderTask::~SolderTask() = default;
bool SolderTask::execute(int component) { return impl_->execute(component); }
}  // namespace ur_onrobot_mtc