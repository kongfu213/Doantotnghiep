#include "ur_onrobot_mtc/tasks/solder_task.hpp"
#include "ur_onrobot_mtc/pcb_geometry.hpp"

// RRTConnect plans free-space transfers; Cartesian interpolation follows each
// derived rectangular pad edge and performs the contact/descent stroke.

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
#include <moveit_msgs/msg/planner_interface_description.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <future>
#include <map>
#include <set>
#include <limits>
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
    if (!node_->has_parameter("solder.planner_timeout_seconds"))
      node_->declare_parameter<double>("solder.planner_timeout_seconds", 1.5);
    // Use this only if MoveIt cannot provide its runtime planner catalog.
    // When the catalog is available, refreshPlannerCatalog() selects the
    // advertised RRTConnect ID or the pipeline default.
    planner_trials_ = {"move_group:RRTConnectkConfigDefault"};
    planner_timeout_seconds_ = node_->get_parameter("solder.planner_timeout_seconds").as_double();
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
      planner_timeout_seconds_ = node_->get_parameter("solder.planner_timeout_seconds").as_double();
      if (!std::isfinite(dwell_) || dwell_ < 0.0 || dwell_ > 30.0 ||
          !std::isfinite(clearance_) || clearance_ < 0.02 || clearance_ > 0.15 ||
          !std::isfinite(planner_timeout_seconds_) || planner_timeout_seconds_ < 0.1 ||
          planner_timeout_seconds_ > 10.0)
        throw std::runtime_error("Invalid solder dwell, approach height, or planner settings");
      if (!scene_.getAttachedObjects().empty())
        throw std::runtime_error("Start SOLDER with both hands free; inspect status after a failed task");

      Group holder_group(node_, prefix(holder) + "ur_onrobot_manipulator");
      Group tool_group(node_, prefix(solderer) + "ur_onrobot_manipulator");
      configure(holder_group, holder); configure(tool_group, solderer);
      refreshPlannerCatalog(tool_group);
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
        const std::string component_id = "component_" + std::to_string(i);
        if (i == 3)
          requireCylinder(component_id, pcb::component_z, pcb::component_y / 2.0);
        else
          requireBox(component_id, 0, {pcb::component_x, pcb::component_y, pcb::component_z});
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
        // Raise the empty holder first. For the left component arm, plan from
        // this clear pose directly to the SRDF home pose instead of targeting
        // an intermediate Cartesian park point that may be outside its reach.
        Transform holder_park = above * hand_to_component.inverse();
        Transform holder_park_high = holder_park;
        holder_park_high.translation() += board.linear().col(2) * 0.05;
        status("PARK_HOLDER_RAISE " + std::to_string(i));
        cartesianOrPlan(holder_group, holder_park_high, "PARK_HOLDER_RAISE");

        // The component is released and the gripper has risen clear of it;
        // restore normal collision rules before planning the return move.
        component_contacts.restore();

        if (holder == ArmSide::LEFT) {
          status("HOME_LEFT_AFTER_RELEASE " + std::to_string(i));
          moveNamedTarget(holder_group, "left_home", "after component release");
        } else {
          const double board_side = -1.0;
          holder_park = holder_park_high;
          holder_park.translation() += board.linear().col(1) * (board_side * 0.17);
          RCLCPP_INFO(node_->get_logger(),
                      "SOLDER holder park TCP=(%.4f %.4f %.4f)",
                      holder_park.translation().x(), holder_park.translation().y(),
                      holder_park.translation().z());
          status("PARK_HOLDER " + std::to_string(i));
          cartesianOrPlan(holder_group, holder_park, "PARK_HOLDER");
        }
        status("HOLDER_CLEAR " + std::to_string(i));

        if (i == 3) {
          solderCircularSeam(tool_group, board, hand_to_tip, i, solderer);
        } else {
          for (std::size_t pad = 0; pad < 2; ++pad)
            solderPad(tool_group, board, hand_to_tip, i, pad, solderer);
        }

        completed_.insert(i);
        // Return the soldering arm to its clear standby pose, then move the
        // component arm to its configured wait pose for the next pick.
        move(tool_group, tool_standby * hand_to_tool.inverse());
        if (i != selected.back()) {
          status("RESET_HOLDER_READY " + std::to_string(i));
          moveJointTarget(holder_group, holder_ready_positions, "next component pick");
        }
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

  void requireCylinder(const std::string& id, double height, double radius)
  {
    const auto objects = scene_.getObjects({id});
    if (!objects.count(id)) throw std::runtime_error("Run pcb_solder.launch.py first: missing " + id);
    const auto& primitives = objects.at(id).primitives;
    if (primitives.size() != 1 ||
        primitives[0].type != shape_msgs::msg::SolidPrimitive::CYLINDER ||
        primitives[0].dimensions.size() != 2)
      throw std::runtime_error("Expected cylinder collision geometry: " + id);
    if (std::abs(primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] - height) > 1e-6 ||
        std::abs(primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] - radius) > 1e-6)
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

  void moveNamedTarget(Group& group, const std::string& target, const std::string& label)
  {
    group.setStartStateToCurrentState();
    group.clearPoseTargets();
    if (!group.setNamedTarget(target))
      throw std::runtime_error("Cannot set named target " + target + " for " + label);
    Group::Plan result;
    const auto code = group.plan(result);
    group.clearPoseTargets();
    if (code != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("No collision-free named target " + target + " for " + label +
                               " code=" + std::to_string(code.val));
    run(group, result);
  }

  bool planCartesian(Group& group, const std::vector<Transform>& targets,
                     const moveit::core::RobotState& start, Group::Plan& result,
                     bool log_incomplete = true)
  {
    if (targets.empty()) return false;
    group.setStartState(start);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(targets.size());
    for (const auto& target : targets) waypoints.push_back(pose(target));
    moveit_msgs::msg::RobotTrajectory message;
    const double fraction = group.computeCartesianPath(waypoints, 0.002, 1.5, message, true);
    if (!std::isfinite(fraction) || fraction < 0.999 || message.joint_trajectory.points.empty()) {
      if (log_incomplete)
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

  bool planCartesian(Group& group, const Transform& target,
                     const moveit::core::RobotState& start, Group::Plan& result,
                     bool log_incomplete = true)
  {
    return planCartesian(group, std::vector<Transform>{target}, start, result, log_incomplete);
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

  struct PadPlanCandidate
  {
    Transform above;
    Transform contact;
    Group::Plan transfer;
    Group::Plan descent;
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
    double clearance = 0.0;
    double tilt_deg = 0.0;
    int roll_deg = 0;
    double tcp_length = 0.0;
    double joint_length = 0.0;
    double duration = 0.0;
    double smoothness = 0.0;
    std::string planner;
    double score = 0.0;
  };

  struct PlannerTrial
  {
    std::string pipeline;
    std::string planner_id;
    bool pose_goal = false;
  };

  static PlannerTrial parsePlannerTrial(const std::string& value)
  {
    const auto separator = value.find(':');
    if (separator == std::string::npos || separator == 0)
      throw std::runtime_error("Invalid solder.planner_trials entry: " + value +
                               " (expected pipeline:planner_id)");
    PlannerTrial trial;
    trial.pipeline = value.substr(0, separator);
    trial.planner_id = value.substr(separator + 1);
    trial.pose_goal = trial.pipeline == "pilz_industrial_motion_planner" &&
                      trial.planner_id == "LIN";
    return trial;
  }

  void refreshPlannerCatalog(Group& group)
  {
    available_planners_.clear();
    available_pipeline_plugins_.clear();
    planner_catalog_available_ = false;
    std::vector<moveit_msgs::msg::PlannerInterfaceDescription> descriptions;
    if (!group.getInterfaceDescriptions(descriptions)) {
      RCLCPP_WARN(node_->get_logger(),
                  "SOLDER could not query loaded planner pipelines; configured trials will be attempted as-is");
      return;
    }
    planner_catalog_available_ = true;
    planner_trials_.clear();
    for (const auto& description : descriptions) {
      const std::set<std::string> ids(description.planner_ids.begin(), description.planner_ids.end());
      available_planners_[description.pipeline_id] = ids;
      available_pipeline_plugins_[description.pipeline_id] = description.name;
      RCLCPP_INFO(node_->get_logger(), "SOLDER loaded pipeline=%s plugin=%s planners=%zu",
                  description.pipeline_id.c_str(), description.name.c_str(), ids.size());

      std::string plugin = description.name;
      std::transform(plugin.begin(), plugin.end(), plugin.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      if (planner_trials_.empty() && plugin.find("ompl") != std::string::npos) {
        std::string selected_id;
        // Prefer the conventional name, but accept custom planner config IDs
        // that still identify the RRTConnect algorithm.
        for (const auto& id : description.planner_ids) {
          if (id == "RRTConnectkConfigDefault") {
            selected_id = id;
            break;
          }
          std::string lowered = id;
          std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                         [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
          if (selected_id.empty() && lowered.find("rrtconnect") != std::string::npos)
            selected_id = id;
        }

        // An empty planner ID delegates to the default planner configured for
        // this pipeline when no RRTConnect-specific ID is advertised.
        planner_trials_.push_back(description.pipeline_id + ":" + selected_id);
        RCLCPP_INFO(node_->get_logger(), "SOLDER selected planner trial=%s:%s",
                    description.pipeline_id.c_str(),
                    selected_id.empty() ? "<pipeline default>" : selected_id.c_str());
      }
    }
    if (planner_trials_.empty())
      RCLCPP_WARN(node_->get_logger(),
                  "SOLDER planner catalog has no OMPL pipeline; only direct Cartesian transfer will be tried");
  }

  std::string resolvePipelineId(const std::string& requested) const
  {
    if (available_planners_.count(requested)) return requested;

    std::string alias = requested;
    std::transform(alias.begin(), alias.end(), alias.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    for (const auto& entry : available_pipeline_plugins_) {
      std::string plugin = entry.second;
      std::transform(plugin.begin(), plugin.end(), plugin.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      if (plugin.find(alias) != std::string::npos) return entry.first;
    }
    return requested;
  }

  bool plannerTrialAvailable(const PlannerTrial& trial) const
  {
    if (!planner_catalog_available_) return true;
    const auto pipeline = available_planners_.find(resolvePipelineId(trial.pipeline));
    if (pipeline == available_planners_.end()) return false;
    return trial.planner_id.empty() || pipeline->second.count(trial.planner_id) != 0;
  }

  static double trajectoryDuration(const Group::Plan& plan)
  {
    const auto& points = plan.trajectory_.joint_trajectory.points;
    if (points.empty()) return std::numeric_limits<double>::infinity();
    const auto& t = points.back().time_from_start;
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
  }

  static double trajectoryJointLength(const Group::Plan& plan)
  {
    const auto& points = plan.trajectory_.joint_trajectory.points;
    if (points.empty()) return std::numeric_limits<double>::infinity();

    double length = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
      if (points[i].positions.size() != points[i - 1].positions.size())
        return std::numeric_limits<double>::infinity();
      for (std::size_t j = 0; j < points[i].positions.size(); ++j)
        length += std::abs(points[i].positions[j] - points[i - 1].positions[j]);
    }
    return length;
  }

  static double trajectoryTcpLength(const Group::Plan& plan,
                                    const moveit::core::RobotModelConstPtr& model,
                                    const std::string& group_name,
                                    const std::string& end_effector)
  {
    if (plan.trajectory_.joint_trajectory.points.empty() || end_effector.empty())
      return std::numeric_limits<double>::infinity();

    moveit::core::RobotState start(model);
    moveit::core::robotStateMsgToRobotState(plan.start_state_, start);
    robot_trajectory::RobotTrajectory trajectory(model, group_name);
    trajectory.setRobotTrajectoryMsg(start, plan.trajectory_);
    if (trajectory.getWayPointCount() < 2)
      return 0.0;

    Eigen::Vector3d previous =
        trajectory.getWayPoint(0).getGlobalLinkTransform(end_effector).translation();
    double length = 0.0;
    for (std::size_t i = 1; i < trajectory.getWayPointCount(); ++i) {
      const Eigen::Vector3d current =
          trajectory.getWayPoint(i).getGlobalLinkTransform(end_effector).translation();
      length += (current - previous).norm();
      previous = current;
    }
    return length;
  }

  static double trajectorySmoothness(const Group::Plan& plan)
  {
    const auto& points = plan.trajectory_.joint_trajectory.points;
    if (points.size() < 3) return 0.0;

    const auto seconds = [](const auto& time) {
      return static_cast<double>(time.sec) + static_cast<double>(time.nanosec) * 1e-9;
    };
    double smoothness = 0.0;
    double previous_dt = 0.0;
    std::vector<double> previous_velocity;
    for (std::size_t i = 1; i < points.size(); ++i) {
      if (points[i].positions.size() != points[i - 1].positions.size())
        return std::numeric_limits<double>::infinity();

      const double dt = seconds(points[i].time_from_start) -
                        seconds(points[i - 1].time_from_start);
      if (!std::isfinite(dt) || dt <= 1e-9)
        return std::numeric_limits<double>::infinity();

      std::vector<double> velocity(points[i].positions.size());
      for (std::size_t j = 0; j < velocity.size(); ++j)
        velocity[j] = (points[i].positions[j] - points[i - 1].positions[j]) / dt;

      if (!previous_velocity.empty()) {
        const double interval = 0.5 * (previous_dt + dt);
        if (interval <= 1e-9)
          return std::numeric_limits<double>::infinity();
        for (std::size_t j = 0; j < velocity.size(); ++j) {
          const double acceleration = (velocity[j] - previous_velocity[j]) / interval;
          smoothness += acceleration * acceleration * interval;
        }
      }
      previous_velocity = std::move(velocity);
      previous_dt = dt;
    }
    return smoothness;
  }

  static void scoreCandidates(std::vector<PadPlanCandidate>& candidates)
  {
    if (candidates.empty()) return;

    const auto range_for = [&](auto metric) {
      double minimum = std::numeric_limits<double>::infinity();
      double maximum = -std::numeric_limits<double>::infinity();
      for (const auto& candidate : candidates) {
        const double value = metric(candidate);
        minimum = std::min(minimum, value);
        maximum = std::max(maximum, value);
      }
      return std::pair<double, double>{minimum, maximum};
    };
    const auto normalized = [](double value, const std::pair<double, double>& range) {
      const double span = range.second - range.first;
      return span > 1e-9 ? (value - range.first) / span : 0.0;
    };

    const auto joint_length_range = range_for([](const auto& c) { return c.joint_length; });
    const auto duration_range = range_for([](const auto& c) { return c.duration; });
    const auto smoothness_range = range_for([](const auto& c) { return c.smoothness; });
    const auto clearance_range = range_for([](const auto& c) { return c.clearance; });

    for (auto& candidate : candidates) {
      // Collision checks are hard filters. This small higher-clearance preference
      // is a geometric proxy, not an exact minimum-distance measurement.
      const double clearance_preference =
          1.0 - normalized(candidate.clearance, clearance_range);
      // TCP path length is the primary objective and is compared directly
      // when ranking candidates. These metrics only break near-equal-length ties.
      candidate.score =
          0.50 * normalized(candidate.joint_length, joint_length_range) +
          0.25 * normalized(candidate.duration, duration_range) +
          0.20 * normalized(candidate.smoothness, smoothness_range) +
          0.05 * clearance_preference;
    }
  }

  void planPadApproach(Group& group, const planning_scene::PlanningScenePtr& scene,
                       const Transform& above, const std::vector<Transform>& contact_path,
                       std::vector<PadPlanCandidate>& candidates,
                       PadSearchStats& stats, const Eigen::Vector3d& direction,
                       double tilt_deg, int roll_deg, double clearance)
  {
    // Bound calls across the enabled planner portfolio and retain feasible
    // alternatives instead of stopping at the first plan.
    constexpr int max_transfer_plans = 16;
    if (stats.transfers >= max_transfer_plans) return;
    ++stats.poses;
    const auto* joints = group.getRobotModel()->getJointModelGroup(group.getName());
    if (!joints) throw std::runtime_error("Missing solder arm joint group");
    const std::size_t candidates_before = candidates.size();
    for (int attempt = 0; attempt < 3 &&
         stats.transfers < max_transfer_plans &&
         candidates.size() == candidates_before; ++attempt) {
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

      // Check the complete descent and solder stroke with the iron attached.
      Group::Plan contact_plan;
      if (!planCartesian(group, contact_path, candidate, contact_plan)) continue;
      ++stats.descents;

      const auto recordCandidate = [&](Group::Plan&& approach_plan,
                                       const std::string& planner_name) {
        // Recheck the descent from the actual end state of this planner's route.
        moveit::core::RobotState start(scene->getCurrentState());
        moveit::core::robotStateMsgToRobotState(approach_plan.start_state_, start);
        robot_trajectory::RobotTrajectory transfer(group.getRobotModel(), group.getName());
        transfer.setRobotTrajectoryMsg(start, approach_plan.trajectory_);
        if (!transfer.getWayPointCount()) return false;
        Group::Plan verified_contact;
        if (!planCartesian(group, contact_path, transfer.getLastWayPoint(), verified_contact))
          return false;

        PadPlanCandidate option;
        option.above = above;
        option.contact = contact_path.front();
        option.transfer = std::move(approach_plan);
        option.descent = std::move(verified_contact);
        option.direction = direction;
        option.clearance = clearance;
        option.tilt_deg = tilt_deg;
        option.roll_deg = roll_deg;
        option.planner = planner_name;
        const auto model = group.getRobotModel();
        const std::string end_effector = group.getEndEffectorLink();
        option.tcp_length =
            trajectoryTcpLength(option.transfer, model, group.getName(), end_effector) +
            trajectoryTcpLength(option.descent, model, group.getName(), end_effector);
        option.joint_length =
            trajectoryJointLength(option.transfer) + trajectoryJointLength(option.descent);
        option.duration =
            trajectoryDuration(option.transfer) + trajectoryDuration(option.descent);
        option.smoothness =
            trajectorySmoothness(option.transfer) + trajectorySmoothness(option.descent);
        if (!std::isfinite(option.tcp_length) || !std::isfinite(option.joint_length) ||
            !std::isfinite(option.duration) || !std::isfinite(option.smoothness)) {
          RCLCPP_WARN(node_->get_logger(),
                      "SOLDER PAD discarded %s plan with invalid trajectory metrics",
                      planner_name.c_str());
          return false;
        }
        RCLCPP_INFO(node_->get_logger(), "SOLDER PAD feasible planner=%s tcp_length=%.3f",
                    planner_name.c_str(), option.tcp_length);
        candidates.push_back(std::move(option));
        return true;
      };

      // First try a collision-checked straight Cartesian transfer.
      ++stats.transfers;
      Group::Plan direct_plan;
      moveit::core::RobotState current(scene->getCurrentState());
      if (planCartesian(group, above, current, direct_plan, false))
        recordCandidate(std::move(direct_plan), "CARTESIAN_DIRECT");

      // Also ask each configured MoveIt pipeline for an alternative. This
      // portfolio lets the TCP-length ranking compare real planner outputs.
      std::vector<double> values;
      candidate.copyJointGroupPositions(joints, values);
      const std::string original_pipeline = group.getPlanningPipelineId();
      const std::string original_planner = group.getPlannerId();
      const double original_planning_time = group.getPlanningTime();
      for (const auto& raw_trial : planner_trials_) {
        if (stats.transfers >= max_transfer_plans) break;
        const PlannerTrial trial = parsePlannerTrial(raw_trial);
        const std::string planner_name = trial.pipeline + ":" +
            (trial.planner_id.empty() ? std::string("default") : trial.planner_id);
        if (!plannerTrialAvailable(trial)) {
          if (warned_unavailable_planners_.insert(planner_name).second)
            RCLCPP_WARN(node_->get_logger(),
                        "SOLDER planner trial skipped because it is not loaded/configured: %s",
                        planner_name.c_str());
          continue;
        }
        const std::string resolved_pipeline = resolvePipelineId(trial.pipeline);
        if (resolved_pipeline != trial.pipeline) {
          const auto plugin = available_pipeline_plugins_.find(resolved_pipeline);
          RCLCPP_INFO(node_->get_logger(),
                      "SOLDER planner alias resolved: %s -> %s (plugin=%s)",
                      trial.pipeline.c_str(), resolved_pipeline.c_str(),
                      plugin == available_pipeline_plugins_.end() ? "unknown" : plugin->second.c_str());
        }
        const std::string resolved_planner_name = resolved_pipeline + ":" +
            (trial.planner_id.empty() ? std::string("default") : trial.planner_id);
        group.clearPoseTargets();
        group.setStartStateToCurrentState();
        group.setPlanningPipelineId(resolved_pipeline);
        group.setPlannerId(trial.planner_id);

        const bool target_set = trial.pose_goal
                                    ? group.setPoseTarget(pose(above))
                                    : group.setJointValueTarget(values);
        if (!target_set) {
          RCLCPP_WARN(node_->get_logger(),
                      "SOLDER cannot set goal for pipeline=%s planner=%s",
                      trial.pipeline.c_str(), trial.planner_id.c_str());
          group.clearPoseTargets();
          group.setPlanningPipelineId(original_pipeline);
          group.setPlannerId(original_planner);
          continue;
        }

        Group::Plan planner_plan;
        group.setPlanningTime(planner_timeout_seconds_);
        ++stats.transfers;
        const auto code = group.plan(planner_plan);
        group.setPlanningTime(original_planning_time);
        group.clearPoseTargets();
        group.setPlanningPipelineId(original_pipeline);
        group.setPlannerId(original_planner);
        if (code != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(node_->get_logger(),
                      "SOLDER planner failed: %s code=%d (pipeline may not be loaded)",
                      resolved_planner_name.c_str(), code.val);
          continue;
        }
        recordCandidate(std::move(planner_plan), resolved_planner_name);
      }
    }
  }

  void solderPad(Group& group, const Transform& board, const Transform& hand_to_tip,
                  int component, std::size_t pad, ArmSide solderer)
  {
    const std::string object_id = "component_" + std::to_string(component);
    const Transform component_pose = worldPose(object_id);
    const auto component_objects = scene_.getObjects({object_id});
    const auto component_it = component_objects.find(object_id);
    if (component_it == component_objects.end() || component_it->second.primitives.size() != 1 ||
        component_it->second.primitives.front().type != shape_msgs::msg::SolidPrimitive::BOX ||
        component_it->second.primitives.front().dimensions.size() != 3)
      throw std::runtime_error("Pad-edge welding requires one box-shaped component: " + object_id);
    const auto& dimensions = component_it->second.primitives.front().dimensions;
    const double component_x = dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
    const double component_y = dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
    const Eigen::Vector3d component_reference = vector(pcb::componentOffset(component - 1));
    const Eigen::Vector3d pad_reference = vector(pcb::padOffset(component - 1, pad));
    const Eigen::Vector3d pad_offset = pad_reference - component_reference;

    // The pad offset tells which side of the component contains this pad.
    // The weld stroke follows that side's edge, perpendicular to the offset.
    Eigen::Vector3d edge_axis_local;
    double edge_length = 0.0;
    if (std::abs(pad_offset.x()) >= std::abs(pad_offset.y())) {
      edge_axis_local = Eigen::Vector3d::UnitY();
      edge_length = component_y;
    } else {
      edge_axis_local = Eigen::Vector3d::UnitX();
      edge_length = component_x;
    }
    const double side_offset = std::hypot(pad_offset.x(), pad_offset.y());
    if (!std::isfinite(side_offset) || side_offset < 1e-6 ||
        !std::isfinite(edge_length) || edge_length <= 0.0)
      throw std::runtime_error("Cannot derive rectangular pad edge from PCB geometry");

    // Transform the geometry-relative pad center and edge axis through the
    // component's actual pose after placement. This keeps the stroke aligned
    // if the PCB/component pose changes in the planning scene.
    const Eigen::Vector3d pad_center = component_pose * pad_offset;
    const Eigen::Vector3d half_edge =
        component_pose.linear() * (0.5 * edge_length * edge_axis_local);
    std::vector<Eigen::Vector3d> pad_edge_points{
        pad_center - half_edge,
        pad_center + half_edge};

    // Shift the simulated weld seam 2 mm along world X, away from the
    // component. Pad 1 in the current scene is on negative X, so its seam
    // moves toward -X; the opposite pad moves toward +X.
    constexpr double weld_x_offset = 0.002;
    const double outward_x = pad_center.x() - component_pose.translation().x();
    const double x_sign = outward_x < 0.0 ? -1.0 : 1.0;
    const Eigen::Vector3d weld_shift(x_sign * weld_x_offset, 0.0, 0.0);
    for (auto& point : pad_edge_points) point += weld_shift;

    RCLCPP_INFO(node_->get_logger(),
                "SOLDER PAD EDGE component=%d pad=%zu length=%.4f x_offset=%.4f start=(%.4f %.4f %.4f) end=(%.4f %.4f %.4f)",
                component, pad + 1, edge_length, weld_shift.x(),
                pad_edge_points.front().x(), pad_edge_points.front().y(), pad_edge_points.front().z(),
                pad_edge_points.back().x(), pad_edge_points.back().y(), pad_edge_points.back().z());
    solderPath(group, board, hand_to_tip, component, pad, solderer,
               pad_edge_points, false, Eigen::Vector3d::Zero());
  }

  void solderCircularSeam(Group& group, const Transform& board, const Transform& hand_to_tip,
                          int component, ArmSide solderer)
  {
    const std::string object_id = "component_" + std::to_string(component);
    const auto objects = scene_.getObjects({object_id});
    const auto object = objects.find(object_id);
    if (object == objects.end() || object->second.header.frame_id != "world" ||
        object->second.primitives.size() != 1 || object->second.primitive_poses.size() != 1 ||
        object->second.primitives.front().type != shape_msgs::msg::SolidPrimitive::CYLINDER)
      throw std::runtime_error("Circular seam needs one world-frame cylinder: " + object_id);

    // Use the object's world pose as the cylinder centre, matching the pose
    // used for component placement. Treating primitive_poses.front() as the
    // world centre placed the seam below the PCB in the previous run (z=-0.007 m).
    const Transform cylinder = worldPose(object_id);
    const Eigen::Vector3d board_normal = board.linear().col(2).normalized();
    Eigen::Vector3d cylinder_axis = cylinder.linear().col(2).normalized();
    if (std::abs(cylinder_axis.dot(board_normal)) < 0.98)
      throw std::runtime_error("Circular component axis is not perpendicular to the PCB");
    if (cylinder_axis.dot(board_normal) < 0.0) cylinder_axis = -cylinder_axis;

    // Trace the round foot where it meets the PCB. The cylinder's collision
    // model supplies the centre and radius, so this follows the scene geometry.
    const double radius = object->second.primitives.front().dimensions.at(
        shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS);
    constexpr double seam_radial_offset = 0.002;  // 2 mm outside the cylinder surface
    const double path_radius = radius + seam_radial_offset;
    const Eigen::Vector3d seam_center =
        cylinder.translation() - cylinder_axis * (pcb::component_z / 2.0);
    RCLCPP_INFO(node_->get_logger(),
                "SOLDER CIRCULAR GEOMETRY component_center=(%.4f %.4f %.4f) axis=(%.3f %.3f %.3f) radius=%.4f path_radius=%.4f radial_offset=%.4f seam_center=(%.4f %.4f %.4f)",
                cylinder.translation().x(), cylinder.translation().y(), cylinder.translation().z(),
                cylinder_axis.x(), cylinder_axis.y(), cylinder_axis.z(), radius, path_radius,
                seam_radial_offset,
                seam_center.x(), seam_center.y(), seam_center.z());
    Eigen::Vector3d u = board.linear().col(0);
    u -= board_normal * u.dot(board_normal);
    u.normalize();
    const Eigen::Vector3d v = board_normal.cross(u).normalized();

    constexpr int samples = 48;
    std::vector<Eigen::Vector3d> seam_points;
    seam_points.reserve(samples + 1);
    for (int i = 0; i <= samples; ++i) {
      const double angle = 2.0 * M_PI * static_cast<double>(i) / samples;
      seam_points.push_back(seam_center + path_radius * (std::cos(angle) * u + std::sin(angle) * v));
    }
    solderPath(group, board, hand_to_tip, component, 0, solderer,
               seam_points, true, seam_center);
  }

  void solderPath(Group& group, const Transform& board, const Transform& hand_to_tip,
                  int component, std::size_t pad, ArmSide solderer,
                  const std::vector<Eigen::Vector3d>& requested_points,
                  bool circular_seam, const Eigen::Vector3d& seam_center)
  {
    if (requested_points.empty()) throw std::runtime_error("Empty solder contact path");
    std::vector<Eigen::Vector3d> contact_points = requested_points;
    const Eigen::Vector3d normal = board.linear().col(2).normalized();
    GraspContacts seam_contacts(node_);
    if (circular_seam) {
      // Permit only the solder iron to touch the intended seam and PCB while
      // planning/executing this visual weld stroke. The saved ACM is restored.
      seam_contacts.allow("component_" + std::to_string(component), {"solder_tool"});
      seam_contacts.allow("pcb", {"solder_tool"});
    }

    status(circular_seam
               ? "SEARCH_CIRCULAR_SEAM " + std::to_string(component)
               : "SEARCH_PAD " + std::to_string(component) + ":" + std::to_string(pad + 1));
    auto scene = snapshot(group);
    moveit::core::RobotState start(scene->getCurrentState());
    start.update();
    if (!start.satisfiesBounds() || scene->isStateColliding(start, "", false)) {
      logContacts(scene, start, circular_seam ? "SEAM START" : "PAD START");
      throw std::runtime_error("Invalid start state at solder path search");
    }
    const std::string base_link = prefix(solderer) + "base_link";
    if (!group.getRobotModel()->getLinkModel(base_link))
      throw std::runtime_error("Missing solder arm base link: " + base_link);

    Eigen::Vector3d preferred;
    if (circular_seam) {
      preferred = start.getGlobalLinkTransform(base_link).translation() - seam_center;
      preferred -= normal * preferred.dot(normal);
      if (preferred.norm() < 1e-6) preferred = board.linear().col(0);
      preferred.normalize();

      // Start on the side facing the soldering arm; the remaining samples form
      // one complete closed circumference.
      Eigen::Vector3d first_radial = contact_points.front() - seam_center;
      first_radial -= normal * first_radial.dot(normal);
      first_radial.normalize();
      const double phase = std::atan2(normal.dot(first_radial.cross(preferred)),
                                      first_radial.dot(preferred));
      const Eigen::Matrix3d rotate =
          Eigen::AngleAxisd(phase, normal).toRotationMatrix();
      for (auto& point : contact_points)
        point = seam_center + rotate * (point - seam_center);
    } else {
      const double sign = pad == 0 ? 1.0 : -1.0;
      preferred = board.linear().transpose() *
          (contact_points.front() - start.getGlobalLinkTransform(base_link).translation());
      preferred.z() = 0.0;
      preferred.x() = sign * std::abs(preferred.x());
      if (preferred.norm() < 1e-6) preferred = Eigen::Vector3d(sign, 0, 0);
      preferred.normalize();
    }

    const Eigen::Vector3d tip_position = contact_points.front();
    RCLCPP_INFO(node_->get_logger(),
                "SOLDER %s TARGET component=%d path_points=%zu start_world=(%.4f %.4f %.4f)",
                circular_seam ? "CIRCULAR_SEAM" : "PAD", component,
                contact_points.size(), tip_position.x(), tip_position.y(), tip_position.z());

    std::vector<Eigen::Vector3d> directions;
    if (circular_seam) {
      directions.push_back(board.linear().transpose() * preferred);
    } else {
      directions.push_back(preferred);
      // Search all sides of the pad. Restricting X by pad index can exclude
      // the only reachable IK branch for the opposite pad.
      for (double azimuth_deg : {-180.0, -135.0, -90.0, -45.0,
                                  0.0, 45.0, 90.0, 135.0}) {
        const double a = azimuth_deg * M_PI / 180.0;
        const Eigen::Vector3d direction(std::cos(a), std::sin(a), 0);
        if ((direction - preferred).norm() > 1e-6) directions.push_back(direction);
      }
      std::stable_sort(directions.begin(), directions.end(), [&](const auto& a, const auto& b) {
        return a.dot(preferred) > b.dot(preferred);
      });
    }

    std::vector<double> heights;
    for (double height : {0.02, std::min(clearance_, 0.04), clearance_}) {
      const bool duplicate = std::any_of(heights.begin(), heights.end(), [&](double existing) {
        return std::abs(existing - height) < 1e-6;
      });
      if (!duplicate) heights.push_back(height);
    }

    std::vector<PadPlanCandidate> candidates;
    PadSearchStats stats;
    constexpr int max_transfer_plans = 16;
    constexpr int max_pose_candidates = 72;
    const int max_poses_per_height =
        std::max(1, (max_pose_candidates + static_cast<int>(heights.size()) - 1) /
                        static_cast<int>(heights.size()));
    const int max_transfers_per_height =
        std::max(1, (max_transfer_plans + static_cast<int>(heights.size()) - 1) /
                        static_cast<int>(heights.size()));
    // Interleave tilt and roll values so the bounded search samples multiple
    // orientations before spending its pose budget on one orientation family.
    const std::array<std::pair<double, int>, 12> orientations{{
        {60.0, 0}, {45.0, 0}, {30.0, 0},
        {60.0, 90}, {45.0, 90}, {30.0, 90},
        {60.0, 180}, {45.0, 180}, {30.0, 180},
        {60.0, 270}, {45.0, 270}, {30.0, 270}}};

    for (double height : heights) {
      if (stats.transfers >= max_transfer_plans || stats.poses >= max_pose_candidates) break;
      const int transfers_at_height = stats.transfers;
      const int poses_at_height = stats.poses;
      for (const auto& orientation : orientations) {
        if (stats.transfers - transfers_at_height >= max_transfers_per_height ||
            stats.transfers >= max_transfer_plans ||
            stats.poses - poses_at_height >= max_poses_per_height ||
            stats.poses >= max_pose_candidates)
          break;
        const double tilt_deg = orientation.first;
        const int roll_deg = orientation.second;
        for (const auto& direction : directions) {
          if (stats.transfers - transfers_at_height >= max_transfers_per_height ||
              stats.transfers >= max_transfer_plans ||
              stats.poses - poses_at_height >= max_poses_per_height ||
              stats.poses >= max_pose_candidates)
            break;
          const double angle = tilt_deg * M_PI / 180.0;
          std::vector<Transform> contact_path;
          contact_path.reserve(contact_points.size());
          for (const auto& point : contact_points) {
            Eigen::Vector3d horizontal;
            if (circular_seam) {
              // The iron handle stays outside the round body; its tip axis
              // points from that outer side into the joint line.
              horizontal = seam_center - point;
              horizontal -= normal * horizontal.dot(normal);
              if (horizontal.norm() < 1e-9) continue;
              horizontal.normalize();
            } else {
              horizontal = (board.linear() * direction).normalized();
            }
            const Eigen::Vector3d across = normal.cross(horizontal).normalized();
            const Eigen::Vector3d axis =
                horizontal * std::sin(angle) - normal * std::cos(angle);
            Transform tip = Transform::Identity();
            tip.linear().col(0) = axis;
            tip.linear().col(1) = across;
            tip.linear().col(2) = axis.cross(across);
            tip.translation() = point;
            tip.linear() = tip.linear() *
                Eigen::AngleAxisd(roll_deg * M_PI / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
            contact_path.push_back(tip * hand_to_tip.inverse());
          }
          if (contact_path.size() != contact_points.size()) continue;
          Transform above = contact_path.front();
          above.translation() += normal * height;
          planPadApproach(group, scene, above, contact_path, candidates, stats,
                          direction, tilt_deg, roll_deg, height);
        }
      }
    }

    RCLCPP_INFO(node_->get_logger(),
                "SOLDER PAD SEARCH poses=%d IK=%d collision_free=%d descents=%d route_attempts=%d candidates=%zu",
                stats.poses, stats.ik, stats.collision_free, stats.descents,
                stats.transfers, candidates.size());
    if (candidates.empty())
      throw std::runtime_error("No complete solder approach/contact plan; see SOLDER PAD SEARCH");

    scoreCandidates(candidates);
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const auto& a, const auto& b) {
                       constexpr double tcp_tie_tolerance = 0.005;  // 5 mm
                       if (std::abs(a.tcp_length - b.tcp_length) > tcp_tie_tolerance)
                         return a.tcp_length < b.tcp_length;
                       return a.score < b.score;
                     });
    const std::size_t report_count = std::min<std::size_t>(3, candidates.size());
    for (std::size_t i = 0; i < report_count; ++i) {
      const auto& option = candidates[i];
      RCLCPP_INFO(node_->get_logger(),
                  "SOLDER PAD OPTION rank=%zu score=%.4f tcp_length=%.3f joint_length=%.3f duration=%.3f smoothness=%.3f planner=%s tilt=%.0f roll=%d clearance=%.3f direction=(%.3f %.3f)",
                  i + 1, option.score, option.tcp_length, option.joint_length, option.duration,
                  option.smoothness, option.planner.c_str(),
                  option.tilt_deg, option.roll_deg,
                  option.clearance, option.direction.x(), option.direction.y());
    }

    const auto& best = candidates.front();
    const Transform above = best.above;
    RCLCPP_INFO(node_->get_logger(),
                "SOLDER %s SELECTED planner=%s tcp_length=%.3f score=%.4f tilt=%.0f roll=%d clearance=%.3f",
                circular_seam ? "CIRCULAR_SEAM" : "PAD",
                best.planner.c_str(), best.tcp_length, best.score,
                best.tilt_deg, best.roll_deg, best.clearance);
    status(circular_seam
               ? "APPROACH_CIRCULAR_SEAM " + std::to_string(component)
               : "APPROACH_PAD " + std::to_string(component) + ":" + std::to_string(pad + 1));
    run(group, best.transfer);
    run(group, best.descent);
    status(circular_seam
               ? "DWELL_CIRCULAR_SEAM " + std::to_string(component)
               : "DWELL_PAD " + std::to_string(component) + ":" + std::to_string(pad + 1));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(dwell_);
    while (std::chrono::steady_clock::now() < deadline) {
      if (!rclcpp::ok()) throw std::runtime_error("Solder dwell interrupted");
      rclcpp::sleep_for(50ms);
    }
    cartesian(group, above);
    seam_contacts.restore();
    if (circular_seam) markSeam(component, contact_points);
    else markPad(component, pad, contact_points.front());
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

  void markSeam(int component, const std::vector<Eigen::Vector3d>& points)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = node_->now();
    m.ns = "soldered_seams";
    m.id = component;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.003;
    m.color.r = 1.0f;
    m.color.g = 0.55f;
    m.color.b = 0.05f;
    m.color.a = 1.0f;
    for (const auto& point : points) {
      geometry_msgs::msg::Point marker_point;
      marker_point.x = point.x();
      marker_point.y = point.y();
      marker_point.z = point.z();
      m.points.push_back(marker_point);
    }
    marks_.markers.push_back(std::move(m));
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
  std::vector<std::string> planner_trials_;
  std::map<std::string, std::set<std::string>> available_planners_;
  std::map<std::string, std::string> available_pipeline_plugins_;
  std::set<std::string> warned_unavailable_planners_;
  bool planner_catalog_available_ = false;
  double planner_timeout_seconds_ = 1.5;
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