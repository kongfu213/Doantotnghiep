#include "ur_onrobot_mtc/tasks/solder_task.hpp"
#include "ur_onrobot_mtc/pcb_geometry.hpp"

#include <Eigen/Geometry>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <set>
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




      for (int i : selected) {

        if (completed_.count(i)) throw std::runtime_error("Component already soldered in this session");
        requireBox("component_" + std::to_string(i), 0, {pcb::component_x, pcb::component_y, pcb::component_z});
      }

      GraspContacts tool_contacts(node_);
      tool_contacts.allow("solder_tool", tool_hand->getLinkModelNamesWithCollisionGeometry());
      status("PICK_TOOL");
      if (!robot_->pick("solder_tool", solderer)) throw std::runtime_error("Tool pickup failed");
      const Transform hand_to_tool = heldTransform("solder_tool", solderer);
      const Transform hand_to_tip = hand_to_tool * Eigen::Translation3d(pcb::tip_from_handle, 0, 0);

      for (int i : selected) {
        const std::string object = "component_" + std::to_string(i);
        GraspContacts component_contacts(node_);
        component_contacts.allow(object, holder_hand->getLinkModelNamesWithCollisionGeometry());
        status("PICK_COMPONENT " + std::to_string(i));
        if (!robot_->pick(object, holder)) throw std::runtime_error("Component pickup failed");
        const Transform hand_to_component = heldTransform(object, holder);
        const Transform target = board * Eigen::Translation3d(vector(pcb::componentOffset(i - 1)));
        Transform above = target;
        above.translation() += board.linear().col(2) * clearance_;
        move(holder_group, above * hand_to_component.inverse());
        cartesian(holder_group, target * hand_to_component.inverse());
        // The component remains attached and the holder receives no commands
        // during the two pad contacts. The tool planner sees the whole robot.
        status("HOLD_COMPONENT " + std::to_string(i));

        for (std::size_t pad = 0; pad < 2; ++pad) {
          solderPad(tool_group, board, hand_to_tip, i, pad);
        }

        // Only release after BOTH pad dwells and the final tool retreat succeed.
        status("RELEASE_COMPONENT " + std::to_string(i));
        if (!robot_->releaseHeldObject(object)) throw std::runtime_error("Component release failed");
        completed_.insert(i);
        cartesian(holder_group, above * hand_to_component.inverse());
        component_contacts.restore();
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
    return code == moveit::core::MoveItErrorCode::SUCCESS;
  }

  void run(Group& group, const Group::Plan& plan)
  {
    if (!rclcpp::ok() || group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Motion execution failed; no automatic retry");
  }

  void move(Group& group, const Transform& target)
  {
    Group::Plan result;
    if (!plan(group, target, result)) throw std::runtime_error("No collision-free transfer plan");
    run(group, result);
  }

  bool planCartesian(Group& group, const Transform& target,
                     const moveit::core::RobotState& start, Group::Plan& result)
  {
    group.setStartState(start);
    moveit_msgs::msg::RobotTrajectory message;
    const double fraction = group.computeCartesianPath({pose(target)}, 0.002, 1.5, message, true);
    if (!std::isfinite(fraction) || fraction < 0.999 || message.joint_trajectory.points.empty())
      return false;
    robot_trajectory::RobotTrajectory trajectory(group.getRobotModel(), group.getName());
    trajectory.setRobotTrajectoryMsg(start, message);
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    if (!timing.computeTimeStamps(trajectory, 0.10, 0.10)) return false;
    trajectory.getRobotTrajectoryMsg(result.trajectory_);
    return true;
  }

  void cartesian(Group& group, const Transform& target)
  {
    auto start = group.getCurrentState(3.0);
    if (!start) throw std::runtime_error("Missing current robot state");
    // Use a diff start state, retaining attached objects from the server scene.
    group.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory message;
    const double fraction = group.computeCartesianPath({pose(target)}, 0.002, 1.5, message, true);
    if (!std::isfinite(fraction) || fraction < 0.999 || message.joint_trajectory.points.empty())
      throw std::runtime_error("Incomplete Cartesian contact/retreat path; refusing partial execution");
    robot_trajectory::RobotTrajectory trajectory(group.getRobotModel(), group.getName());
    trajectory.setRobotTrajectoryMsg(*start, message);
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    if (!timing.computeTimeStamps(trajectory, 0.10, 0.10))
      throw std::runtime_error("Cartesian time parameterization failed");
    Group::Plan result;
    trajectory.getRobotTrajectoryMsg(result.trajectory_);
    run(group, result);
  }

  void solderPad(Group& group, const Transform& board, const Transform& hand_to_tip,
                  int component, std::size_t pad)
  {
    const Eigen::Vector3d pad_position = board * vector(pcb::padOffset(component - 1, pad));
    const Eigen::Vector3d normal = board.linear().col(2);
    Group::Plan approach_plan, contact_plan;
    Transform contact = Transform::Identity(), above = Transform::Identity();
    bool found = false;
    // The tool's local +X is its long axis. Point it toward the pad from the
    // outside of the component; try roll variants without moving either hand.
    for (double tilt_deg : {30.0, 45.0, 60.0}) {
      const double angle = tilt_deg * M_PI / 180.0;
      const double sign = pad == 0 ? 1.0 : -1.0;
      const Eigen::Vector3d axis = board.linear() * Eigen::Vector3d(sign * std::sin(angle), 0, -std::cos(angle));
      const Eigen::Vector3d across = board.linear().col(1);
      Transform tip = Transform::Identity();
      tip.linear().col(0) = axis;
      tip.linear().col(1) = across;
      tip.linear().col(2) = axis.cross(across);
      tip.translation() = pad_position + normal * 0.0005;
      for (int roll = 0; roll < 4; ++roll) {
        Transform oriented = tip;
        oriented.linear() = tip.linear() * Eigen::AngleAxisd(roll * M_PI / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
        contact = oriented * hand_to_tip.inverse();
        above = contact;
        above.translation() += normal * clearance_;
        if (plan(group, above, approach_plan)) {
          auto start = group.getCurrentState(3.0);
          if (!start) throw std::runtime_error("Missing robot state");
          moveit::core::robotStateMsgToRobotState(approach_plan.start_state_, *start);
          robot_trajectory::RobotTrajectory transfer(group.getRobotModel(), group.getName());
          transfer.setRobotTrajectoryMsg(*start, approach_plan.trajectory_);
          if (transfer.getWayPointCount() &&
              planCartesian(group, contact, transfer.getLastWayPoint(), contact_plan)) {
            found = true;
            break;
          }
        }
      }
      if (found) break;
    }
    if (!found) throw std::runtime_error("No tool approach pose with the other arm holding the component");
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