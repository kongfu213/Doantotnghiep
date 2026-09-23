#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace mtc = moveit::task_constructor;

class MTCPersistentDemo
{
public:
  explicit MTCPersistentDemo(const rclcpp::NodeOptions& options)
  : node_(std::make_shared<rclcpp::Node>("mtc_persistent_demo", options))
  {
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface() const
  {
    return node_->get_node_base_interface();
  }

  void initialize()
  {
    // Reuse the same solver objects for every command.
    sampling_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    sampling_planner_->setProperty("planning_pipeline", "ompl");
    sampling_planner_->setPlannerId("RRTConnectkConfigDefault");

    interpolation_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner_->setMaxVelocityScalingFactor(0.5);
    cartesian_planner_->setMaxAccelerationScalingFactor(0.5);
    cartesian_planner_->setStepSize(0.005);

    command_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot_command", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          handleCommand(msg->data);
        });

    setupDemoScene();

    RCLCPP_INFO(node_->get_logger(), "============================================");
    RCLCPP_INFO(node_->get_logger(), " MTC PERSISTENT DEMO READY");
    RCLCPP_INFO(node_->get_logger(), " Node stays alive; MTC tasks are created per command");
    RCLCPP_INFO(node_->get_logger(), " pick  A left|right");
    RCLCPP_INFO(node_->get_logger(), " place A x y z [yaw_deg]");
    RCLCPP_INFO(node_->get_logger(), " reset_scene");
    RCLCPP_INFO(node_->get_logger(), " status");
    RCLCPP_INFO(node_->get_logger(), "============================================");
  }

private:
  struct ArmConfig
  {
    std::string arm_group;
    std::string hand_group;
    std::string hand_frame;
  };

  static std::string normalizeObjectName(std::string name)
  {
    if (name == "A" || name == "a") return "stack_A";
    if (name == "B" || name == "b") return "stack_B";
    if (name == "C" || name == "c") return "stack_C";
    if (name == "D" || name == "d") return "stack_D";
    return name;
  }

  static std::string normalizeArm(std::string arm)
  {
    std::transform(arm.begin(), arm.end(), arm.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (arm == "l") return "left";
    if (arm == "r") return "right";
    return arm;
  }

  ArmConfig armConfig(const std::string& arm) const
  {
    if (arm == "left") {
      return {"left_ur_onrobot_manipulator",
              "left_ur_onrobot_gripper",
              "left_gripper_tcp"};
    }
    if (arm == "right") {
      return {"right_ur_onrobot_manipulator",
              "right_ur_onrobot_gripper",
              "right_gripper_tcp"};
    }
    throw std::runtime_error("arm must be left or right");
  }

  void setupDemoScene()
  {
    moveit::planning_interface::PlanningSceneInterface psi;

    std::vector<std::string> remove_ids = {
        "table", "stack_A", "stack_B", "stack_C", "stack_D"};
    psi.removeCollisionObjects(remove_ids);

    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = "world";
    table.id = "table";
    table.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive table_box;
    table_box.type = shape_msgs::msg::SolidPrimitive::BOX;
    table_box.dimensions = {1.0, 1.0, 0.10};

    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.0;
    table_pose.position.y = 0.0;
    // Top surface = -0.003 m.
    table_pose.position.z = -0.053;

    table.primitives.push_back(table_box);
    table.primitive_poses.push_back(table_pose);

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(table);

    const double cube_size = 0.04;
    const double cube_z = 0.0175;

    auto make_cube = [&](const std::string& id, double x, double y) {
      moveit_msgs::msg::CollisionObject obj;
      obj.header.frame_id = "world";
      obj.id = id;
      obj.operation = moveit_msgs::msg::CollisionObject::ADD;

      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::BOX;
      box.dimensions = {cube_size, cube_size, cube_size};

      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = cube_z;

      obj.primitives.push_back(box);
      obj.primitive_poses.push_back(pose);
      return obj;
    };

    objects.push_back(make_cube("stack_A",  0.30,  0.25));
    objects.push_back(make_cube("stack_B", -0.25,  0.25));
    objects.push_back(make_cube("stack_C",  0.30, -0.25));
    objects.push_back(make_cube("stack_D", -0.25, -0.25));

    psi.applyCollisionObjects(objects);
    RCLCPP_INFO(node_->get_logger(), "Demo scene created: table + stack_A/B/C/D");
  }

  mtc::Task createPickTask(const std::string& object_name, const std::string& arm)
  {
    const ArmConfig cfg = armConfig(arm);

    mtc::Task task;
    task.stages()->setName("pick_" + object_name + "_" + arm);
    task.loadRobotModel(node_);

    task.setProperty("group", cfg.arm_group);
    task.setProperty("eef", cfg.hand_group);
    task.setProperty("ik_frame", cfg.hand_frame);

    const auto* hand_jmg = task.getRobotModel()->getJointModelGroup(cfg.hand_group);
    if (!hand_jmg) {
      throw std::runtime_error("Missing JointModelGroup: " + cfg.hand_group);
    }
    const auto hand_links = hand_jmg->getLinkModelNamesWithCollisionGeometry();

    mtc::Stage* current_state_ptr = nullptr;
    {
      auto current = std::make_unique<mtc::stages::CurrentState>("current");
      current_state_ptr = current.get();
      task.add(std::move(current));
    }

    // The robot bases are intentionally mounted on / intersect the table model.
    // Ignore only these fixed base-table contacts so that ComputeIK does not
    // reject every grasp candidate because of a permanent mounting collision.
    {
      auto allow_mount = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow base-table mounting contact");
      allow_mount->allowCollisions("table", "left_onrobot_base_link", true);
      allow_mount->allowCollisions("table", "right_onrobot_base_link", true);
      task.add(std::move(allow_mount));
    }

    {
      auto open = std::make_unique<mtc::stages::MoveTo>(
          "open hand", interpolation_planner_);
      open->setGroup(cfg.hand_group);
      open->setGoal("open");
      task.add(std::move(open));
    }

    {
      auto connect = std::make_unique<mtc::stages::Connect>(
          "move to pick",
          mtc::stages::Connect::GroupPlannerVector{
              {cfg.arm_group, sampling_planner_}});
      // Trial value: avoid waiting 15 s per failed connection.
      connect->setTimeout(4.0);
      connect->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(connect));
    }

    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(
        mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      auto approach = std::make_unique<mtc::stages::MoveRelative>(
          "approach object", cartesian_planner_);
      approach->properties().set("marker_ns", "approach_object");
      approach->properties().set("link", cfg.hand_frame);
      approach->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      approach->setMinMaxDistance(0.03, 0.10);

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = cfg.hand_frame;
      direction.vector.z = 1.0;
      approach->setDirection(direction);
      grasp->insert(std::move(approach));
    }

    {
      auto generator = std::make_unique<mtc::stages::GenerateGraspPose>(
          "generate grasp pose");
      generator->properties().configureInitFrom(mtc::Stage::PARENT);
      generator->properties().set("marker_ns", "grasp_pose");
      generator->setPreGraspPose("open");
      generator->setObject(object_name);

      // IMPORTANT: coarse 90-degree sampling first, not 5/10-degree brute force.
      generator->setAngleDelta(M_PI / 2.0);
      generator->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q =
          Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
      grasp_tf.linear() = q.matrix();
      grasp_tf.translation().z() = 0.015;

      auto ik = std::make_unique<mtc::stages::ComputeIK>(
          "grasp pose IK", std::move(generator));
      ik->setMaxIKSolutions(4);
      ik->setMinSolutionDistance(0.2);
      ik->setIKFrame(grasp_tf, cfg.hand_frame);
      ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      grasp->insert(std::move(ik));
    }

    {
      auto allow = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow collision hand-object");
      allow->allowCollisions(object_name, hand_links, true);
      grasp->insert(std::move(allow));
    }

    {
      auto close = std::make_unique<mtc::stages::MoveTo>(
          "close hand", interpolation_planner_);
      close->setGroup(cfg.hand_group);
      close->setGoal("closed");
      grasp->insert(std::move(close));
    }

    {
      auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      attach->attachObject(object_name, cfg.hand_frame);
      grasp->insert(std::move(attach));
    }

    {
      auto lift = std::make_unique<mtc::stages::MoveRelative>(
          "lift object", cartesian_planner_);
      lift->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      lift->properties().set("marker_ns", "lift_object");
      lift->setIKFrame(cfg.hand_frame);
      lift->setMinMaxDistance(0.05, 0.10);

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "world";
      direction.vector.z = 1.0;
      lift->setDirection(direction);
      grasp->insert(std::move(lift));
    }

    task.add(std::move(grasp));
    return task;
  }

  mtc::Task createPlaceTask(const std::string& object_name,
                            const std::string& arm,
                            double x, double y, double z,
                            double yaw_deg)
  {
    const ArmConfig cfg = armConfig(arm);

    mtc::Task task;
    task.stages()->setName("place_" + object_name + "_" + arm);
    task.loadRobotModel(node_);

    task.setProperty("group", cfg.arm_group);
    task.setProperty("eef", cfg.hand_group);
    task.setProperty("ik_frame", cfg.hand_frame);

    const auto* hand_jmg = task.getRobotModel()->getJointModelGroup(cfg.hand_group);
    if (!hand_jmg) {
      throw std::runtime_error("Missing JointModelGroup: " + cfg.hand_group);
    }
    const auto hand_links = hand_jmg->getLinkModelNamesWithCollisionGeometry();

    mtc::Stage* current_state_ptr = nullptr;
    {
      auto current = std::make_unique<mtc::stages::CurrentState>("current attached state");
      current_state_ptr = current.get();
      task.add(std::move(current));
    }

    {
      auto allow_mount = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow base-table mounting contact");
      allow_mount->allowCollisions("table", "left_onrobot_base_link", true);
      allow_mount->allowCollisions("table", "right_onrobot_base_link", true);
      task.add(std::move(allow_mount));
    }

    {
      auto allow = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow support contact");
      allow->allowCollisions(object_name, "table", true);
      allow->allowCollisions("table", hand_links, true);
      task.add(std::move(allow));
    }

    {
      auto connect = std::make_unique<mtc::stages::Connect>(
          "move to place",
          mtc::stages::Connect::GroupPlannerVector{
              {cfg.arm_group, sampling_planner_}});
      connect->setTimeout(4.0);
      connect->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(connect));
    }

    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(
        mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      auto generator = std::make_unique<mtc::stages::GeneratePlacePose>(
          "generate place pose");
      generator->properties().configureInitFrom(mtc::Stage::PARENT);
      generator->properties().set("marker_ns", "place_pose");
      generator->setObject(object_name);
      generator->setMonitoredStage(current_state_ptr);

      geometry_msgs::msg::PoseStamped target;
      target.header.frame_id = "world";
      target.pose.position.x = x;
      target.pose.position.y = y;
      target.pose.position.z = z;
      const double yaw = yaw_deg * M_PI / 180.0;
      target.pose.orientation.x = 0.0;
      target.pose.orientation.y = 0.0;
      target.pose.orientation.z = std::sin(yaw / 2.0);
      target.pose.orientation.w = std::cos(yaw / 2.0);
      generator->setPose(target);

      auto ik = std::make_unique<mtc::stages::ComputeIK>(
          "place pose IK", std::move(generator));
      ik->setMaxIKSolutions(8);
      ik->setMinSolutionDistance(0.2);
      // The target pose is the pose of the attached object itself.
      ik->setIKFrame(object_name);
      ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(ik));
    }

    {
      auto open = std::make_unique<mtc::stages::MoveTo>(
          "open hand", interpolation_planner_);
      open->setGroup(cfg.hand_group);
      open->setGoal("open");
      place->insert(std::move(open));
    }

    {
      auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      detach->detachObject(object_name, cfg.hand_frame);
      place->insert(std::move(detach));
    }

    {
      auto restore = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "restore collision rules");
      restore->allowCollisions(object_name, hand_links, false);
      restore->allowCollisions(object_name, "table", false);
      restore->allowCollisions("table", hand_links, false);
      place->insert(std::move(restore));
    }

    {
      auto retreat = std::make_unique<mtc::stages::MoveRelative>(
          "retreat", cartesian_planner_);
      retreat->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      retreat->properties().set("marker_ns", "retreat");
      retreat->setIKFrame(cfg.hand_frame);
      retreat->setMinMaxDistance(0.05, 0.10);

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "world";
      direction.vector.z = 1.0;
      retreat->setDirection(direction);
      place->insert(std::move(retreat));
    }

    task.add(std::move(place));
    return task;
  }

  bool planAndExecute(mtc::Task& task)
  {
    try {
      task.init();
    } catch (const mtc::InitStageException& e) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), e);
      return false;
    }

    // Keep this deliberately small for the trial.
    if (!task.plan(3)) {
      RCLCPP_ERROR(node_->get_logger(), "MTC planning failed");
      task.printState();
      return false;
    }

    task.introspection().publishSolution(*task.solutions().front());
    const auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "MTC execution failed, code=%d", result.val);
      return false;
    }
    return true;
  }

  bool runPick(const std::string& object_name, const std::string& arm)
  {
    RCLCPP_INFO(node_->get_logger(), "PICK START object=%s arm=%s",
                object_name.c_str(), arm.c_str());
    auto task = createPickTask(object_name, arm);
    const bool ok = planAndExecute(task);
    if (ok) {
      held_by_[object_name] = arm;
      RCLCPP_INFO(node_->get_logger(), "PICK SUCCESS object=%s arm=%s",
                  object_name.c_str(), arm.c_str());
    }
    return ok;
  }

  bool runPlace(const std::string& object_name,
                double x, double y, double z, double yaw_deg)
  {
    const auto it = held_by_.find(object_name);
    if (it == held_by_.end()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "PLACE rejected: %s was not picked by this demo node",
                   object_name.c_str());
      return false;
    }

    const std::string arm = it->second;
    RCLCPP_INFO(node_->get_logger(),
                "PLACE START object=%s arm=%s target=(%.3f %.3f %.3f) yaw=%.1f",
                object_name.c_str(), arm.c_str(), x, y, z, yaw_deg);

    auto task = createPlaceTask(object_name, arm, x, y, z, yaw_deg);
    const bool ok = planAndExecute(task);
    if (ok) {
      held_by_.erase(it);
      RCLCPP_INFO(node_->get_logger(), "PLACE SUCCESS object=%s", object_name.c_str());
    }
    return ok;
  }

  void handleCommand(const std::string& line)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);

    std::istringstream iss(line);
    std::string command;
    iss >> command;
    if (command.empty()) return;

    try {
      if (command == "pick") {
        std::string object_token;
        std::string arm_token;
        iss >> object_token >> arm_token;
        if (object_token.empty() || arm_token.empty()) {
          RCLCPP_ERROR(node_->get_logger(), "Usage: pick A left|right");
          return;
        }

        const std::string object_name = normalizeObjectName(object_token);
        const std::string arm = normalizeArm(arm_token);
        if (arm != "left" && arm != "right") {
          RCLCPP_ERROR(node_->get_logger(),
                       "This trial intentionally disables AUTO selector. Use left or right.");
          return;
        }
        runPick(object_name, arm);
        return;
      }

      if (command == "place") {
        std::string object_token;
        double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
        if (!(iss >> object_token >> x >> y >> z)) {
          RCLCPP_ERROR(node_->get_logger(), "Usage: place A x y z [yaw_deg]");
          return;
        }
        if (!(iss >> yaw)) yaw = 0.0;

        runPlace(normalizeObjectName(object_token), x, y, z, yaw);
        return;
      }

      if (command == "reset_scene") {
        held_by_.clear();
        setupDemoScene();
        return;
      }

      if (command == "status") {
        if (held_by_.empty()) {
          RCLCPP_INFO(node_->get_logger(), "STATUS: no tracked attached objects");
        } else {
          for (const auto& [obj, arm] : held_by_) {
            RCLCPP_INFO(node_->get_logger(), "STATUS: %s held by %s",
                        obj.c_str(), arm.c_str());
          }
        }
        return;
      }

      if (command == "help") {
        RCLCPP_INFO(node_->get_logger(), "pick A left|right");
        RCLCPP_INFO(node_->get_logger(), "place A x y z [yaw_deg]");
        RCLCPP_INFO(node_->get_logger(), "reset_scene | status");
        return;
      }

      RCLCPP_ERROR(node_->get_logger(), "Unknown command: %s", command.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "Command exception: %s", e.what());
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

  std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner_;
  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> interpolation_planner_;
  std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;

  std::unordered_map<std::string, std::string> held_by_;
  std::mutex command_mutex_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto server = std::make_shared<MTCPersistentDemo>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server->getNodeBaseInterface());

  std::thread spin_thread([&executor]() { executor.spin(); });

  // Let subscriptions/services settle before touching the PlanningScene.
  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  server->initialize();

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
