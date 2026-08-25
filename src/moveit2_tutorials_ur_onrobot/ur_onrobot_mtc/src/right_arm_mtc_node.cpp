#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("right_mtc_node");
namespace mtc = moveit::task_constructor;

class RightMTCTaskNode
{
public:
  explicit RightMTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr RightMTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

RightMTCTaskNode::RightMTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("right_mtc_node", options) }
{
}

mtc::Task RightMTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("right_arm_task");
  task.loadRobotModel(node_);

  // Thông số Tay Phải chuẩn hóa
  const std::string arm_group = "right_ur_onrobot_manipulator";
  const std::string hand_group = "right_ur_onrobot_gripper";
  const std::string hand_frame = "right_gripper_tcp";
  const std::string object_name = "right_object";
  const std::string table_name = "table";

  task.setProperty("group", arm_group);
  task.setProperty("eef", hand_group);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;
  auto current_state = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = current_state.get();
  task.add(std::move(current_state));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setProperty("planning_pipeline", "ompl");
  sampling_planner->setPlannerId("RRTConnectkConfigDefault");

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(0.001);

  const auto* hand_jmg = task.getRobotModel()->getJointModelGroup(hand_group);
  if (!hand_jmg) {
    throw std::runtime_error("Không tìm thấy JointModelGroup: " + hand_group);
  }
  const auto hand_links = hand_jmg->getLinkModelNamesWithCollisionGeometry();

  // 1. Mở tay kẹp
  auto open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  open_hand->setGroup(hand_group);
  open_hand->setGoal("open");
  task.add(std::move(open_hand));

  // 2. Di chuyển đến điểm gắp
  auto move_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick", mtc::stages::Connect::GroupPlannerVector{ { arm_group, sampling_planner } });
  move_pick->setTimeout(15.0);
  move_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(move_pick));

  mtc::Stage* attach_stage = nullptr;

  // 3. Chuỗi Pick
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // 3.1 Cho phép va chạm (SỬA LỖI VA CHẠM KHỚP NỔI TAY PHẢI)
    {
      auto allow_coll = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      allow_coll->allowCollisions(object_name, hand_links, true);
      allow_coll->allowCollisions(hand_links, hand_links, true);
      grasp->insert(std::move(allow_coll));
    }

    // 3.2 Tiếp cận vật
    {
      auto approach = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      approach->properties().set("marker_ns", "approach_object");
      approach->properties().set("link", hand_frame);
      approach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      approach->setMinMaxDistance(0.02, 0.15);

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = hand_frame;
      direction.vector.z = 1.0;
      approach->setDirection(direction);

      grasp->insert(std::move(approach));
    }

    // 3.3 Tạo Pose gắp + IK
    {
      auto gen_pose = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      gen_pose->properties().configureInitFrom(mtc::Stage::PARENT);
      gen_pose->properties().set("marker_ns", "grasp_pose");
      gen_pose->setPreGraspPose("open");
      gen_pose->setObject(object_name);
      gen_pose->setAngleDelta(M_PI / 18.0);
      gen_pose->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d grasp_tf = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
      grasp_tf.linear() = q.matrix();
      grasp_tf.translation().z() = 0.015;

      auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(gen_pose));
      ik_wrapper->setMaxIKSolutions(8);
      ik_wrapper->setMinSolutionDistance(0.1);
      ik_wrapper->setIKFrame(grasp_tf, hand_frame);
      ik_wrapper->setIgnoreCollisions(true);
      ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

      grasp->insert(std::move(ik_wrapper));
    }

    // 3.4 Đóng tay
    {
      auto close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      close_hand->setGroup(hand_group);
      close_hand->setGoal("closed");
      grasp->insert(std::move(close_hand));
    }

    // 3.5 Gắn vật
    {
      auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      attach->attachObject(object_name, hand_frame);
      attach_stage = attach.get();
      grasp->insert(std::move(attach));
    }

    // 3.6 Nâng vật
    {
      auto lift = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      lift->properties().set("marker_ns", "lift_object");
      lift->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      lift->setMinMaxDistance(0.05, 0.15);
      lift->setIKFrame(hand_frame);

      geometry_msgs::msg::Vector3Stamped lift_direction;
      lift_direction.header.frame_id = "world";
      lift_direction.vector.z = 1.0;
      lift->setDirection(lift_direction);

      grasp->insert(std::move(lift));
    }

    task.add(std::move(grasp));
  }

  // 4. Di chuyển sang nơi đặt
  {
    auto move_place = std::make_unique<mtc::stages::Connect>(
        "move to place", mtc::stages::Connect::GroupPlannerVector{ { arm_group, sampling_planner } });
    move_place->setTimeout(15.0);
    move_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(move_place));
  }

  // 5. Chuỗi Place
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // 5.1 Cho phép va chạm với bàn
    {
      auto allow_table = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,table)");
      allow_table->allowCollisions(object_name, table_name, true);
      allow_table->allowCollisions(table_name, hand_links, true);
      place->insert(std::move(allow_table));
    }

    // 5.2 Tọa độ thả vật (TAY PHẢI - Đặt vị trí đối xứng x = -0.1)
    {
      auto gen_place = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      gen_place->properties().configureInitFrom(mtc::Stage::PARENT);
      gen_place->properties().set("marker_ns", "place_pose");
      gen_place->setObject(object_name);

      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.header.frame_id = "world";
      target_pose.pose.position.x = -0.1;
      target_pose.pose.position.y = 0.4;
      target_pose.pose.position.z = 0.052;
      target_pose.pose.orientation.w = 1.0;

      gen_place->setPose(target_pose);
      gen_place->setMonitoredStage(attach_stage);

      auto ik_place = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(gen_place));
      ik_place->setMaxIKSolutions(8);
      ik_place->setMinSolutionDistance(0.1);
      ik_place->setIKFrame(object_name);
      ik_place->setIgnoreCollisions(true);
      ik_place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      ik_place->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

      place->insert(std::move(ik_place));
    }

    // 5.3 Mở tay kẹp
    {
      auto open_hand_post = std::make_unique<mtc::stages::MoveTo>("open hand post", interpolation_planner);
      open_hand_post->setGroup(hand_group);
      open_hand_post->setGoal("open");
      place->insert(std::move(open_hand_post));
    }

    // 5.4 Thả vật
    {
      auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      detach->detachObject(object_name, hand_frame);
      place->insert(std::move(detach));
    }

    // 5.5 Rút tay lên
    {
      auto retreat = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      retreat->properties().set("marker_ns", "retreat");
      retreat->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      retreat->setMinMaxDistance(0.05, 0.15);
      retreat->setIKFrame(hand_frame);

      geometry_msgs::msg::Vector3Stamped retreat_direction;
      retreat_direction.header.frame_id = "world";
      retreat_direction.vector.z = 1.0;
      retreat->setDirection(retreat_direction);

      place->insert(std::move(retreat));
    }

    task.add(std::move(place));
  }

  // 6. Trở về vị trí ban đầu
  {
    auto home = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    home->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    home->setGoal("right_test_configuration");
    home->setTimeout(15.0);
    task.add(std::move(home));
  }

  return task;
}

void RightMTCTaskNode::doTask()
{
  task_ = createTask();
  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, "========= LỖI INIT MTC STAGE (RIGHT) =========\n" << e);
    return;
  }

  if (!task_.plan(5)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Right Task planning failed");
    return;
  }

  task_.introspection().publishSolution(*task_.solutions().front());
  const auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(LOGGER, "Right Task execution failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "Right Task executed successfully");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<RightMTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  std::this_thread::sleep_for(std::chrono::seconds(2));
  mtc_task_node->doTask();

  rclcpp::shutdown();
  spin_thread->join();
  return 0;
}