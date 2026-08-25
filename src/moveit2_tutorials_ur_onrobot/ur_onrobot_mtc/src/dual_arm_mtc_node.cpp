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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dual_arm_mtc_node");
namespace mtc = moveit::task_constructor;

class DualArmMTCTaskNode
{
public:
  explicit DualArmMTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();

private:
  mtc::Task createUnifiedDualArmTask();
  void addArmPickPlaceSequence(mtc::Task& task, 
                               mtc::Stage* current_state_ptr,
                               const std::string& arm_prefix, 
                               const std::string& object_name, 
                               double place_x,
                               std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner,
                               std::shared_ptr<mtc::solvers::JointInterpolationPlanner> interpolation_planner,
                               std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner);

  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr DualArmMTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

DualArmMTCTaskNode::DualArmMTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("dual_arm_mtc_node", options) }
{
}

void DualArmMTCTaskNode::addArmPickPlaceSequence(
    mtc::Task& task, 
    mtc::Stage* current_state_ptr,
    const std::string& arm_prefix, 
    const std::string& object_name, 
    double place_x,
    std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner,
    std::shared_ptr<mtc::solvers::JointInterpolationPlanner> interpolation_planner,
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner)
{
  const std::string arm_group = arm_prefix + "_ur_onrobot_manipulator";
  const std::string hand_group = arm_prefix + "_ur_onrobot_gripper";
  const std::string hand_frame = arm_prefix + "_gripper_tcp";
  const std::string home_pose = arm_prefix + "_test_configuration";
  const std::string table_name = "table";

  const auto* hand_jmg = task.getRobotModel()->getJointModelGroup(hand_group);
  if (!hand_jmg) {
    throw std::runtime_error("Không tìm thấy JointModelGroup: " + hand_group);
  }
  const auto hand_links = hand_jmg->getLinkModelNamesWithCollisionGeometry();

  // 1. Mở tay kẹp
  auto open_hand = std::make_unique<mtc::stages::MoveTo>("open hand " + arm_prefix, interpolation_planner);
  open_hand->setGroup(hand_group);
  open_hand->setGoal("open");
  task.add(std::move(open_hand));

  // 2. Di chuyển đến điểm gắp
  auto move_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick " + arm_prefix, mtc::stages::Connect::GroupPlannerVector{ { arm_group, sampling_planner } });
  move_pick->setTimeout(15.0);
  move_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(move_pick));

  mtc::Stage* attach_stage = nullptr;

  // 3. Chuỗi Pick
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object " + arm_prefix);
    grasp->setProperty("group", arm_group);
    grasp->setProperty("eef", hand_group);
    grasp->setProperty("ik_frame", hand_frame);

    // Bỏ qua va chạm
    auto allow_coll = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision " + arm_prefix);
    allow_coll->allowCollisions(object_name, hand_links, true);
    allow_coll->allowCollisions(hand_links, hand_links, true);
    allow_coll->allowCollisions(table_name, hand_links, true);
    grasp->insert(std::move(allow_coll));

    // Tiếp cận vật
    auto approach = std::make_unique<mtc::stages::MoveRelative>("approach object " + arm_prefix, cartesian_planner);
    approach->properties().set("marker_ns", "approach_" + arm_prefix);
    approach->properties().set("link", hand_frame);
    approach->setProperty("group", arm_group);
    approach->setMinMaxDistance(0.05, 0.15);

    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = hand_frame;
    direction.vector.z = 1.0;
    approach->setDirection(direction);
    grasp->insert(std::move(approach));

    // Pose gắp + IK
    auto gen_pose = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose " + arm_prefix);
    gen_pose->properties().set("marker_ns", "grasp_pose_" + arm_prefix);
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

    auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK " + arm_prefix, std::move(gen_pose));
    ik_wrapper->setMaxIKSolutions(8);
    ik_wrapper->setMinSolutionDistance(0.1);
    ik_wrapper->setIKFrame(grasp_tf, hand_frame);
    ik_wrapper->setIgnoreCollisions(true);
    ik_wrapper->setProperty("group", arm_group);
    ik_wrapper->setProperty("eef", hand_group);
    ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    grasp->insert(std::move(ik_wrapper));

    // Đóng tay
    auto close_hand = std::make_unique<mtc::stages::MoveTo>("close hand " + arm_prefix, interpolation_planner);
    close_hand->setGroup(hand_group);
    close_hand->setGoal("closed");
    grasp->insert(std::move(close_hand));

    // Gắn vật
    auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object " + arm_prefix);
    attach->attachObject(object_name, hand_frame);
    attach_stage = attach.get();
    grasp->insert(std::move(attach));

    // Nâng vật
    auto lift = std::make_unique<mtc::stages::MoveRelative>("lift object " + arm_prefix, cartesian_planner);
    lift->properties().set("marker_ns", "lift_" + arm_prefix);
    lift->setProperty("group", arm_group);
    lift->setMinMaxDistance(0.05, 0.15);
    lift->setIKFrame(hand_frame);

    geometry_msgs::msg::Vector3Stamped lift_direction;
    lift_direction.header.frame_id = "world";
    lift_direction.vector.z = 1.0;
    lift->setDirection(lift_direction);
    grasp->insert(std::move(lift));

    task.add(std::move(grasp));
  }

  // 4. Di chuyển sang vị trí đặt
  auto move_place = std::make_unique<mtc::stages::Connect>(
      "move to place " + arm_prefix, mtc::stages::Connect::GroupPlannerVector{ { arm_group, sampling_planner } });
  move_place->setTimeout(15.0);
  move_place->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(move_place));

  // 5. Chuỗi Place
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object " + arm_prefix);
    place->setProperty("group", arm_group);
    place->setProperty("eef", hand_group);
    place->setProperty("ik_frame", hand_frame);

    auto allow_table = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision table " + arm_prefix);
    allow_table->allowCollisions(object_name, table_name, true);
    allow_table->allowCollisions(table_name, hand_links, true);
    place->insert(std::move(allow_table));

    auto gen_place = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose " + arm_prefix);
    gen_place->properties().set("marker_ns", "place_pose_" + arm_prefix);
    gen_place->setObject(object_name);

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "world";
    target_pose.pose.position.x = place_x;
    target_pose.pose.position.y = 0.4;
    target_pose.pose.position.z = 0.45;
    target_pose.pose.orientation.w = 1.0;

    gen_place->setPose(target_pose);
    gen_place->setMonitoredStage(attach_stage);

    auto ik_place = std::make_unique<mtc::stages::ComputeIK>("place pose IK " + arm_prefix, std::move(gen_place));
    ik_place->setMaxIKSolutions(8);
    ik_place->setMinSolutionDistance(0.1);
    ik_place->setIKFrame(object_name);
    ik_place->setIgnoreCollisions(true);
    ik_place->setProperty("group", arm_group);
    ik_place->setProperty("eef", hand_group);
    ik_place->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    place->insert(std::move(ik_place));

    auto open_hand_post = std::make_unique<mtc::stages::MoveTo>("open hand post " + arm_prefix, interpolation_planner);
    open_hand_post->setGroup(hand_group);
    open_hand_post->setGoal("open");
    place->insert(std::move(open_hand_post));

    auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object " + arm_prefix);
    detach->detachObject(object_name, hand_frame);
    place->insert(std::move(detach));

    auto retreat = std::make_unique<mtc::stages::MoveRelative>("retreat " + arm_prefix, cartesian_planner);
    retreat->properties().set("marker_ns", "retreat_" + arm_prefix);
    retreat->setProperty("group", arm_group);
    retreat->setMinMaxDistance(0.05, 0.15);
    retreat->setIKFrame(hand_frame);

    geometry_msgs::msg::Vector3Stamped retreat_direction;
    retreat_direction.header.frame_id = "world";
    retreat_direction.vector.z = 1.0;
    retreat->setDirection(retreat_direction);
    place->insert(std::move(retreat));

    task.add(std::move(place));
  }

  // 6. Trở về vị trí Home
  auto home = std::make_unique<mtc::stages::MoveTo>("return home " + arm_prefix, sampling_planner);
  home->setProperty("group", arm_group);
  home->setGoal(home_pose);
  home->setTimeout(15.0);
  task.add(std::move(home));
}

mtc::Task DualArmMTCTaskNode::createUnifiedDualArmTask()
{
  mtc::Task task;
  task.stages()->setName("unified_dual_arm_task");
  task.loadRobotModel(node_);

  auto current_state = std::make_unique<mtc::stages::CurrentState>("current_state");
  auto* current_state_ptr = current_state.get();
  task.add(std::move(current_state));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setProperty("planning_pipeline", "ompl");
  sampling_planner->setPlannerId("RRTConnectkConfigDefault");

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(0.002);

  // Thêm toàn bộ chuỗi Tay Trái
  addArmPickPlaceSequence(task, current_state_ptr, "left", "left_object", 0.1, 
                          sampling_planner, interpolation_planner, cartesian_planner);

  // Thêm tiếp toàn bộ chuỗi Tay Phải vào chung 1 Task
  addArmPickPlaceSequence(task, current_state_ptr, "right", "right_object", -0.1, 
                          sampling_planner, interpolation_planner, cartesian_planner);

  return task;
}

void DualArmMTCTaskNode::doTask()
{
  RCLCPP_INFO(LOGGER, "=== LẬP KẾ HOẠCH HỢP NHẤT CHO CẢ 2 TAY (SINGLE TASK) ===");
  auto dual_task = createUnifiedDualArmTask();

  try {
    dual_task.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, "Lỗi Init MTC Task: " << e);
    return;
  }

  if (dual_task.plan(5)) {
    RCLCPP_INFO(LOGGER, "Lập kế hoạch thành công! Đang gửi quỹ đạo cho cả 2 tay...");
    dual_task.introspection().publishSolution(*dual_task.solutions().front());
    
    auto result = dual_task.execute(*dual_task.solutions().front());
    if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_INFO(LOGGER, "=== CẢ 2 TAY ĐÃ HOÀN THÀNH QUY TRÌNH GẮP THẢ ===");
    } else {
      RCLCPP_ERROR(LOGGER, "Thực thi quỹ đạo thất bại!");
    }
  } else {
    RCLCPP_ERROR(LOGGER, "Lập kế hoạch (Plan) cho cả 2 tay thất bại!");
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto dual_mtc_node = std::make_shared<DualArmMTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &dual_mtc_node]() {
    executor.add_node(dual_mtc_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(dual_mtc_node->getNodeBaseInterface());
  });

  std::this_thread::sleep_for(std::chrono::seconds(2));
  dual_mtc_node->doTask();

  rclcpp::shutdown();
  spin_thread->join();
  return 0;
}