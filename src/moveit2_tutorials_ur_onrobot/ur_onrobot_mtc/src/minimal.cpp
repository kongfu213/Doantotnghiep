#include <memory>
#include <thread>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/container.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dual_mtc_node");
namespace mtc = moveit::task_constructor;

class DualArmMTCTaskNode
{
public:
  DualArmMTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();
  void setupPlanningScene();

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr DualArmMTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

DualArmMTCTaskNode::DualArmMTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("dual_mtc_node", options) }
{
}

void DualArmMTCTaskNode::setupPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;

  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = "world";
  table.id = "table";
  shape_msgs::msg::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  table_primitive.dimensions = { 1.0, 1.0, 0.1 };

  geometry_msgs::msg::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = 0.0;
  table_pose.position.y = 0.0;
  table_pose.position.z = -0.05; 

  table.primitives.push_back(table_primitive);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;

  moveit_msgs::msg::CollisionObject left_obj;
  left_obj.id = "left_object";
  left_obj.header.frame_id = "world";
  left_obj.primitives.resize(1);
  left_obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  left_obj.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose left_pose;
  left_pose.position.x = 0.3;
  left_pose.position.y = 0.2; 
  left_pose.position.z = 0.05;
  left_pose.orientation.x = 0.707;
  left_pose.orientation.w = 0.707;
  left_obj.primitive_poses.push_back(left_pose);
  left_obj.operation = left_obj.ADD;

  moveit_msgs::msg::CollisionObject right_obj;
  right_obj.id = "right_object";
  right_obj.header.frame_id = "world";
  right_obj.primitives.resize(1);
  right_obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  right_obj.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose right_pose;
  right_pose.position.x = 0.3;
  right_pose.position.y = -0.2; 
  right_pose.position.z = 0.05;
  right_pose.orientation.x = 0.707;
  right_pose.orientation.w = 0.707;
  right_obj.primitive_poses.push_back(right_pose);
  right_obj.operation = right_obj.ADD;

  psi.applyCollisionObject(table);
  psi.applyCollisionObject(left_obj);
  psi.applyCollisionObject(right_obj);
}

mtc::Task DualArmMTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("dual_arm_pick_place_task");
  task.loadRobotModel(node_);

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  
  // [SỬA] Đặt 5mm mỗi bước. Quãng đường retreat 5cm sẽ chia ra đúng 10 steps -> Không bị dính warning rác.
  cartesian_planner->setStepSize(0.005);

  mtc::Stage* current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto merger = std::make_unique<mtc::Merger>("dual_arm_parallel_execution");

  // ==========================================
  // 1. QUY TRÌNH CHO TAY TRÁI (LEFT ARM)
  // ==========================================
  {
    auto left_pipeline = std::make_unique<mtc::SerialContainer>("left_arm_pipeline");

    const std::string arm_group = "left_ur_onrobot_manipulator";
    const std::string hand_group = "left_ur_onrobot_gripper";
    const std::string hand_frame = "left_gripper_tcp";
    const std::string object_name = "left_object";

    left_pipeline->setProperty("group", arm_group);
    left_pipeline->setProperty("eef", hand_group);
    left_pipeline->setProperty("ik_frame", hand_frame);

    auto open_hand = std::make_unique<mtc::stages::MoveTo>("left open hand", interpolation_planner);
    open_hand->setGroup(hand_group);
    open_hand->setGoal("open");
    left_pipeline->insert(std::move(open_hand));

    auto move_pick = std::make_unique<mtc::stages::Connect>(
        "left move to pick", mtc::stages::Connect::GroupPlannerVector{ { arm_group, sampling_planner } });
    move_pick->setTimeout(20.0);
    left_pipeline->insert(std::move(move_pick));

    mtc::Stage* attach_stage = nullptr;

    // Pick Container Trái
    {
      auto grasp = std::make_unique<mtc::SerialContainer>("left pick object");
      grasp->setProperty("group", arm_group);
      grasp->setProperty("eef", hand_group);
      grasp->setProperty("ik_frame", hand_frame);

      // [SỬA LỚN] ĐƯA KHỐI NÀY LÊN ĐẦU TIÊN CỦA CONTAINER.
      // Cho phép tay chạm vật và bàn TRƯỚC KHI sinh ra đường đi approach và tính toán IK.
      auto allow_coll = std::make_unique<mtc::stages::ModifyPlanningScene>("allow left coll");
      const auto* jmg = task.getRobotModel()->getJointModelGroup(hand_group);
      allow_coll->allowCollisions(object_name, *jmg, true);
      allow_coll->allowCollisions("table", *jmg, true);
      allow_coll->allowCollisions("left_onrobot_base_link", "table", true);
      grasp->insert(std::move(allow_coll));

      auto approach = std::make_unique<mtc::stages::MoveRelative>("approach left obj", cartesian_planner);
      approach->setGroup(arm_group); 
      approach->properties().set("link", hand_frame);
      approach->setMinMaxDistance(0.05, 0.15); // Hạ khoảng cách tối thiểu
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      approach->setDirection(vec);
      grasp->insert(std::move(approach));

      auto gen_pose = std::make_unique<mtc::stages::GenerateGraspPose>("generate left grasp pose");
      gen_pose->properties().set("eef", hand_group);
      gen_pose->setPreGraspPose("open");
      gen_pose->setObject(object_name);
      gen_pose->setAngleDelta(M_PI / 18); // Chia lưới góc thưa hơn (10 độ/bước) để tính IK nhanh hơn
      gen_pose->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d tf;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      tf.linear() = q.matrix();

      auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>("left grasp IK", std::move(gen_pose));
      ik_wrapper->properties().set("eef", hand_group);
      ik_wrapper->properties().set("group", arm_group);
      ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      ik_wrapper->setMaxIKSolutions(4);
      ik_wrapper->setMinSolutionDistance(1.0);
      ik_wrapper->setIKFrame(tf, hand_frame);
      grasp->insert(std::move(ik_wrapper));

      auto close_hand = std::make_unique<mtc::stages::MoveTo>("close left hand", interpolation_planner);
      close_hand->setGroup(hand_group);
      close_hand->setGoal("closed");
      grasp->insert(std::move(close_hand));

      auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach left obj");
      attach->attachObject(object_name, hand_frame);
      attach_stage = attach.get();
      grasp->insert(std::move(attach));

      auto lift = std::make_unique<mtc::stages::MoveRelative>("lift left obj", cartesian_planner);
      lift->setGroup(arm_group); 
      lift->setMinMaxDistance(0.05, 0.15);
      lift->setIKFrame(hand_frame);
      geometry_msgs::msg::Vector3Stamped lift_vec;
      lift_vec.header.frame_id = "world";
      lift_vec.vector.z = 1.0;
      lift->setDirection(lift_vec);
      grasp->insert(std::move(lift));

      left_pipeline->insert(std::move(grasp));
    }

    auto move_place = std::make_unique<mtc::stages::Connect>(
        "left move to place", mtc::stages::Connect::GroupPlannerVector{ { arm_group, sampling_planner } });
    move_place->setTimeout(20.0);
    left_pipeline->insert(std::move(move_place));

    // Place Container Trái
    {
      auto place = std::make_unique<mtc::SerialContainer>("left place object");
      place->setProperty("group", arm_group);
      place->setProperty("eef", hand_group);
      place->setProperty("ik_frame", hand_frame);

      auto allow_table = std::make_unique<mtc::stages::ModifyPlanningScene>("allow left obj-table coll");
      allow_table->allowCollisions(object_name, "table", true);
      const auto* jmg = task.getRobotModel()->getJointModelGroup(hand_group);
      allow_table->allowCollisions("table", *jmg, true); 
      place->insert(std::move(allow_table));

      auto gen_place = std::make_unique<mtc::stages::GeneratePlacePose>("gen left place pose");
      gen_place->properties().set("eef", hand_group);
      gen_place->setObject(object_name);

      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.header.frame_id = "world";
      target_pose.pose.position.x = 0.1;
      target_pose.pose.position.y = 0.4;
      target_pose.pose.position.z = 0.05; 
      target_pose.pose.orientation.w = 1.0;
      gen_place->setPose(target_pose);
      gen_place->setMonitoredStage(attach_stage);

      auto ik_place = std::make_unique<mtc::stages::ComputeIK>("left place IK", std::move(gen_place));
      ik_place->properties().set("eef", hand_group);
      ik_place->properties().set("group", arm_group);
      ik_place->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      ik_place->setMaxIKSolutions(4);
      ik_place->setMinSolutionDistance(1.0);
      ik_place->setIKFrame(object_name);
      place->insert(std::move(ik_place));

      auto open_hand = std::make_unique<mtc::stages::MoveTo>("open left hand post", interpolation_planner);
      open_hand->setGroup(hand_group);
      open_hand->setGoal("open");
      place->insert(std::move(open_hand));

      auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach left obj");
      detach->detachObject(object_name, hand_frame);
      place->insert(std::move(detach));

      auto retreat = std::make_unique<mtc::stages::MoveRelative>("retreat left", cartesian_planner);
      retreat->setGroup(arm_group);
      retreat->setMinMaxDistance(0.05, 0.15);
      retreat->setIKFrame(hand_frame);
      geometry_msgs::msg::Vector3Stamped ret_vec;
      ret_vec.header.frame_id = "world";
      ret_vec.vector.z = 1.0;
      retreat->setDirection(ret_vec);
      place->insert(std::move(retreat));

      left_pipeline->insert(std::move(place));
    }

    auto home = std::make_unique<mtc::stages::MoveTo>("left return home", interpolation_planner);
    home->setGroup(arm_group);
    home->setGoal("left_test_configuration"); // Hãy đảm bảo preset này có thật trong file SRDF
    left_pipeline->insert(std::move(home));

    merger->insert(std::move(left_pipeline));
  }

  // ==========================================
  // 2. QUY TRÌNH CHO TAY PHẢI (RIGHT ARM)
  // ==========================================
  {
    auto right_pipeline = std::make_unique<mtc::SerialContainer>("right_arm_pipeline");

    const std::string arm_group = "right_ur_onrobot_manipulator";
    const std::string hand_group = "right_ur_onrobot_gripper";
    const std::string hand_frame = "right_gripper_tcp";
    const std::string object_name = "right_object";

    right_pipeline->setProperty("group", arm_group);
    right_pipeline->setProperty("eef", hand_group);
    right_pipeline->setProperty("ik_frame", hand_frame);

    auto open_hand = std::make_unique<mtc::stages::MoveTo>("right open hand", interpolation_planner);
    open_hand->setGroup(hand_group);
    open_hand->setGoal("open");
    right_pipeline->insert(std::move(open_hand));

    auto move_pick = std::make_unique<mtc::stages::Connect>(
        "right move to pick", mtc::stages::Connect::GroupPlannerVector{ { arm_group, sampling_planner } });
    move_pick->setTimeout(20.0);
    right_pipeline->insert(std::move(move_pick));

    mtc::Stage* attach_stage = nullptr;

    // Pick Container Phải
    {
      auto grasp = std::make_unique<mtc::SerialContainer>("right pick object");
      grasp->setProperty("group", arm_group);
      grasp->setProperty("eef", hand_group);
      grasp->setProperty("ik_frame", hand_frame);

      // [SỬA LỚN] ĐƯA KHỐI NÀY LÊN ĐẦU
      auto allow_coll = std::make_unique<mtc::stages::ModifyPlanningScene>("allow right coll");
      const auto* jmg = task.getRobotModel()->getJointModelGroup(hand_group);
      allow_coll->allowCollisions(object_name, *jmg, true);
      allow_coll->allowCollisions("table", *jmg, true);
      allow_coll->allowCollisions("right_onrobot_base_link", "table", true);
      grasp->insert(std::move(allow_coll));

      auto approach = std::make_unique<mtc::stages::MoveRelative>("approach right obj", cartesian_planner);
      approach->setGroup(arm_group); 
      approach->properties().set("link", hand_frame);
      approach->setMinMaxDistance(0.05, 0.15);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      approach->setDirection(vec);
      grasp->insert(std::move(approach));

      auto gen_pose = std::make_unique<mtc::stages::GenerateGraspPose>("generate right grasp pose");
      gen_pose->properties().set("eef", hand_group);
      gen_pose->setPreGraspPose("open");
      gen_pose->setObject(object_name);
      gen_pose->setAngleDelta(M_PI / 18);
      gen_pose->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d tf;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      tf.linear() = q.matrix();

      auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>("right grasp IK", std::move(gen_pose));
      ik_wrapper->properties().set("eef", hand_group);
      ik_wrapper->properties().set("group", arm_group);
      ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      ik_wrapper->setMaxIKSolutions(4);
      ik_wrapper->setMinSolutionDistance(1.0);
      ik_wrapper->setIKFrame(tf, hand_frame);
      grasp->insert(std::move(ik_wrapper));

      auto close_hand = std::make_unique<mtc::stages::MoveTo>("close right hand", interpolation_planner);
      close_hand->setGroup(hand_group);
      close_hand->setGoal("closed");
      grasp->insert(std::move(close_hand));

      auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach right obj");
      attach->attachObject(object_name, hand_frame);
      attach_stage = attach.get();
      grasp->insert(std::move(attach));

      auto lift = std::make_unique<mtc::stages::MoveRelative>("lift right obj", cartesian_planner);
      lift->setGroup(arm_group); 
      lift->setMinMaxDistance(0.05, 0.15);
      lift->setIKFrame(hand_frame);
      geometry_msgs::msg::Vector3Stamped lift_vec;
      lift_vec.header.frame_id = "world";
      lift_vec.vector.z = 1.0;
      lift->setDirection(lift_vec);
      grasp->insert(std::move(lift));

      right_pipeline->insert(std::move(grasp));
    }

    auto move_place = std::make_unique<mtc::stages::Connect>(
        "right move to place", mtc::stages::Connect::GroupPlannerVector{ { arm_group, sampling_planner } });
    move_place->setTimeout(20.0);
    right_pipeline->insert(std::move(move_place));

    // Place Container Phải
    {
      auto place = std::make_unique<mtc::SerialContainer>("right place object");
      place->setProperty("group", arm_group);
      place->setProperty("eef", hand_group);
      place->setProperty("ik_frame", hand_frame);

      auto allow_table = std::make_unique<mtc::stages::ModifyPlanningScene>("allow right obj-table coll");
      allow_table->allowCollisions(object_name, "table", true);
      const auto* jmg = task.getRobotModel()->getJointModelGroup(hand_group);
      allow_table->allowCollisions("table", *jmg, true); 
      place->insert(std::move(allow_table));

      auto gen_place = std::make_unique<mtc::stages::GeneratePlacePose>("gen right place pose");
      gen_place->properties().set("eef", hand_group);
      gen_place->setObject(object_name);

      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.header.frame_id = "world";
      target_pose.pose.position.x = 0.1;
      target_pose.pose.position.y = -0.4;
      target_pose.pose.position.z = 0.05;
      target_pose.pose.orientation.w = 1.0;
      gen_place->setPose(target_pose);
      gen_place->setMonitoredStage(attach_stage);

      auto ik_place = std::make_unique<mtc::stages::ComputeIK>("right place IK", std::move(gen_place));
      ik_place->properties().set("eef", hand_group);
      ik_place->properties().set("group", arm_group);
      ik_place->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      ik_place->setMaxIKSolutions(4);
      ik_place->setMinSolutionDistance(1.0);
      ik_place->setIKFrame(object_name);
      place->insert(std::move(ik_place));

      auto open_hand = std::make_unique<mtc::stages::MoveTo>("open right hand post", interpolation_planner);
      open_hand->setGroup(hand_group);
      open_hand->setGoal("open");
      place->insert(std::move(open_hand));

      auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach right obj");
      detach->detachObject(object_name, hand_frame);
      place->insert(std::move(detach));

      auto retreat = std::make_unique<mtc::stages::MoveRelative>("retreat right", cartesian_planner);
      retreat->setGroup(arm_group); 
      retreat->setMinMaxDistance(0.05, 0.15);
      retreat->setIKFrame(hand_frame);
      geometry_msgs::msg::Vector3Stamped ret_vec;
      ret_vec.header.frame_id = "world";
      ret_vec.vector.z = 1.0;
      retreat->setDirection(ret_vec);
      place->insert(std::move(retreat));

      right_pipeline->insert(std::move(place));
    }

    auto home = std::make_unique<mtc::stages::MoveTo>("right return home", interpolation_planner);
    home->setGroup(arm_group);
    home->setGoal("right_test_configuration"); // Hãy đảm bảo preset này có thật trong file SRDF
    right_pipeline->insert(std::move(home));

    merger->insert(std::move(right_pipeline));
  }

  task.add(std::move(merger));
  return task;
}

void DualArmMTCTaskNode::doTask()
{
  task_ = createTask();
  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Dual Arm Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(LOGGER, "Dual Arm Task execution failed");
    return;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_node = std::make_shared<DualArmMTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_node]() {
    executor.add_node(mtc_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_node->getNodeBaseInterface());
  });

  std::this_thread::sleep_for(std::chrono::seconds(1));

  mtc_node->setupPlanningScene();
  mtc_node->doTask();

  rclcpp::shutdown();
  spin_thread->join();
  return 0;
}