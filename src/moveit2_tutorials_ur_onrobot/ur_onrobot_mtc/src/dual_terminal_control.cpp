#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <thread> // Bổ sung thư viện đa luồng để chạy 2 tay cùng lúc

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dual_terminal_control", 
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  using moveit::planning_interface::MoveGroupInterface;
  using moveit::planning_interface::PlanningSceneInterface;
  
  // Khai báo 4 nhóm điều khiển
  MoveGroupInterface left_arm(node, "left_ur_onrobot_manipulator");
  MoveGroupInterface right_arm(node, "right_ur_onrobot_manipulator");
  MoveGroupInterface left_gripper(node, "left_ur_onrobot_gripper"); 
  MoveGroupInterface right_gripper(node, "right_ur_onrobot_gripper");
  
  PlanningSceneInterface planning_scene_interface;

  std::cout << "\n======================================================\n";
  std::cout << "  HE THONG DIEU KHIEN 2 TAY MAY (DUAL ARM CLI)\n";
  std::cout << "======================================================\n";
  std::cout << "[Lenh he thong]: spawn (tao vat), q (thoat)\n";
  std::cout << "[Cu phap tay]  : <left/right/both> <lenh>\n";
  std::cout << " - Di chuyen   : left home, right up, both home...\n";
  std::cout << " - Dong/mo kep : left open, right closed, both open...\n";
  std::cout << " - Tuong tac   : left attach, right detach, both detach...\n";
  std::cout << "======================================================\n";

  // Hàm tiện ích hỗ trợ nhả vật và rút tay lên 10cm
  auto detach_and_lift = [](MoveGroupInterface* arm, const std::string& arm_name) {
    arm->detachObject("object");
    std::cout << "==> " << arm_name << " arm: Da nha vat.\n";

    geometry_msgs::msg::Pose current_pose = arm->getCurrentPose().pose;
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.z += 0.10; // Lùi tay lên cao 10cm theo trục Z

    std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = arm->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction > 0.8) {
      arm->execute(trajectory);
      std::cout << "==> " << arm_name << " arm: Da tu dong rut tay len cao 10cm!\n";
    } else {
      std::cout << "==> " << arm_name << " arm: Khong the rut tay len 10cm (Out of reach)!\n";
    }
  };

  while (rclcpp::ok()) {
    std::cout << "\n[LENH] -> ";
    std::string input;
    std::getline(std::cin, input);
    if (input.empty()) continue;

    std::istringstream iss(input);
    std::string side, action;
    iss >> side >> action;

    if (side == "q") break;

    // --- LỆNH SPAWN ---
    if (side == "spawn") {
      std::vector<moveit_msgs::msg::CollisionObject> objects;

      moveit_msgs::msg::CollisionObject table;
      table.header.frame_id = "world";
      table.id = "table";
      shape_msgs::msg::SolidPrimitive table_primitive;
      table_primitive.type = table_primitive.BOX;
      table_primitive.dimensions = {3.0, 3.0, 0.1};

      geometry_msgs::msg::Pose table_pose;
      table_pose.position.z = -0.06;
      table_pose.orientation.w = 1.0;
      table.primitives.push_back(table_primitive);
      table.primitive_poses.push_back(table_pose);
      table.operation = table.ADD;
      objects.push_back(table);

      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = "world";
      object.id = "object";
      shape_msgs::msg::SolidPrimitive obj_primitive;
      obj_primitive.type = obj_primitive.CYLINDER;
      obj_primitive.dimensions = {0.1, 0.02};

      geometry_msgs::msg::Pose obj_pose;
      obj_pose.position.x = 0.0;
      obj_pose.position.y = 1.0;
      obj_pose.position.z = 0.02;
      obj_pose.orientation.x = 0.707;
      obj_pose.orientation.w = 0.707;
      object.primitives.push_back(obj_primitive);
      object.primitive_poses.push_back(obj_pose);
      object.operation = object.ADD;
      objects.push_back(object);

      planning_scene_interface.applyCollisionObjects(objects);
      std::cout << "==> Da tao table va object vao RViz!\n";
      continue;
    }

    if (action.empty()) {
      std::cout << "==> Loi: Thieu hanh dong (VD: left home, both home)\n";
      continue;
    }

    // =========================================================
    // --- XỬ LÝ ĐIỀU KHIỂN CẢ 2 TAY CÙNG LÚC (BOTH) ---
    // =========================================================
    if (side == "both") {
      if (action == "open" || action == "closed") {
        left_gripper.setNamedTarget(action);
        right_gripper.setNamedTarget(action);
        
        std::thread t1([&]() { left_gripper.move(); });
        std::thread t2([&]() { right_gripper.move(); });
        t1.join();
        t2.join();
        std::cout << "==> Both grippers: Da thuc hien " << action << " xong!\n";
      } 
      else if (action == "attach") {
        auto touch_left = left_gripper.getLinkNames();
        auto touch_right = right_gripper.getLinkNames();
        left_arm.attachObject("object", "left_gripper_tcp", touch_left);
        right_arm.attachObject("object", "right_gripper_tcp", touch_right);
        std::cout << "==> Both arms: Da gan vat vao ca 2 tay kep!\n";
      } 
      else if (action == "detach") {
        std::thread t1(detach_and_lift, &left_arm, "left");
        std::thread t2(detach_and_lift, &right_arm, "right");
        t1.join();
        t2.join();
      } 
      else { // Chạy các pose di chuyển: home, up,...
        left_arm.setNamedTarget(action);
        right_arm.setNamedTarget(action);

        std::thread t1([&]() { left_arm.move(); });
        std::thread t2([&]() { right_arm.move(); });
        t1.join();
        t2.join();
        std::cout << "==> Both arms: Da thuc hien pose '" << action << "' xong song song!\n";
      }
      continue;
    }

    // =========================================================
    // --- XỬ LÝ ĐIỀU KHIỂN ĐƠN LẺ (LEFT / RIGHT) ---
    // =========================================================
    MoveGroupInterface* target_arm = nullptr;
    MoveGroupInterface* target_gripper = nullptr;
    std::string tcp_link = "";

    if (side == "left") {
      target_arm = &left_arm;
      target_gripper = &left_gripper;
      tcp_link = "left_gripper_tcp";
    } else if (side == "right") {
      target_arm = &right_arm;
      target_gripper = &right_gripper;
      tcp_link = "right_gripper_tcp";
    } else {
      std::cout << "==> Loi: Chi dung 'left', 'right' hoac 'both'!\n";
      continue;
    }

    // Lệnh Attach đơn lẻ
    if (action == "attach") {
      std::vector<std::string> touch_links = target_gripper->getLinkNames();
      target_arm->attachObject("object", tcp_link, touch_links);
      std::cout << "==> " << side << " arm: Da gan vat vao tay kep\n";
      continue;
    } 
    // Lệnh Detach đơn lẻ + rút tay 10cm
    else if (action == "detach") {
      detach_and_lift(target_arm, side);
      continue;
    }

    // Lệnh Pose đơn lẻ (home, up, open, closed...)
    moveit::core::MoveItErrorCode success;
    if (action == "open" || action == "closed") {
      target_gripper->setNamedTarget(action);
      success = target_gripper->move();
    } else {
      target_arm->setNamedTarget(action);
      success = target_arm->move();
    }

    if (success == moveit::core::MoveItErrorCode::SUCCESS) {
      std::cout << "==> " << side << " arm thuc hien thanh cong: " << action << "\n";
    } else {
      std::cout << "==> Loi: " << side << " arm khong the thuc hien lenh '" << action << "'\n";
    }
  }

  rclcpp::shutdown();
  return 0;
}