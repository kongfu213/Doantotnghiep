#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("terminal_control", 
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  using moveit::planning_interface::MoveGroupInterface;
  using moveit::planning_interface::PlanningSceneInterface;
  
  // Khai bao 2 nhom dieu khien
  MoveGroupInterface arm(node, "ur_onrobot_manipulator");
  MoveGroupInterface gripper(node, "ur_onrobot_gripper");
  PlanningSceneInterface planning_scene_interface;

  std::cout << "\n-----------------------------------------\n";
  std::cout << "HE THONG GAP DAT THONG MINH (PHIEN BAN MTC)\n";
  std::cout << "-----------------------------------------\n";
  std::cout << "Lenh di chuyen: home, up, gap_vat\n";
  std::cout << "Lenh kep nha : open, closed\n";
  std::cout << "Lenh tuong tac: spawn, attach, detach\n";
  std::cout << "Go 'q' de thoat chuong trinh\n";
  std::cout << "-----------------------------------------\n";

  while (rclcpp::ok()) {
    std::string target;
    std::cout << "\n[LENH] -> ";
    std::cin >> target;

    if (target == "q") break;

    // --- LENH SPAWN: TAO BAN VA VAT (Lay tu code MTC cua ong) ---
    if (target == "spawn") {
      std::vector<moveit_msgs::msg::CollisionObject> objects;

      // 1. Tao cai ban (table)
      moveit_msgs::msg::CollisionObject table;
      table.header.frame_id = "world";
      table.id = "table";
      shape_msgs::msg::SolidPrimitive table_primitive;
      table_primitive.type = table_primitive.BOX;
      table_primitive.dimensions = {1.0, 1.0, 0.1}; // X, Y, Z

      geometry_msgs::msg::Pose table_pose;
      table_pose.position.x = 0.0;
      table_pose.position.y = 0.0;
      table_pose.position.z = -0.06;
      table_pose.orientation.w = 1.0;
      table.primitives.push_back(table_primitive);
      table.primitive_poses.push_back(table_pose);
      table.operation = table.ADD;
      objects.push_back(table);

      // 2. Tao vat the (object) hinh tru nam ngang
      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = "world";
      object.id = "object";
      shape_msgs::msg::SolidPrimitive obj_primitive;
      obj_primitive.type = obj_primitive.CYLINDER;
      obj_primitive.dimensions = {0.1, 0.02}; // Height, Radius

      geometry_msgs::msg::Pose obj_pose;
      obj_pose.position.x = 0.3;
      obj_pose.position.y = -0.2;
      obj_pose.position.z = 0.02;
      obj_pose.orientation.x = 0.707; // Nam ngang theo code MTC
      obj_pose.orientation.w = 0.707;
      object.primitives.push_back(obj_primitive);
      object.primitive_poses.push_back(obj_pose);
      object.operation = object.ADD;
      objects.push_back(object);

      planning_scene_interface.applyCollisionObjects(objects);
      std::cout << "==> Da tao table va object vao RViz!\n";
      continue;
    }

    // --- LENH ATTACH: KET NOI VAT VAO TAY KEP ---
    else if (target == "attach") {
      std::vector<std::string> touch_links = gripper.getLinkNames();
      // "gripper_tcp" la diem cuoi cua tay kep OnRobot
      arm.attachObject("object", "gripper_tcp", touch_links);
      std::cout << "==> Da gan vat vao tay kep (Attach success)\n";
      continue;
    }

    // --- LENH DETACH: THA VAT RA ---
    else if (target == "detach") {
      arm.detachObject("object");
      std::cout << "==> Da nha vat (Detach success)\n";
      continue;
    }

    // --- DIEU KHIEN POSE ---
    moveit::core::MoveItErrorCode success;
    if (target == "open" || target == "closed") {
      gripper.setNamedTarget(target);
      success = gripper.move();
    } else {
      arm.setNamedTarget(target);
      success = arm.move();
    }

    if (success == moveit::core::MoveItErrorCode::SUCCESS) {
      std::cout << "==> Thuc hien thanh cong: " << target << "\n";
    } else {
      std::cout << "==> Loi: Khong the thuc hien lenh nay!\n";
    }
  }

  rclcpp::shutdown();
  return 0;
}