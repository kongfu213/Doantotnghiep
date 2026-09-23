#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <iostream>
#include <string>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("robot_cli");
  auto pub = node->create_publisher<std_msgs::msg::String>("/robot_command", 10);

  std::cout << "==========================================\n";
  std::cout << " Dual Arm Robot CLI\n";
  std::cout << "==========================================\n";
  std::cout << "Commands:\n";
  std::cout << "  pick A                    # auto-select best arm\n";
  std::cout << "  pick A left               # force LEFT\n";
  std::cout << "  pick A right              # force RIGHT\n";
  std::cout << "  place A x y z [yaw_deg]\n";
  std::cout << "  move A x y z [yaw_deg]\n";
  std::cout << "  stack A B C D\n";
  std::cout << "  stack_at x y A B C D\n";
  std::cout << "  swap A B\n";
  std::cout << "  mission <name>\n";
  std::cout << "  missions                    # list available missions\n";
  std::cout << "  status\n";
  std::cout << "  reset_scene\n";
  std::cout << "  help\n";
  std::cout << "  quit\n";
  std::cout << "> " << std::flush;

  std::string line;
  while (rclcpp::ok() && std::getline(std::cin, line)) {
    if (line == "quit" || line == "exit")
      break;

    if (line.empty()) {
      std::cout << "> " << std::flush;
      continue;
    }

    std_msgs::msg::String msg;
    msg.data = line;
    pub->publish(msg);

    // Flush one ROS cycle so the command is sent immediately.
    rclcpp::spin_some(node);
    std::cout << "> " << std::flush;
  }

  rclcpp::shutdown();
  return 0;
}
