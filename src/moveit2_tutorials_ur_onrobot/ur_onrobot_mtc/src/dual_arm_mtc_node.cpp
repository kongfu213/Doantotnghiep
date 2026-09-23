#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <cmath>

#include "ur_onrobot_mtc/dual_arm_interface.hpp"
#include "ur_onrobot_mtc/tasks/pick_task.hpp"
#include "ur_onrobot_mtc/tasks/place_task.hpp"
#include "ur_onrobot_mtc/tasks/move_task.hpp"
#include "ur_onrobot_mtc/tasks/mission_task.hpp"
#include "ur_onrobot_mtc/tasks/robot_selector.hpp"
#include "ur_onrobot_mtc/tasks/stack_task.hpp"
#include "ur_onrobot_mtc/tasks/swap_task.hpp"

namespace ur_onrobot_mtc
{

class CommandDispatcher
{
public:
  explicit CommandDispatcher(std::shared_ptr<DualArmInterface> robot)
    : robot_(std::move(robot))
  {
    selector_ = std::make_shared<RobotSelector>(robot_);
    pick_task_ = std::make_shared<PickTask>(robot_, selector_);
    place_task_ = std::make_shared<PlaceTask>(robot_);
    move_task_ = std::make_shared<MoveTask>(robot_);
    stack_task_ = std::make_shared<StackTask>(robot_);
    swap_task_ = std::make_shared<SwapTask>(robot_);
    mission_task_ = std::make_shared<MissionTask>(robot_);

    command_sub_ = robot_->node()->create_subscription<std_msgs::msg::String>(
        "/robot_command",
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          if (!msg || msg->data.empty())
            return;

          {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push_back(msg->data);
          }
          cv_.notify_one();
        });

    worker_ = std::thread([this]() { workerLoop(); });
  }

  ~CommandDispatcher()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    cv_.notify_all();

    if (worker_.joinable())
      worker_.join();
  }

  void printHelp() const
  {
    auto logger = robot_->node()->get_logger();

    RCLCPP_INFO(logger, "==========================================");
    RCLCPP_INFO(logger, " DUAL ARM COMMAND SERVER");
    RCLCPP_INFO(logger, "==========================================");
    RCLCPP_INFO(logger, " pick A [auto|left|right]");
    RCLCPP_INFO(logger, " place A x y z [yaw_deg]");
    RCLCPP_INFO(logger, " move A x y z [yaw_deg]");
    RCLCPP_INFO(logger, " stack A B C D        # dual-pick in pairs, stack at first object x/y");
    RCLCPP_INFO(logger, " stack_at x y A B C D # dual-pick in pairs, explicit stack target");
    RCLCPP_INFO(logger, " swap A B");
    RCLCPP_INFO(logger, " mission <name>           # run a predefined mission");
    RCLCPP_INFO(logger, " missions                 # list predefined missions");
    RCLCPP_INFO(logger, " status");
    RCLCPP_INFO(logger, " reset_scene");
    RCLCPP_INFO(logger, " help");
    RCLCPP_INFO(logger, "==========================================");
  }

private:
  void workerLoop()
  {
    while (rclcpp::ok()) {
      std::string command;

      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() {
          return stop_ || !queue_.empty() || !rclcpp::ok();
        });

        if (stop_ || !rclcpp::ok())
          return;

        command = queue_.front();
        queue_.pop_front();
      }

      RCLCPP_INFO(
          robot_->node()->get_logger(),
          "COMMAND START: %s",
          command.c_str());

      const bool ok = executeCommand(command);

      if (ok) {
        RCLCPP_INFO(
            robot_->node()->get_logger(),
            "COMMAND SUCCESS: %s",
            command.c_str());
      }
      else {
        RCLCPP_ERROR(
            robot_->node()->get_logger(),
            "COMMAND FAILED: %s",
            command.c_str());
      }
    }
  }

  bool executeCommand(const std::string& command)
  {
    std::istringstream ss(command);
    std::string op;
    ss >> op;

    if (op.empty())
      return false;

    if (op == "help") {
      printHelp();
      return true;
    }

    if (op == "status") {
      robot_->printStatus();
      return true;
    }

    if (op == "reset_scene")
      return robot_->resetScene();

    if (op == "missions") {
      mission_task_->printAvailable();
      return true;
    }

    if (op == "mission") {
      std::string mission_name;
      if (!(ss >> mission_name)) {
        RCLCPP_ERROR(robot_->node()->get_logger(),
                     "Usage: mission <name>");
        mission_task_->printAvailable();
        return false;
      }

      // A mission runs synchronously inside the same command worker.
      // Incoming terminal commands remain queued until the mission finishes.
      return mission_task_->execute(
          mission_name,
          [this](const std::string& step_command) {
            return executeCommand(step_command);
          });
    }

    if (op == "pick") {
      std::string object;
      std::string arm_text = "auto";
      ss >> object;

      if (object.empty()) {
        RCLCPP_ERROR(robot_->node()->get_logger(),
                     "Usage: pick <object> [auto|left|right]");
        return false;
      }

      if (ss >> arm_text) {
        // Optional arm was supplied.
      }

      const ArmSide arm = DualArmInterface::parseArmSide(arm_text);
      if (arm == ArmSide::NONE) {
        RCLCPP_ERROR(robot_->node()->get_logger(),
                     "Invalid arm: %s", arm_text.c_str());
        return false;
      }

      return pick_task_->execute(object, arm);
    }

    if (op == "place") {
      std::string object;
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;

      if (!(ss >> object >> x >> y >> z)) {
        RCLCPP_ERROR(robot_->node()->get_logger(),
                     "Usage: place <object> <x> <y> <z> [yaw_deg]");
        return false;
      }

      // No yaw in the command means AUTO orientation.
      // A finite yaw is used only when the user explicitly supplies it.
      double yaw_rad = std::numeric_limits<double>::quiet_NaN();
      double yaw_deg = 0.0;
      if (ss >> yaw_deg)
        yaw_rad = yaw_deg * M_PI / 180.0;

      return place_task_->execute(object, x, y, z, yaw_rad);
    }

    if (op == "move") {
      std::string object;
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;

      if (!(ss >> object >> x >> y >> z)) {
        RCLCPP_ERROR(robot_->node()->get_logger(),
                     "Usage: move <object> <x> <y> <z> [yaw_deg]");
        return false;
      }

      // NaN means: the user did not request a fixed yaw.
      // DualArmInterface::moveObject() will choose a feasible orientation.
      double yaw_rad = std::numeric_limits<double>::quiet_NaN();
      double yaw_deg = 0.0;
      if (ss >> yaw_deg)
        yaw_rad = yaw_deg * M_PI / 180.0;

      return move_task_->execute(object, x, y, z, yaw_rad);
    }

    if (op == "stack" || op == "stack_at") {
      double center_x = 0.0;
      double center_y = 0.0;

      if (op == "stack_at") {
        if (!(ss >> center_x >> center_y)) {
          RCLCPP_ERROR(robot_->node()->get_logger(),
                       "Usage: stack_at <x> <y> <object1> <object2> ...");
          return false;
        }
      }

      std::vector<std::string> objects;
      std::string object;
      while (ss >> object)
        objects.push_back(object);

      if (objects.empty()) {
        RCLCPP_ERROR(robot_->node()->get_logger(),
                     "Usage: stack <object1> <object2> ...");
        return false;
      }

      // Dynamic default: plain `stack A B ...` stacks at the CURRENT x/y of
      // the first object. No stack-center pose is taken from the launch file.
      if (op == "stack") {
        double object_z = 0.0;
        if (!robot_->getObjectPose(objects.front(), center_x, center_y, object_z)) {
          RCLCPP_ERROR(robot_->node()->get_logger(),
                       "STACK: cannot read current pose of %s from PlanningScene",
                       objects.front().c_str());
          return false;
        }

        RCLCPP_INFO(robot_->node()->get_logger(),
                    "STACK dynamic target from first object %s: (%.3f, %.3f)",
                    objects.front().c_str(), center_x, center_y);
      }

      return stack_task_->execute(objects, center_x, center_y);
    }

    if (op == "swap") {
      std::string a;
      std::string b;

      if (!(ss >> a >> b)) {
        RCLCPP_ERROR(robot_->node()->get_logger(),
                     "Usage: swap <objectA> <objectB>");
        return false;
      }

      return swap_task_->execute(a, b);
    }

    RCLCPP_ERROR(robot_->node()->get_logger(),
                 "Unknown command: %s", op.c_str());
    return false;
  }

  std::shared_ptr<DualArmInterface> robot_;
  std::shared_ptr<RobotSelector> selector_;
  std::shared_ptr<PickTask> pick_task_;
  std::shared_ptr<PlaceTask> place_task_;
  std::shared_ptr<MoveTask> move_task_;
  std::shared_ptr<StackTask> stack_task_;
  std::shared_ptr<SwapTask> swap_task_;
  std::shared_ptr<MissionTask> mission_task_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

  std::deque<std::string> queue_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  bool stop_ = false;
};

}  // namespace ur_onrobot_mtc

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto robot = std::make_shared<ur_onrobot_mtc::DualArmInterface>(options);
  auto dispatcher = std::make_shared<ur_onrobot_mtc::CommandDispatcher>(robot);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot->getNodeBaseInterface());

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  // Allow joint_states / move_group / PlanningScene to become available.
  std::this_thread::sleep_for(std::chrono::seconds(1));

  if (!robot->initializeScene()) {
    RCLCPP_ERROR(robot->node()->get_logger(),
                 "Failed to initialize dual-arm scene");
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  dispatcher->printHelp();
  RCLCPP_INFO(robot->node()->get_logger(),
              "READY: run 'ros2 run ur_onrobot_mtc robot_cli' in another terminal");

  spin_thread.join();
  return 0;
}
