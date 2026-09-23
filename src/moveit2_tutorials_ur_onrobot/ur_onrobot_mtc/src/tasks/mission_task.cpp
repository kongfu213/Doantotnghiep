#include "ur_onrobot_mtc/tasks/mission_task.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <utility>

#include <rclcpp/rclcpp.hpp>

namespace ur_onrobot_mtc
{
namespace
{

std::string trim(const std::string& input)
{
  auto first = std::find_if_not(
      input.begin(), input.end(),
      [](unsigned char c) { return std::isspace(c); });

  if (first == input.end())
    return {};

  auto last = std::find_if_not(
      input.rbegin(), input.rend(),
      [](unsigned char c) { return std::isspace(c); }).base();

  return std::string(first, last);
}

std::string firstToken(const std::string& command)
{
  std::istringstream ss(command);
  std::string token;
  ss >> token;
  return token;
}

}  // namespace

MissionTask::MissionTask(std::shared_ptr<DualArmInterface> robot)
  : TaskBase(std::move(robot))
{
}

std::vector<std::string> MissionTask::availableNames() const
{
  const auto node = robot_->node();

  if (!node->has_parameter("mission_names"))
    return {};

  const auto parameter = node->get_parameter("mission_names");
  if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    return {};

  return parameter.as_string_array();
}

void MissionTask::printAvailable() const
{
  const auto names = availableNames();
  const auto logger = robot_->node()->get_logger();

  if (names.empty()) {
    RCLCPP_WARN(
        logger,
        "No missions configured. Check config/missions.yaml and launch parameters.");
    return;
  }

  RCLCPP_INFO(logger, "Available missions:");
  for (const auto& name : names)
    RCLCPP_INFO(logger, "  mission %s", name.c_str());
}

bool MissionTask::loadSteps(const std::string& mission_name,
                            std::vector<std::string>& steps) const
{
  steps.clear();

  const auto node = robot_->node();
  const auto logger = node->get_logger();
  const std::string parameter_name = "missions." + mission_name;

  if (!node->has_parameter(parameter_name)) {
    RCLCPP_ERROR(
        logger,
        "Mission '%s' not found. Parameter '%s' is missing.",
        mission_name.c_str(), parameter_name.c_str());
    printAvailable();
    return false;
  }

  const auto parameter = node->get_parameter(parameter_name);
  if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    RCLCPP_ERROR(
        logger,
        "Mission '%s' must be a string array in missions.yaml.",
        mission_name.c_str());
    return false;
  }

  for (const auto& raw_step : parameter.as_string_array()) {
    const std::string step = trim(raw_step);
    if (step.empty())
      continue;

    // Keep mission execution finite and easy to debug.
    if (firstToken(step) == "mission") {
      RCLCPP_ERROR(
          logger,
          "Mission '%s' contains nested mission command '%s'. Nested missions are disabled.",
          mission_name.c_str(), step.c_str());
      return false;
    }

    steps.push_back(step);
  }

  if (steps.empty()) {
    RCLCPP_ERROR(
        logger,
        "Mission '%s' has no executable steps.",
        mission_name.c_str());
    return false;
  }

  return true;
}

bool MissionTask::execute(const std::string& mission_name,
                          const StepExecutor& executor)
{
  const auto logger = robot_->node()->get_logger();

  if (!executor) {
    RCLCPP_ERROR(logger, "MISSION: no step executor provided");
    return false;
  }

  std::vector<std::string> steps;
  if (!loadSteps(mission_name, steps))
    return false;

  RCLCPP_INFO(
      logger,
      "==========================================");
  RCLCPP_INFO(
      logger,
      "MISSION START: %s (%zu steps)",
      mission_name.c_str(), steps.size());
  RCLCPP_INFO(
      logger,
      "==========================================");

  for (std::size_t i = 0; i < steps.size(); ++i) {
    RCLCPP_INFO(
        logger,
        "MISSION %s STEP %zu/%zu: %s",
        mission_name.c_str(), i + 1, steps.size(), steps[i].c_str());

    if (!executor(steps[i])) {
      RCLCPP_ERROR(
          logger,
          "MISSION FAILED: %s at step %zu/%zu: %s",
          mission_name.c_str(), i + 1, steps.size(), steps[i].c_str());
      return false;
    }

    RCLCPP_INFO(
        logger,
        "MISSION STEP SUCCESS %zu/%zu",
        i + 1, steps.size());
  }

  RCLCPP_INFO(
      logger,
      "MISSION SUCCESS: %s",
      mission_name.c_str());
  return true;
}

}  // namespace ur_onrobot_mtc
