#pragma once

#include <array>
#include <cstddef>
#include <stdexcept>

namespace ur_onrobot_mtc::pcb
{
// Shared by the scene and the solder task. All lengths are metres.
inline constexpr double board_x = 0.220, board_y = 0.150, board_z = 0.005;
inline constexpr double component_x = 0.012, component_y = 0.022, component_z = 0.014;
inline constexpr double handle_length = 0.100, handle_width = 0.026, handle_height = 0.024;
inline constexpr double shaft_length = 0.055, shaft_width = 0.009;
inline constexpr double tip_length = 0.025, tip_width = 0.005;
inline constexpr double tip_from_handle = handle_length / 2.0 + shaft_length + tip_length;
inline constexpr double pad_dx = 0.011, pad_x = 0.006, pad_y = 0.005, pad_z = 0.0008;
inline constexpr double pad_center_above_board = 0.0015;
inline constexpr double placement_clearance = 0.0005;
inline constexpr std::array<std::array<double, 2>, 3> slots = {{
  {{-0.055, 0.032}}, {{0.055, 0.032}}, {{0.000, -0.035}}
}};

inline std::array<double, 3> componentOffset(std::size_t slot)
{
  const auto& xy = slots.at(slot);
  return {xy[0], xy[1], board_z / 2.0 + component_z / 2.0 + placement_clearance};
}

inline std::array<double, 3> padOffset(std::size_t slot, std::size_t pad)
{
  if (pad > 1) throw std::out_of_range("PCB pad must be 0 or 1");
  const auto& xy = slots.at(slot);
  return {xy[0] + (pad == 0 ? -pad_dx : pad_dx), xy[1],
          board_z / 2.0 + pad_center_above_board + pad_z / 2.0};
}
}  // namespace ur_onrobot_mtc::pcb
