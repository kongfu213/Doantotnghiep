#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace ur_onrobot_mtc
{

class PcbSceneNode : public rclcpp::Node
{
public:
  PcbSceneNode()
  : Node("setup_pcb_scene")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    table_top_ = declare_parameter<double>("table_top", -0.003);
    pcb_x_ = declare_parameter<double>("pcb_x", 0.0);
    pcb_y_ = declare_parameter<double>("pcb_y", 0.0);

    rclcpp::QoS marker_qos(1);
    marker_qos.reliable().transient_local();
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/pcb_scene_markers", marker_qos);

    spawn_timer_ = create_wall_timer(750ms, [this]() {
      spawn_timer_->cancel();
      spawnScene();
    });
  }

private:
  static geometry_msgs::msg::Pose makePose(double x, double y, double z)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = 1.0;
    return pose;
  }

  static geometry_msgs::msg::Point makePoint(double x, double y, double z)
  {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  moveit_msgs::msg::CollisionObject makeBox(
      const std::string& id,
      double sx, double sy, double sz,
      double x, double y, double z) const
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame_id_;
    object.id = id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {sx, sy, sz};

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(makePose(x, y, z));
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    return object;
  }

  visualization_msgs::msg::Marker makeCubeMarker(
      int id,
      const std::string& ns,
      double x, double y, double z,
      double sx, double sy, double sz,
      float r, float g, float b, float a = 1.0f) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = makePose(x, y, z);
    marker.scale.x = sx;
    marker.scale.y = sy;
    marker.scale.z = sz;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    return marker;
  }

  visualization_msgs::msg::Marker makeCylinderMarker(
      int id,
      const std::string& ns,
      double x, double y, double z,
      double diameter, double height,
      float r, float g, float b, float a = 1.0f) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = makePose(x, y, z);
    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = height;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    return marker;
  }

  visualization_msgs::msg::Marker makeLineStripMarker(
      int id,
      const std::string& ns,
      const std::vector<geometry_msgs::msg::Point>& points,
      double width,
      float r, float g, float b, float a = 1.0f) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = width;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.points = points;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    return marker;
  }

  visualization_msgs::msg::Marker makeTextMarker(
      int id,
      const std::string& ns,
      const std::string& text,
      double x, double y, double z,
      double text_height,
      float r, float g, float b, float a = 1.0f) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = makePose(x, y, z);
    marker.scale.z = text_height;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.text = text;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    return marker;
  }

  void spawnScene()
  {
    // ---------------- Physical collision geometry ----------------
    constexpr double pcb_sx = 0.180;   // 180 mm
    constexpr double pcb_sy = 0.120;   // 120 mm
    constexpr double pcb_sz = 0.005;   // 5 mm

    constexpr double component_sx = 0.016;
    constexpr double component_sy = 0.030;
    constexpr double component_sz = 0.020;

    const double pcb_center_z = table_top_ + 0.5 * pcb_sz;
    const double pcb_top_z = table_top_ + pcb_sz;
    const double component_source_z = table_top_ + 0.5 * component_sz;
    const double component_target_z = pcb_top_z + 0.5 * component_sz;

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.reserve(5);

    // Keep the collision model simple and fast: PCB is still one flat box.
    objects.push_back(makeBox(
        "pcb",
        pcb_sx, pcb_sy, pcb_sz,
        pcb_x_, pcb_y_, pcb_center_z));

    // Four components wait near the robot on the +X side.
    constexpr double source_x = 0.220;
    const std::array<double, 4> source_y = {-0.075, -0.025, 0.025, 0.075};

    for (std::size_t i = 0; i < source_y.size(); ++i) {
      objects.push_back(makeBox(
          "component_" + std::to_string(i + 1),
          component_sx, component_sy, component_sz,
          source_x, source_y[i], component_source_z));
    }

    if (!planning_scene_interface_.applyCollisionObjects(objects)) {
      RCLCPP_ERROR(get_logger(), "Failed to apply PCB collision objects to PlanningScene");
      return;
    }

    // ---------------- Visual-only PCB details ----------------
    // Slot centres on the PCB. These are target coordinates, not collision objects.
    const std::array<std::array<double, 2>, 4> slots = {{
      {{-0.045,  0.030}},
      {{ 0.045,  0.030}},
      {{-0.045, -0.030}},
      {{ 0.045, -0.030}}
    }};

    constexpr double pad_dx = 0.015;      // pad centres are 30 mm apart
    constexpr double pad_sx = 0.008;
    constexpr double pad_sy = 0.006;
    constexpr double pad_sz = 0.0008;

    // Raise visual details very slightly above the collision board to avoid z-fighting.
    const double visual_z = pcb_top_z + 0.0012;
    const double pad_z = pcb_top_z + 0.0015;

    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;

    // PCB outer silkscreen border.
    {
      const double hx = 0.5 * pcb_sx - 0.004;
      const double hy = 0.5 * pcb_sy - 0.004;
      std::vector<geometry_msgs::msg::Point> border = {
        makePoint(pcb_x_ - hx, pcb_y_ - hy, visual_z),
        makePoint(pcb_x_ + hx, pcb_y_ - hy, visual_z),
        makePoint(pcb_x_ + hx, pcb_y_ + hy, visual_z),
        makePoint(pcb_x_ - hx, pcb_y_ + hy, visual_z),
        makePoint(pcb_x_ - hx, pcb_y_ - hy, visual_z)
      };
      markers.markers.push_back(makeLineStripMarker(
          marker_id++, "pcb_silkscreen", border,
          0.0012, 0.95f, 0.95f, 0.95f));
    }

    // Four mounting holes + decorative vias. Visual only.
    const std::array<std::array<double, 2>, 4> mounting_holes = {{
      {{-0.080, -0.050}},
      {{ 0.080, -0.050}},
      {{ 0.080,  0.050}},
      {{-0.080,  0.050}}
    }};

    for (const auto& h : mounting_holes) {
      markers.markers.push_back(makeCylinderMarker(
          marker_id++, "pcb_holes",
          pcb_x_ + h[0], pcb_y_ + h[1], pad_z,
          0.006, 0.0010,
          0.05f, 0.05f, 0.05f));
    }

    const std::array<std::array<double, 2>, 8> vias = {{
      {{-0.070,  0.000}}, {{-0.030,  0.000}},
      {{ 0.030,  0.000}}, {{ 0.070,  0.000}},
      {{-0.065,  0.020}}, {{-0.065, -0.020}},
      {{ 0.065,  0.020}}, {{ 0.065, -0.020}}
    }};

    for (const auto& v : vias) {
      markers.markers.push_back(makeCylinderMarker(
          marker_id++, "pcb_vias",
          pcb_x_ + v[0], pcb_y_ + v[1], pad_z,
          0.0035, 0.0009,
          0.12f, 0.12f, 0.12f));
    }

    // Copper-ish traces. These are visual only and intentionally stylised.
    for (std::size_t i = 0; i < slots.size(); ++i) {
      const double slot_x = pcb_x_ + slots[i][0];
      const double slot_y = pcb_y_ + slots[i][1];
      const double pad_left_x = slot_x - pad_dx;
      const double pad_right_x = slot_x + pad_dx;

      // Trace from left pad towards left side of PCB.
      std::vector<geometry_msgs::msg::Point> left_trace = {
        makePoint(pad_left_x, slot_y, visual_z),
        makePoint(pad_left_x - 0.012, slot_y, visual_z),
        makePoint(pad_left_x - 0.012, slot_y + ((slot_y > pcb_y_) ? 0.010 : -0.010), visual_z),
        makePoint(pcb_x_ - 0.075, slot_y + ((slot_y > pcb_y_) ? 0.010 : -0.010), visual_z)
      };
      markers.markers.push_back(makeLineStripMarker(
          marker_id++, "pcb_traces", left_trace,
          0.0018, 0.90f, 0.55f, 0.08f));

      // Trace from right pad towards right side of PCB.
      std::vector<geometry_msgs::msg::Point> right_trace = {
        makePoint(pad_right_x, slot_y, visual_z),
        makePoint(pad_right_x + 0.012, slot_y, visual_z),
        makePoint(pad_right_x + 0.012, slot_y + ((slot_y > pcb_y_) ? -0.010 : 0.010), visual_z),
        makePoint(pcb_x_ + 0.075, slot_y + ((slot_y > pcb_y_) ? -0.010 : 0.010), visual_z)
      };
      markers.markers.push_back(makeLineStripMarker(
          marker_id++, "pcb_traces", right_trace,
          0.0018, 0.90f, 0.55f, 0.08f));
    }

    // Pads + slot outline + labels.
    for (std::size_t i = 0; i < slots.size(); ++i) {
      const double slot_x = pcb_x_ + slots[i][0];
      const double slot_y = pcb_y_ + slots[i][1];

      for (int pad = 0; pad < 2; ++pad) {
        const double sign = (pad == 0) ? -1.0 : 1.0;
        markers.markers.push_back(makeCubeMarker(
            marker_id++, "pcb_pads",
            slot_x + sign * pad_dx, slot_y, pad_z,
            pad_sx, pad_sy, pad_sz,
            1.0f, 0.65f, 0.05f));
      }

      // White rectangular silkscreen around future component location.
      const double hx = 0.5 * component_sx + 0.003;
      const double hy = 0.5 * component_sy + 0.003;
      std::vector<geometry_msgs::msg::Point> footprint = {
        makePoint(slot_x - hx, slot_y - hy, visual_z),
        makePoint(slot_x + hx, slot_y - hy, visual_z),
        makePoint(slot_x + hx, slot_y + hy, visual_z),
        makePoint(slot_x - hx, slot_y + hy, visual_z),
        makePoint(slot_x - hx, slot_y - hy, visual_z)
      };
      markers.markers.push_back(makeLineStripMarker(
          marker_id++, "pcb_footprints", footprint,
          0.0010, 0.95f, 0.95f, 0.95f));

      markers.markers.push_back(makeTextMarker(
          marker_id++, "pcb_slot_labels",
          "U" + std::to_string(i + 1),
          slot_x, slot_y + 0.022, pcb_top_z + 0.003,
          0.009, 0.95f, 0.95f, 0.95f));

      RCLCPP_INFO(
          get_logger(),
          "slot_%zu target=(%.4f %.4f %.4f), pads=(%.4f %.4f)/(%.4f %.4f)",
          i + 1,
          slot_x, slot_y, component_target_z,
          slot_x - pad_dx, slot_y,
          slot_x + pad_dx, slot_y);
    }

    // Board text / polarity cue to make the scene look less like a plain rectangle.
    markers.markers.push_back(makeTextMarker(
        marker_id++, "pcb_labels", "UR3e DUAL SOLDER DEMO",
        pcb_x_, pcb_y_ - 0.052, pcb_top_z + 0.003,
        0.0075, 0.95f, 0.95f, 0.95f));

    // A small orientation marker near +X,+Y corner.
    markers.markers.push_back(makeCubeMarker(
        marker_id++, "pcb_orientation",
        pcb_x_ + 0.078, pcb_y_ + 0.048, pad_z,
        0.006, 0.006, 0.001,
        0.95f, 0.95f, 0.95f));

    marker_pub_->publish(markers);

    RCLCPP_INFO(get_logger(), "==========================================");
    RCLCPP_INFO(get_logger(), " PCB SOLDER SCENE V2 READY");
    RCLCPP_INFO(get_logger(), " Collision: PCB + 4 movable components");
    RCLCPP_INFO(get_logger(), " Visual: traces, 8 pads, holes, vias, silkscreen, footprints");
    RCLCPP_INFO(get_logger(), " Marker topic: /pcb_scene_markers");
    RCLCPP_INFO(get_logger(), " Mapping: component_N -> slot_N");
    RCLCPP_INFO(get_logger(), "==========================================");
  }

  std::string frame_id_;
  double table_top_{-0.003};
  double pcb_x_{0.0};
  double pcb_y_{0.0};

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr spawn_timer_;
};

}  // namespace ur_onrobot_mtc

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur_onrobot_mtc::PcbSceneNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
