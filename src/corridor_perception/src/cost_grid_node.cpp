// Cost grid. SYSTEM UNDER TEST.
// Must never subscribe to Gazebo ground truth.
//
// A bird's eye view grid in base_link, rebuilt from scratch on every scan,
// for the 3d-2 planner to search. No map and no global frame: the grid covers
// only what the vehicle can currently see, and everything in it is placed
// relative to the corridor geometry and the obstacle tracks of this scan.
//
// Two dimensions, not three. The vehicle has no third degree of freedom, so a
// third axis would hold no decision. The height question is settled upstream
// by the extraction z band, which stops at 4.5 m, above the 4.0 m roof.
//
// Geometry:
//   x from 0 to x_max, y from -y_half to +y_half, cell size `resolution`.
//   Cell CENTRES sit at multiples of the resolution, so the vehicle origin
//   (0, 0) is a cell centre. Every replan in 3d-2 starts there.
//   y_half is wider than half the corridor so an offset vehicle still has
//   both walls inside the grid.
//
// Values, as nav_msgs/OccupancyGrid int8:
//   100  lethal: the vehicle centre here would put the body, or a steered
//        wheel, into a wall or an obstacle. Inside tight_radius.
//   99   blocked for NEW paths only: between tight_radius and loose_radius.
//   0..lateral_cost_max (at most 98)
//        lateral cost, quadratic in distance from the fitted centreline, so
//        the cheapest path through an empty corridor is the centre and the
//        vehicle returns there by itself after an obstacle.
//
// The two radii are the hysteresis. The planner treats 99 and above as
// blocked when it plans; the validator treats only 100 as blocked when it
// checks a path it already holds. Centimetre jitter in a measured obstacle
// then cannot flip a held path between valid and invalid every scan.
//
// Radii: tight = 0.9 m half width (outer wheel faces) + 0.1 m steered wheel
// swing at 12 degrees. loose = tight + 0.3 m clearance. The same radii apply
// to the walls and to obstacles: a steered wheel swings out near a wall just
// as near an obstacle.
//
// Obstacles. ObstacleTrack.x is the NEAREST FACE, not the centre (unlike
// Obstacle.x from extraction, which is the box centre). Two corrections are
// applied before inflation, both measured in 3d-1 on the 3c bags:
//
//   Depth is unobservable. The reported length is only what was visible: in
//   single_left it stayed at 0.07 to 0.16 m at every range against a true
//   1.2 m, and even a visible side face never shows the back. So the obstacle
//   is taken to extend from its near face to the end of the grid: a shadow.
//   Unseen is not free.
//
//   Width is short. A true edge falls between beams. Measured per edge: an
//   edge-on corner lost 0.96 to 0.98 of one azimuth step every scan, and an
//   ordinary edge lost up to 1.28 steps between 10 and 20 m of sensor range,
//   mechanism not identified. The worst per-side loss over every scan was
//   0.22 m. lateral_pad is added to each side to cover it; it is a fixed
//   bound, not range dependent.
//
// Nothing is published when the corridor geometry of THIS scan is missing or
// invalid. An all-free grid would assert the corridor is clear, which is not
// the same statement as not knowing where the walls are.
//
// The grid ticks on /obstacle_tracks, which arrives once per scan whenever
// the tracker has a pose, with an empty array when nothing is tracked.
// Tracks and geometry come from the same scan and carry its stamp, so they
// are paired by exact stamp. The two topics have no ordering guarantee: in
// 3d-2 the tracks of a scan reached this node before that scan's geometry on
// about 3 % of scans, and the grid was then built from the previous scan's
// geometry, which once published a grid on a scan whose geometry was
// invalid (criterion P8). So tracks that arrive first are held until their
// geometry arrives; tracks whose geometry has not come within hold_timeout
// are dropped, and nothing is published for them.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <corridor_msgs/msg/corridor_geometry.hpp>
#include <corridor_msgs/msg/obstacle_track_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

class CostGridNode : public rclcpp::Node
{
public:
  CostGridNode() : Node("cost_grid")
  {
    resolution_ = declare_parameter("resolution", 0.25);
    // Matches max_x in obstacle_extraction: the planner cannot look further
    // than detection does.
    x_max_ = declare_parameter("x_max", 30.0);
    y_half_ = declare_parameter("y_half", 6.0);
    tight_radius_ = declare_parameter("tight_radius", 1.0);
    loose_radius_ = declare_parameter("loose_radius", 1.3);
    lateral_pad_ = declare_parameter("lateral_pad", 0.5);
    lateral_cost_max_ = std::clamp<int64_t>(
      declare_parameter("lateral_cost_max", 98), 0, 98);
    hold_timeout_ = declare_parameter("hold_timeout", 0.5);
    frame_ = declare_parameter("frame", std::string("base_link"));

    nx_ = static_cast<int>(std::lround(x_max_ / resolution_)) + 1;
    ny_ = 2 * static_cast<int>(std::lround(y_half_ / resolution_)) + 1;

    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/cost_grid", 10);

    geo_sub_ = create_subscription<corridor_msgs::msg::CorridorGeometry>(
      "/corridor_geometry", rclcpp::SensorDataQoS(),
      std::bind(&CostGridNode::on_geometry, this, std::placeholders::_1));

    // Best effort subscription: compatible with a reliable or a best effort
    // publisher, so it does not depend on how the tracker declared its QoS.
    trk_sub_ = create_subscription<corridor_msgs::msg::ObstacleTrackArray>(
      "/obstacle_tracks", rclcpp::SensorDataQoS(),
      std::bind(&CostGridNode::on_tracks, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "cost_grid up: %d by %d cells at %.2f m, x 0 to %.1f, y +-%.1f, "
      "tight %.2f m, loose %.2f m, lateral pad %.2f m, lateral cost max %ld",
      nx_, ny_, resolution_, x_max_, y_half_, tight_radius_, loose_radius_,
      lateral_pad_, lateral_cost_max_);
  }

private:
  static int64_t stamp_ns(const builtin_interfaces::msg::Time & t)
  {
    return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
  }

  void on_geometry(const corridor_msgs::msg::CorridorGeometry::SharedPtr msg)
  {
    const int64_t s = stamp_ns(msg->header.stamp);
    geo_[s] = msg;
    while (geo_.size() > 40) { geo_.erase(geo_.begin()); }   // 2 s at 20 Hz
    // Tracks of this scan that arrived before its geometry.
    const auto it = held_.find(s);
    if (it != held_.end()) {
      const auto tracks = it->second;
      held_.erase(it);
      build(tracks, *msg);
    }
  }

  void on_tracks(const corridor_msgs::msg::ObstacleTrackArray::SharedPtr msg)
  {
    const int64_t s = stamp_ns(msg->header.stamp);

    // Held tracks whose geometry never came: drop them, publish nothing.
    const int64_t hold_ns = static_cast<int64_t>(hold_timeout_ * 1e9);
    for (auto it = held_.begin(); it != held_.end() && it->first < s - hold_ns;) {
      it = held_.erase(it);
      ++dropped_;
    }

    const auto g = geo_.find(s);
    if (g == geo_.end()) {
      held_[s] = msg;
      ++held_count_;
      return;
    }
    build(msg, *g->second);
  }

  void build(const corridor_msgs::msg::ObstacleTrackArray::SharedPtr & msg,
             const corridor_msgs::msg::CorridorGeometry & geo)
  {
    if (!geo.valid) {
      ++silent_invalid_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "no valid corridor geometry for this scan: not publishing a cost grid");
      return;
    }

    const auto t0 = std::chrono::steady_clock::now();

    // Centreline in base_link, inverting corridor_geometry exactly as
    // obstacle_extraction does: heading = -atan(m), offset = -c / sqrt(1 + m^2).
    const double m = -std::tan(geo.heading_error);
    const double norm = std::sqrt(1.0 + m * m);
    const double c = -geo.lateral_offset * norm;
    const double half_w = 0.5 * geo.corridor_width;

    // Distances from the centreline beyond which a wall is lethal, and beyond
    // which a new path may not go.
    const double wall_lethal = half_w - tight_radius_;
    const double wall_block = half_w - loose_radius_;
    if (wall_block <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "corridor width %.2f m leaves no plannable band: not publishing",
        geo.corridor_width);
      return;
    }

    // Each obstacle as a padded, shadowed region: x from the near face to
    // infinity, y across the measured width plus the pad on each side.
    struct Region { double x0, y_lo, y_hi; };
    std::vector<Region> regions;
    regions.reserve(msg->tracks.size());
    for (const auto & t : msg->tracks) {
      const double half = 0.5 * t.width + lateral_pad_;
      regions.push_back({t.x, t.y - half, t.y + half});
    }

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = msg->header.stamp;
    grid.header.frame_id = frame_;
    grid.info.map_load_time = msg->header.stamp;
    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = static_cast<uint32_t>(nx_);
    grid.info.height = static_cast<uint32_t>(ny_);
    // The origin is the CORNER of cell (0, 0), half a cell below its centre.
    grid.info.origin.position.x = -0.5 * resolution_;
    grid.info.origin.position.y = -y_half_ - 0.5 * resolution_;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<size_t>(nx_) * static_cast<size_t>(ny_), 0);

    const double cost_scale = static_cast<double>(lateral_cost_max_);

    for (int j = 0; j < ny_; ++j) {
      const double y = -y_half_ + j * resolution_;
      for (int i = 0; i < nx_; ++i) {
        const double x = i * resolution_;
        int value;

        // Signed perpendicular distance from the centreline, positive left.
        const double d = (y - (m * x + c)) / norm;
        const double ad = std::abs(d);
        if (ad > wall_lethal) {
          value = 100;
        } else if (ad > wall_block) {
          value = 99;
        } else {
          const double r = ad / wall_block;
          value = static_cast<int>(std::lround(cost_scale * r * r));
        }

        // Obstacles only ever raise a cell, never lower it.
        for (const Region & g : regions) {
          if (value == 100) { break; }
          const double dx = std::max(0.0, g.x0 - x);
          const double dy = std::max({0.0, g.y_lo - y, y - g.y_hi});
          const double dist = std::hypot(dx, dy);
          if (dist <= tight_radius_) {
            value = 100;
          } else if (dist <= loose_radius_) {
            value = std::max(value, 99);
          }
        }

        grid.data[static_cast<size_t>(j) * static_cast<size_t>(nx_) +
                  static_cast<size_t>(i)] = static_cast<int8_t>(value);
      }
    }

    pub_->publish(grid);

    // Build time for C4, from the monotonic clock, never wall clock.
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    build_ms_.push_back(ms);
    if (build_ms_.size() % 200 == 0) {
      std::vector<double> s = build_ms_;
      std::sort(s.begin(), s.end());
      const auto at = [&s](double q) {
        return s[std::min(s.size() - 1, static_cast<size_t>(q * static_cast<double>(s.size())))];
      };
      RCLCPP_INFO(get_logger(),
        "build time over %zu grids: median %.3f ms, p99 %.3f ms, max %.3f ms; "
        "tracks held for their geometry %zu, dropped %zu; scans silent on invalid geometry %zu",
        s.size(), at(0.5), at(0.99), s.back(), held_count_, dropped_, silent_invalid_);
    }
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Subscription<corridor_msgs::msg::CorridorGeometry>::SharedPtr geo_sub_;
  rclcpp::Subscription<corridor_msgs::msg::ObstacleTrackArray>::SharedPtr trk_sub_;

  std::map<int64_t, corridor_msgs::msg::CorridorGeometry::SharedPtr> geo_;     // by stamp
  std::map<int64_t, corridor_msgs::msg::ObstacleTrackArray::SharedPtr> held_; // by stamp
  size_t held_count_ = 0, dropped_ = 0, silent_invalid_ = 0;

  double resolution_, x_max_, y_half_, tight_radius_, loose_radius_;
  double lateral_pad_, hold_timeout_;
  int64_t lateral_cost_max_;
  std::string frame_;
  int nx_ = 0, ny_ = 0;
  std::vector<double> build_ms_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostGridNode>());
  rclcpp::shutdown();
  return 0;
}
