// Obstacle extraction. SYSTEM UNDER TEST.
// Must never subscribe to Gazebo ground truth.
//
// Consumes one LiDAR scan and the corridor geometry from the same scan, and
// produces the obstacles in base_link. Per scan only: nothing is tracked or
// accumulated here. That is deliberate. An obstacle is re-measured every
// 50 ms, so a rolling horizon never needs to remember where one was, and
// longitudinal drift cannot accumulate into an avoidance decision.
//
// Method:
//   1. transform the cloud into base_link, as corridor_geometry does
//   2. keep points with z in [z_min, z_max]: above the floor, below the
//      vehicle clearance envelope
//   3. drop points within wall_margin of either wall, using the corridor
//      geometry rather than re-fitting the walls here, so there is one
//      source of truth for where the walls are
//   4. bucket what remains into cells and connect occupied neighbours
//   5. publish each cluster's bounding box
//
// A uniform grid, not DBSCAN. An obstacle at 30 m returns 5 to 10 points on
// one or two layers, and at 10 m about 40. With counts that small there is
// nothing for a density based method to earn: the grid is linear in point
// count and has one parameter.
//
// The scope assumption of this phase is that detection is solved. That is
// honoured by segmenting geometrically from the point cloud, never by reading
// obstacle poses from the simulator: taking them from Gazebo would put ground
// truth inside the system under test and make every later metric meaningless.
//
// Known limitation: an obstacle closer than wall_margin to a wall is absorbed
// into that wall's fit upstream, which corrupts the corridor geometry the EKF
// uses, and its own points are then dropped here as wall points. The scenario
// worlds keep every obstacle at least 1 m clear of both walls. Obstacles
// against a wall are a separate experiment, not a case this node handles.
//
// Nothing is published when the corridor geometry is missing, stale or
// invalid. An empty array would assert that the corridor is clear, which is
// not the same statement as not knowing where the walls are.
 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <corridor_msgs/msg/corridor_geometry.hpp>
#include <corridor_msgs/msg/obstacle.hpp>
#include <corridor_msgs/msg/obstacle_array.hpp>
 
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>
 
namespace
{
struct Point2
{
  double x = 0.0;
  double y = 0.0;
};
 
// Cell coordinates packed into one key. The grid is sparse: a scan leaves at
// most a few hundred surviving points, so a hash map costs less than an array
// spanning the sensor's whole field of view.
inline int64_t cell_key(int32_t cx, int32_t cy)
{
  return (static_cast<int64_t>(cx) << 32) ^ static_cast<uint32_t>(cy);
}
}  // namespace
 
class ObstacleExtractionNode : public rclcpp::Node
{
public:
  ObstacleExtractionNode() : Node("obstacle_extraction")
  {
    z_min_ = declare_parameter("z_min", 0.1);
    z_max_ = declare_parameter("z_max", 5.0);
    // Same value the wall fit rejects at, so a point is either a wall point
    // upstream or an obstacle point here, never both and never neither.
    wall_margin_ = declare_parameter("wall_margin", 0.30);
    cell_size_ = declare_parameter("cell_size", 0.25);
    min_cluster_points_ = declare_parameter("min_cluster_points", 4);
    max_x_ = declare_parameter("max_x", 30.0);
    geometry_timeout_ = declare_parameter("geometry_timeout", 0.2);
    target_frame_ = declare_parameter("target_frame", std::string("base_link"));
 
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
 
    pub_ = create_publisher<corridor_msgs::msg::ObstacleArray>("/obstacles", 10);
 
    geo_sub_ = create_subscription<corridor_msgs::msg::CorridorGeometry>(
      "/corridor_geometry", rclcpp::SensorDataQoS(),
      [this](const corridor_msgs::msg::CorridorGeometry::SharedPtr msg) {
        last_geo_ = *msg;
        have_geo_ = true;
      });
 
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/scan/points", rclcpp::SensorDataQoS(),
      std::bind(&ObstacleExtractionNode::on_cloud, this, std::placeholders::_1));
 
    RCLCPP_INFO(get_logger(),
      "obstacle_extraction up: z band [%.2f, %.2f] m, wall margin %.2f m, "
      "cell %.2f m, min %ld points per cluster, max x %.1f m, frame %s",
      z_min_, z_max_, wall_margin_, cell_size_, min_cluster_points_, max_x_,
      target_frame_.c_str());
  }
 
private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!have_geo_ || !last_geo_.valid) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "no valid corridor geometry: not publishing obstacles");
      return;
    }
    const double age =
      (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_geo_.header.stamp)).seconds();
    if (std::abs(age) > geometry_timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "corridor geometry is %.3f s from this scan: not publishing obstacles", age);
      return;
    }
 
    sensor_msgs::msg::PointCloud2 cloud;
    try {
      const auto tf = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, tf2::TimePointZero);
      tf2::doTransform(*msg, cloud, tf);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "transform unavailable: %s", ex.what());
      return;
    }
 
    // Rebuild the centreline in base_link from the published geometry.
    // corridor_geometry computes heading as -atan(m) and offset as
    // -c / sqrt(1 + m^2), so this inverts exactly that.
    const double m = -std::tan(last_geo_.heading_error);
    const double norm = std::sqrt(1.0 + m * m);
    const double c = -last_geo_.lateral_offset * norm;
    const double half_width = 0.5 * last_geo_.corridor_width;
    // A point is kept only if it sits this far inside both walls.
    const double inner = half_width - wall_margin_;
    if (inner <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "corridor width %.2f m leaves nothing inside the wall margin", half_width * 2.0);
      return;
    }
 
    std::vector<Point2> pts;
    std::unordered_map<int64_t, std::vector<uint32_t>> grid;
 
    sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");
 
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const double x = *it_x, y = *it_y, z = *it_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) { continue; }
      if (z < z_min_ || z > z_max_) { continue; }
      if (x < 0.0 || x > max_x_) { continue; }
 
      // Signed perpendicular distance from the centreline, positive left.
      const double d = (y - (m * x + c)) / norm;
      if (std::abs(d) > inner) { continue; }       // wall, or too close to one
 
      const uint32_t idx = static_cast<uint32_t>(pts.size());
      pts.push_back({x, y});
      grid[cell_key(static_cast<int32_t>(std::floor(x / cell_size_)),
                    static_cast<int32_t>(std::floor(y / cell_size_)))].push_back(idx);
    }
 
    corridor_msgs::msg::ObstacleArray out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = target_frame_;
 
    // Connect occupied cells to their eight neighbours, breadth first.
    std::unordered_map<int64_t, bool> visited;
    visited.reserve(grid.size());
 
    for (const auto & entry : grid) {
      if (visited[entry.first]) { continue; }
 
      std::vector<int64_t> queue{entry.first};
      visited[entry.first] = true;
      std::vector<uint32_t> members;
 
      for (size_t qi = 0; qi < queue.size(); ++qi) {
        const int64_t key = queue[qi];
        const auto found = grid.find(key);
        if (found == grid.end()) { continue; }
        members.insert(members.end(), found->second.begin(), found->second.end());
 
        const int32_t cx = static_cast<int32_t>(key >> 32);
        const int32_t cy = static_cast<int32_t>(static_cast<uint32_t>(key & 0xffffffff));
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) { continue; }
            const int64_t nk = cell_key(cx + dx, cy + dy);
            if (grid.count(nk) && !visited[nk]) {
              visited[nk] = true;
              queue.push_back(nk);
            }
          }
        }
      }
 
      if (static_cast<int64_t>(members.size()) < min_cluster_points_) { continue; }
 
      double min_x = pts[members[0]].x, max_x = min_x;
      double min_y = pts[members[0]].y, max_y = min_y;
      for (const uint32_t i : members) {
        min_x = std::min(min_x, pts[i].x);
        max_x = std::max(max_x, pts[i].x);
        min_y = std::min(min_y, pts[i].y);
        max_y = std::max(max_y, pts[i].y);
      }
 
      corridor_msgs::msg::Obstacle ob;
      // Centre of the bounding box, not the centroid: the LiDAR only sees the
      // near faces, so the centroid sits on the surface rather than the body.
      // Neither is the true centre of a partly seen object, and the box is at
      // least a stated convention rather than a biased estimate.
      ob.x = 0.5 * (min_x + max_x);
      ob.y = 0.5 * (min_y + max_y);
      ob.length = max_x - min_x;
      ob.width = max_y - min_y;
      // The nearest returned point, which is a conservative bound on the
      // distance rather than an unbiased estimate of it. Under range noise the
      // minimum over N points picks the most negative noise sample, so it is
      // biased low by roughly sigma * sqrt(2 ln N): about 80 mm measured at
      // 200 points with 30 mm range noise, falling to 36 mm at 6 points. That
      // reports the obstacle nearer than it is, which is the right direction
      // to be wrong for avoidance, and is why this is the field a planner
      // should act on.
      ob.min_x = min_x;
      ob.n_points = static_cast<uint32_t>(members.size());
      out.obstacles.push_back(ob);
    }
 
    pub_->publish(out);
  }
 
  rclcpp::Publisher<corridor_msgs::msg::ObstacleArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<corridor_msgs::msg::CorridorGeometry>::SharedPtr geo_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
 
  corridor_msgs::msg::CorridorGeometry last_geo_;
  bool have_geo_ = false;
 
  double z_min_, z_max_, wall_margin_, cell_size_, max_x_, geometry_timeout_;
  int64_t min_cluster_points_;
  std::string target_frame_;
};
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleExtractionNode>());
  rclcpp::shutdown();
  return 0;
}
 