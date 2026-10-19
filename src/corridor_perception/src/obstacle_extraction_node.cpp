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
//   5. merge clusters whose boxes are closer than a vehicle width
//   6. publish each merged cluster's bounding box
//
// A uniform grid, not DBSCAN. An obstacle at 30 m returns 5 to 10 points on
// one or two layers, and at 10 m about 40. With counts that small there is
// nothing for a density based method to earn: the grid is linear in point
// count and has one parameter.
//
// Step 6 is a merge, and it is not cosmetic. A box offset laterally from the
// sensor shows its side face as well as its front face, and along a surface
// nearly parallel to the beam a fixed azimuth step projects into a very large
// spacing in x: at 11 m the side face of a 1.2 m box returns four or five
// points spread about 0.3 m apart, which exceeds cell_size. Those points
// cannot connect to each other or to the front face, so one obstacle is
// reported as two, and the far one is a thin cluster of constant y sitting
// behind the near face. Measured in the slalom scenario: 73 percent of scans
// between 5 and 10 m split, falling to zero beyond 15 m, because the side
// face only comes into view as the obstacle closes.
//
// This was invisible in the single_left scenario, where the obstacle sits at
// y 0.6 and is 1.2 m wide, putting its side face at y 0.0, exactly in the
// sensor plane and therefore unseen. One lateral offset in the scenario set
// happened to hide the defect entirely.
//
// Merging is preferred to a larger cell_size or a higher
// min_cluster_points. A higher threshold would also discard every genuine
// obstacle beyond 25 m, where the true cluster holds about 6 points. A larger
// cell would hide this case while merging unrelated obstacles at a distance
// set by nothing in particular. The merge threshold instead has a physical
// meaning: it is the narrowest gap the vehicle could pass through, so two
// clusters closer than that cannot be driven between whatever they are, and
// collapsing them loses a distinction a planner could not have used.
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
//
// Test hook: a forced detection outage, so the tracker's ego-motion prediction
// can be exercised against truth. It exists because a sign error in the
// tracker's motion transform passes every static criterion while being the one
// bug this stage is here to catch, and the only way to see it is to stop
// feeding the tracker measurements and watch where it thinks the obstacle went.
//
// Two mechanisms, both off by default, so a run with no parameters set is
// exactly the node 3b verified. A hook wired on in normal runs would mean the
// verified node and the shipped node are different nodes.
//
//   drop_prob, drop_min_run, drop_max_run, drop_seed
//       Random onset, random length. Each scan that is not already inside an
//       outage begins one with probability drop_prob, lasting a uniformly
//       drawn drop_min_run to drop_max_run scans. This is the realistic case:
//       occlusion, dust and a wet or dark surface produce bursts of lost
//       returns at unpredictable moments, not a metronome.
//
//   drop_every, drop_run
//       Strictly periodic. Useful when a specific failure needs reproducing at
//       a known scan index, and for a first sanity check that the coasting
//       path runs at all.
//
// The generator is seeded, not seeded from the clock. Random and reproducible
// are not in conflict here, and they must both hold: an outage pattern that
// changes every run turns a failure into something that cannot be debugged,
// while a pattern that never changes only ever tests one alignment of outage
// against range. Vary the seed across runs, keep the seed in the run log.
 
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
#include <random>
#include <unordered_map>
#include <vector>
 
namespace
{
struct Point2
{
  double x = 0.0;
  double y = 0.0;
};
 
// A cluster's axis aligned extent, kept separately from the message so the
// merge step has something to work on before anything is published.
struct Box
{
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  uint32_t n = 0;
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
    // The narrowest gap the vehicle could use. Two clusters closer than this
    // are one obstacle as far as any planner is concerned. Set below the
    // vehicle width, not above it, so a genuinely passable gap survives.
    //
    // 1.0 m, not the 0.5 m first tried. The side face of an offset box does
    // not return a face: at most bearings the beam misses it entirely, and
    // where it does strike, several layers land at one azimuth and produce a
    // vertical column whose box is almost a single point. Measured in the
    // slalom scenario, those columns sit about 0.72 m behind the near face, so
    // a 0.5 m threshold left them separate on 4 percent of scans. Runs of four
    // or five consecutive split scans are enough to promote a fragment past
    // confirm_hits, so a small residual split rate produces confirmed phantom
    // tracks rather than a small error.
    merge_gap_ = declare_parameter("merge_gap", 1.0);
    max_x_ = declare_parameter("max_x", 30.0);
    geometry_timeout_ = declare_parameter("geometry_timeout", 0.2);
    target_frame_ = declare_parameter("target_frame", std::string("base_link"));
    // Test hook, both mechanisms off at 0. See the note at the top.
    drop_every_ = declare_parameter("drop_every", 0);
    drop_run_ = declare_parameter("drop_run", 0);
    drop_prob_ = declare_parameter("drop_prob", 0.0);
    drop_min_run_ = declare_parameter("drop_min_run", 4);
    drop_max_run_ = declare_parameter("drop_max_run", 20);
    drop_seed_ = declare_parameter("drop_seed", 42);
    rng_.seed(static_cast<uint32_t>(drop_seed_));
 
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
      "cell %.2f m, min %ld points per cluster, merge gap %.2f m, max x %.1f m, "
      "frame %s",
      z_min_, z_max_, wall_margin_, cell_size_, min_cluster_points_, merge_gap_,
      max_x_, target_frame_.c_str());
    if (drop_every_ > 0 && drop_run_ > 0) {
      RCLCPP_WARN(get_logger(),
        "TEST HOOK ACTIVE: periodic, dropping %ld of every %ld scans. "
        "This is not a normal run.", drop_run_, drop_every_);
    }
    if (drop_prob_ > 0.0) {
      RCLCPP_WARN(get_logger(),
        "TEST HOOK ACTIVE: random, onset probability %.3f per scan, run %ld to "
        "%ld scans, seed %ld. This is not a normal run.",
        drop_prob_, drop_min_run_, drop_max_run_, drop_seed_);
    }
  }
 
private:
  // True when this scan should be suppressed. Counts the scan either way, so
  // the periodic mechanism stays aligned with the scan index and the log line
  // reports a true count.
  bool dropping()
  {
    const int64_t n = scan_count_++;
 
    if (drop_every_ > 0 && drop_run_ > 0) {
      if ((n % drop_every_) >= (drop_every_ - drop_run_)) {
        ++dropped_;
        return true;
      }
    }
 
    if (drop_prob_ > 0.0) {
      if (outage_left_ > 0) {
        --outage_left_;
        ++dropped_;
        return true;
      }
      std::uniform_real_distribution<double> u(0.0, 1.0);
      if (u(rng_) < drop_prob_) {
        const int64_t lo = std::min(drop_min_run_, drop_max_run_);
        const int64_t hi = std::max(drop_min_run_, drop_max_run_);
        std::uniform_int_distribution<int64_t> len(lo, hi);
        // One scan of the drawn run is this one, so the remainder is len - 1.
        outage_left_ = len(rng_) - 1;
        ++dropped_;
        RCLCPP_INFO(get_logger(),
          "test hook: outage starting at scan %ld, %ld scans", n, outage_left_ + 1);
        return true;
      }
    }
 
    return false;
  }
 
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Forced outage. An EMPTY array, not silence, and the difference is the
    // whole test.
    //
    // Silence means "I do not know where the walls are" and is what this node
    // publishes when the geometry is missing or stale. An empty array means
    // "the scan arrived, I looked, nothing clustered", which is what dust, an
    // absorbing wet surface or a low reflectivity target actually produce. The
    // outage being modelled is the second case.
    //
    // It also matters mechanically. The tracker's only tick is the arrival of
    // an ObstacleArray, so suppressing the message entirely does not make the
    // tracker coast, it stops the tracker dead: no prediction, no miss
    // counted, and no track message either. The dropout then removes one
    // message from each topic and is invisible to any test comparing the two.
    if (dropping()) {
      corridor_msgs::msg::ObstacleArray empty;
      empty.header.stamp = msg->header.stamp;
      empty.header.frame_id = target_frame_;
      pub_->publish(empty);
      return;
    }
 
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
 
    std::vector<Box> boxes;
 
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
 
      Box box;
      box.min_x = box.max_x = pts[members[0]].x;
      box.min_y = box.max_y = pts[members[0]].y;
      for (const uint32_t i : members) {
        box.min_x = std::min(box.min_x, pts[i].x);
        box.max_x = std::max(box.max_x, pts[i].x);
        box.min_y = std::min(box.min_y, pts[i].y);
        box.max_y = std::max(box.max_y, pts[i].y);
      }
      box.n = static_cast<uint32_t>(members.size());
      boxes.push_back(box);
    }
 
    // Merge boxes separated by less than merge_gap, repeating until nothing
    // changes, since a merge can bring a third box within reach. Clusters are
    // few, so the quadratic pass costs nothing.
    bool merged_any = true;
    while (merged_any) {
      merged_any = false;
      for (size_t i = 0; i < boxes.size() && !merged_any; ++i) {
        for (size_t j = i + 1; j < boxes.size(); ++j) {
          // Gap between boxes per axis, zero when they overlap on that axis.
          const double gx = std::max(0.0, std::max(boxes[i].min_x - boxes[j].max_x,
                                                   boxes[j].min_x - boxes[i].max_x));
          const double gy = std::max(0.0, std::max(boxes[i].min_y - boxes[j].max_y,
                                                   boxes[j].min_y - boxes[i].max_y));
          if (std::hypot(gx, gy) > merge_gap_) { continue; }
          boxes[i].min_x = std::min(boxes[i].min_x, boxes[j].min_x);
          boxes[i].max_x = std::max(boxes[i].max_x, boxes[j].max_x);
          boxes[i].min_y = std::min(boxes[i].min_y, boxes[j].min_y);
          boxes[i].max_y = std::max(boxes[i].max_y, boxes[j].max_y);
          boxes[i].n += boxes[j].n;
          boxes.erase(boxes.begin() + static_cast<long>(j));
          merged_any = true;
          break;
        }
      }
    }
 
    for (const Box & b : boxes) {
      // The point threshold is applied after merging, not before. A side face
      // contributes only four or five returns, and discarding it first would
      // throw away the very points that make the box the right size.
      if (static_cast<int64_t>(b.n) < min_cluster_points_) { continue; }
      const double min_x = b.min_x, max_x = b.max_x;
      const double min_y = b.min_y, max_y = b.max_y;
 
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
      ob.n_points = b.n;
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
  double merge_gap_ = 0.5;
  int64_t min_cluster_points_;
  int64_t drop_every_ = 0, drop_run_ = 0, scan_count_ = 0, dropped_ = 0;
  int64_t drop_min_run_ = 0, drop_max_run_ = 0, drop_seed_ = 0, outage_left_ = 0;
  double drop_prob_ = 0.0;
  std::mt19937 rng_;
  std::string target_frame_;
};
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleExtractionNode>());
  rclcpp::shutdown();
  return 0;
}
 