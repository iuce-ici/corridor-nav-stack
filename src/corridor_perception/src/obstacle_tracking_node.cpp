// Obstacle tracking. SYSTEM UNDER TEST.
// Must never subscribe to Gazebo ground truth.
//
// Carries obstacle detections across scans using the EKF's motion estimate.
// This is the step the detector cannot do alone: a detection exists for 50 ms
// and the next scan brings a fresh, unlabelled set of clusters. Knowing that
// this cluster is the same obstacle as that one requires knowing how far the
// vehicle moved in between, which is what the EKF provides.
//
// Frames. Tracks live in base_link and are carried forward by ego motion, not
// held in any persistent map frame. There is no global map and no GPS in a
// tunnel, so none is built here. Between consecutive scans the EKF's motion
// estimate is excellent and errors do not compound, because every track is
// re-measured 20 times a second.
//
// The transform between two base_link frames, from EKF poses in the corridor
// aligned frame: a point p in the previous frame maps to
//   p' = R(theta0 - theta1) p + R(-theta1) (t0 - t1)
// which is a rotation by the heading change and a translation by the motion
// expressed in the new frame.
//
// Each track is a two state Kalman filter on position. The obstacle is static,
// so there is no velocity state: all of the apparent motion is the vehicle's,
// and it is removed by the prediction rather than estimated. Process noise
// covers what the ego motion estimate got wrong over one scan interval.
//
// Measurement is (min_x, y): the nearest face and the lateral centre. Note
// that min_x carries a slowly varying bias, since the minimum over N noisy
// points is low by about sigma * sqrt(2 ln N) and N grows as the vehicle
// approaches: roughly 36 mm at 6 points and 82 mm at 217. The obstacle
// therefore appears to close very slightly faster than it does. The bias is
// systematic rather than random, so no filter removes it; it is small against
// the clearance a planner uses, and it errs toward caution.
//
// Nothing is published when the EKF pose for a scan is missing or stale.
// Publishing tracks that were not predicted forward would present stale
// positions as current, which is worse than publishing nothing.
 
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <corridor_msgs/msg/obstacle_array.hpp>
#include <corridor_msgs/msg/obstacle_track.hpp>
#include <corridor_msgs/msg/obstacle_track_array.hpp>
 
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>
 
namespace
{
struct Pose2
{
  double t = 0.0;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};
 
struct Track
{
  uint32_t id = 0;
  double x = 0.0, y = 0.0;             // mean, base_link
  double pxx = 0.0, pxy = 0.0, pyy = 0.0;   // covariance, symmetric
  double length = 0.0, width = 0.0, min_x = 0.0;
  uint32_t n_points = 0;
  uint32_t age = 0, hits = 0, misses = 0;
  bool confirmed = false;
};
}  // namespace
 
class ObstacleTrackingNode : public rclcpp::Node
{
public:
  ObstacleTrackingNode() : Node("obstacle_tracking")
  {
    // Per scan growth of position uncertainty, metres. Covers what the ego
    // motion estimate got wrong over one 50 ms interval, which is small: the
    // EKF holds lateral error to tens of microns and heading to microradians.
    process_noise_ = declare_parameter("process_noise", 0.02);
    // Measurement noise, from the 3b detection run: lateral error stayed
    // within 54 mm and the near face within 125 mm including its bias.
    meas_noise_x_ = declare_parameter("meas_noise_x", 0.05);
    meas_noise_y_ = declare_parameter("meas_noise_y", 0.05);
    // Chi squared with two degrees of freedom at 99 percent.
    gate_ = declare_parameter("gate", 9.21);
    confirm_hits_ = declare_parameter("confirm_hits", 3);
    max_misses_ = declare_parameter("max_misses", 10);
    pose_timeout_ = declare_parameter("pose_timeout", 0.1);
    // Initial uncertainty of a new track, metres.
    init_sigma_ = declare_parameter("init_sigma", 0.5);
 
    pub_ = create_publisher<corridor_msgs::msg::ObstacleTrackArray>(
      "/obstacle_tracks", 10);
 
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom_ekf", 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        Pose2 p;
        p.t = rclcpp::Time(msg->header.stamp).seconds();
        p.x = msg->pose.pose.position.x;
        p.y = msg->pose.pose.position.y;
        p.theta = 2.0 * std::atan2(msg->pose.pose.orientation.z,
                                   msg->pose.pose.orientation.w);
        poses_.push_back(p);
        while (poses_.size() > 400) { poses_.pop_front(); }
      });
 
    sub_ = create_subscription<corridor_msgs::msg::ObstacleArray>(
      "/obstacles", rclcpp::SensorDataQoS(),
      std::bind(&ObstacleTrackingNode::on_obstacles, this, std::placeholders::_1));
 
    RCLCPP_INFO(get_logger(),
      "obstacle_tracking up: process noise %.3f m, measurement noise "
      "%.3f by %.3f m, gate %.2f, confirm after %ld hits, drop after %ld misses",
      process_noise_, meas_noise_x_, meas_noise_y_, gate_,
      confirm_hits_, max_misses_);
  }
 
private:
  // Nearest recorded pose to a stamp, if one is close enough in time.
  bool pose_at(double t, Pose2 & out) const
  {
    if (poses_.empty()) { return false; }
    const Pose2 * best = nullptr;
    double best_dt = std::numeric_limits<double>::max();
    for (const auto & p : poses_) {
      const double dt = std::abs(p.t - t);
      if (dt < best_dt) { best_dt = dt; best = &p; }
    }
    if (best == nullptr || best_dt > pose_timeout_) { return false; }
    out = *best;
    return true;
  }
 
  void on_obstacles(const corridor_msgs::msg::ObstacleArray::SharedPtr msg)
  {
    const double t = rclcpp::Time(msg->header.stamp).seconds();
    Pose2 now;
    if (!pose_at(t, now)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "no EKF pose within %.3f s of this scan: holding tracks, publishing nothing",
        pose_timeout_);
      return;
    }
 
    if (have_prev_) {
      // Carry every track from the previous base_link into this one.
      const double dtheta = prev_.theta - now.theta;
      const double c1 = std::cos(-now.theta), s1 = std::sin(-now.theta);
      const double dx_w = prev_.x - now.x, dy_w = prev_.y - now.y;
      const double tx = c1 * dx_w - s1 * dy_w;
      const double ty = s1 * dx_w + c1 * dy_w;
      const double c = std::cos(dtheta), s = std::sin(dtheta);
      const double q = process_noise_ * process_noise_;
 
      for (auto & tr : tracks_) {
        const double nx = c * tr.x - s * tr.y + tx;
        const double ny = s * tr.x + c * tr.y + ty;
        tr.x = nx;
        tr.y = ny;
        // P' = R P R^T + Q, written out for a 2 by 2 symmetric matrix.
        const double a = tr.pxx, b = tr.pxy, d = tr.pyy;
        tr.pxx = c * c * a - 2.0 * c * s * b + s * s * d + q;
        tr.pxy = c * s * a + (c * c - s * s) * b - c * s * d;
        tr.pyy = s * s * a + 2.0 * c * s * b + c * c * d + q;
      }
    }
    prev_ = now;
    have_prev_ = true;
 
    // Associate: for each track, the nearest unclaimed detection inside the
    // gate. Greedy rather than optimal. With obstacles metres apart in an 8 m
    // bore, the assignment is never ambiguous enough to need Hungarian.
    const size_t nd = msg->obstacles.size();
    std::vector<bool> claimed(nd, false);
    const double rx = meas_noise_x_ * meas_noise_x_;
    const double ry = meas_noise_y_ * meas_noise_y_;
 
    for (auto & tr : tracks_) {
      ++tr.age;
      size_t best = nd;
      double best_d2 = gate_;
 
      for (size_t j = 0; j < nd; ++j) {
        if (claimed[j]) { continue; }
        const auto & ob = msg->obstacles[j];
        const double ex = ob.min_x - tr.x;
        const double ey = ob.y - tr.y;
        // Innovation covariance S = P + R, then the Mahalanobis distance.
        const double sxx = tr.pxx + rx, sxy = tr.pxy, syy = tr.pyy + ry;
        const double det = sxx * syy - sxy * sxy;
        if (det <= 0.0) { continue; }
        const double d2 = (ex * ex * syy - 2.0 * ex * ey * sxy + ey * ey * sxx) / det;
        if (d2 < best_d2) { best_d2 = d2; best = j; }
      }
 
      if (best == nd) {
        ++tr.misses;
        continue;
      }
 
      claimed[best] = true;
      tr.misses = 0;
      ++tr.hits;
      if (!tr.confirmed && static_cast<int64_t>(tr.hits) >= confirm_hits_) {
        tr.confirmed = true;
      }
 
      const auto & ob = msg->obstacles[best];
      // Standard Kalman update with H the identity: K = P (P + R)^-1.
      const double sxx = tr.pxx + rx, sxy = tr.pxy, syy = tr.pyy + ry;
      const double det = sxx * syy - sxy * sxy;
      const double i00 = syy / det, i01 = -sxy / det, i11 = sxx / det;
      const double k00 = tr.pxx * i00 + tr.pxy * i01;
      const double k01 = tr.pxx * i01 + tr.pxy * i11;
      const double k10 = tr.pxy * i00 + tr.pyy * i01;
      const double k11 = tr.pxy * i01 + tr.pyy * i11;
 
      const double ex = ob.min_x - tr.x, ey = ob.y - tr.y;
      tr.x += k00 * ex + k01 * ey;
      tr.y += k10 * ex + k11 * ey;
 
      // P = (I - K) P, with I - K applied on the left.
      const double a = tr.pxx, b = tr.pxy, d = tr.pyy;
      const double m00 = 1.0 - k00, m01 = -k01, m10 = -k10, m11 = 1.0 - k11;
      tr.pxx = m00 * a + m01 * b;
      tr.pxy = m00 * b + m01 * d;
      const double pyx = m10 * a + m11 * b;
      tr.pyy = m10 * b + m11 * d;
      tr.pxy = 0.5 * (tr.pxy + pyx);     // keep it symmetric under rounding
 
      tr.length = ob.length;
      tr.width = ob.width;
      tr.min_x = ob.min_x;
      tr.n_points = ob.n_points;
    }
 
    // Unclaimed detections start new tracks, unconfirmed until they persist.
    for (size_t j = 0; j < nd; ++j) {
      if (claimed[j]) { continue; }
      const auto & ob = msg->obstacles[j];
      Track tr;
      tr.id = next_id_++;
      tr.x = ob.min_x;
      tr.y = ob.y;
      tr.pxx = tr.pyy = init_sigma_ * init_sigma_;
      tr.pxy = 0.0;
      tr.length = ob.length;
      tr.width = ob.width;
      tr.min_x = ob.min_x;
      tr.n_points = ob.n_points;
      tr.age = 1;
      tr.hits = 1;
      tracks_.push_back(tr);
    }
 
    tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
        [this](const Track & tr) {
          return static_cast<int64_t>(tr.misses) > max_misses_;
        }),
      tracks_.end());
 
    corridor_msgs::msg::ObstacleTrackArray out;
    out.header = msg->header;
    for (const auto & tr : tracks_) {
      if (!tr.confirmed) { continue; }
      corridor_msgs::msg::ObstacleTrack o;
      o.id = tr.id;
      o.x = tr.x;
      o.y = tr.y;
      o.length = tr.length;
      o.width = tr.width;
      o.min_x = tr.min_x;
      o.n_points = tr.n_points;
      o.age = tr.age;
      o.hits = tr.hits;
      o.misses = tr.misses;
      o.confirmed = tr.confirmed;
      out.tracks.push_back(o);
    }
    pub_->publish(out);
  }
 
  rclcpp::Publisher<corridor_msgs::msg::ObstacleTrackArray>::SharedPtr pub_;
  rclcpp::Subscription<corridor_msgs::msg::ObstacleArray>::SharedPtr sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
 
  std::deque<Pose2> poses_;
  std::vector<Track> tracks_;
  Pose2 prev_;
  bool have_prev_ = false;
  uint32_t next_id_ = 1;
 
  double process_noise_, meas_noise_x_, meas_noise_y_, gate_, pose_timeout_, init_sigma_;
  int64_t confirm_hits_, max_misses_;
};
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleTrackingNode>());
  rclcpp::shutdown();
  return 0;
}
 