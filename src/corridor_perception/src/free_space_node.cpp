// Free space from evidence. SYSTEM UNDER TEST.
// Must never subscribe to Gazebo ground truth.
//
// A 3D voxel memory of what the LiDAR has actually shown, in the EKF's frame
// (odom_ekf, corridor aligned), rolling with the vehicle. Phase 3d-3.
//
// Evidence, per scan, for every return:
//   free      every voxel the beam passed through, ending `pass_margin`
//             before the return
//   floor     a return below `floor_z` verifies the floor of its column
//   occupied  any other return marks its voxel occupied, permanently
// A beam with no return is ignored: no return is not evidence of free space
// (on a real sensor it can be a dark surface).
//
// A column is VERIFIED when its floor is verified (a floor return, or a beam
// through its lowest voxel, or it lies in the footprint or the granted zone)
// and no voxel up to `z_max` is occupied. Unverified voxels above are allowed:
// no floating objects is assumed, so anything in a column stands on its floor.
// An overhead object no beam has hit is taken as absent; that is a stated
// assumption, not a measurement.
//
// A column is DRIVABLE when it and its 8 neighbours are verified. Evidence in
// part of a voxel does not verify all of it: a box edge can sit inside a voxel
// whose floor the LiDAR saw beside the box, with the box's side seen edge on
// and never hit. One voxel of erosion removes that, for any obstacle at least
// one voxel thick or hit on its faces. A neighbour outside the lateral window
// is unseen and counts as not verified. An eroded column reads 50.
//
// Footprint: the ground the vehicle stands on (base_link x from `rear` to
// `bumper`, |y| up to `body_half_width`) is free by definition and always
// published drivable.
//
// Proximity zone: a half ellipse ahead of the vehicle, in the vehicle frame,
// centred on the middle of the front bumper, semi-axes `zone` forward and
// `zone_lateral` to each side. Its floor is taken as verified by the vehicle's
// other safety sensors, but only while the LiDAR has no occupied voxel (any
// height) whose centre lies inside it. It lets the vehicle start: the floor
// just ahead of the bumper can never be seen by this sensor from where the
// vehicle stands. It looks only ahead and only at the vehicle's immediate
// vicinity, so it does not depend on the tunnel's width.
//
// Output, /drivable (nav_msgs/OccupancyGrid, base_link, stamp of the scan),
// laid out exactly as /cost_grid: 0 drivable, 50 unknown, 100 occupied.
// Nothing is published when no EKF pose lies within `pose_timeout` of the scan.
//
// Test support, /free_space/pose (geometry_msgs/PoseStamped, stamp of the scan),
// so a checker can replay the scan bit for bit. NOT a pose message in the usual
// sense: position = (x, y, theta) of the EKF pose used, theta in radians, and
// orientation.x = 1 when the proximity zone was granted, else 0, and
// orientation.y = how many scans the node has inserted so far, this one
// included (1 for the first), so a checker can tell whether its recording
// holds every scan the map was built from. And
// /free_space/mount (std_msgs/Float64MultiArray): the sensor to base_link
// rotation (row major, 9 values) and translation (3) this node used, as TF
// delivered them. Nothing in the pipeline reads either.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr uint8_t FREE = 1;
constexpr uint8_t OCC = 2;
constexpr uint8_t FLOOR = 4;   // only on the lowest voxel of a column

struct Pose2
{
  double t = 0.0, x = 0.0, y = 0.0, theta = 0.0;
};
}  // namespace

class FreeSpaceNode : public rclcpp::Node
{
public:
  FreeSpaceNode() : Node("free_space")
  {
    vox_ = declare_parameter("voxel", 0.25);
    z_max_ = declare_parameter("z_max", 4.5);
    floor_z_ = declare_parameter("floor_z", 0.15);
    pass_margin_ = declare_parameter("pass_margin", 0.25);
    half_width_ = declare_parameter("map_half_width", 3.75);   // a whole number of voxels each side
    behind_ = declare_parameter("map_behind", 20.0);
    ahead_ = declare_parameter("map_ahead", 40.0);
    zone_ = declare_parameter("zone", 5.0);                 // half ellipse, forward semi-axis
    zone_lat_ = declare_parameter("zone_lateral", 1.5);     // half ellipse, lateral semi-axis
    body_hw_ = declare_parameter("body_half_width", 0.9);
    rear_ = declare_parameter("rear", -1.0);       // rear face, base_link x
    bumper_ = declare_parameter("bumper", 4.0);    // front face, base_link x
    pose_timeout_ = declare_parameter("pose_timeout", 0.1);
    // Output grid, identical to /cost_grid.
    res_ = declare_parameter("resolution", 0.25);
    x_max_ = declare_parameter("x_max", 30.0);
    y_half_ = declare_parameter("y_half", 6.0);

    if (std::abs(half_width_ / vox_ - std::round(half_width_ / vox_)) > 1e-9) {
      throw std::runtime_error("map_half_width must be a whole number of voxels");
    }
    nz_ = static_cast<int>(std::lround(z_max_ / vox_));
    ny_ = static_cast<int>(std::lround(2.0 * half_width_ / vox_));
    nslot_ = static_cast<int>(std::lround((behind_ + ahead_) / vox_));
    vox_flags_.assign(static_cast<size_t>(nslot_) * ny_ * nz_, 0);
    slot_tag_.assign(nslot_, std::numeric_limits<int64_t>::min());
    gx_ = static_cast<int>(std::lround(x_max_ / res_)) + 1;
    gy_ = static_cast<int>(std::lround(2.0 * y_half_ / res_)) + 1;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/drivable", rclcpp::SensorDataQoS());
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/free_space/pose", 10);
    mount_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/free_space/mount", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom_ekf", 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr m) {
        Pose2 p;
        p.t = rclcpp::Time(m->header.stamp).seconds();
        p.x = m->pose.pose.position.x;
        p.y = m->pose.pose.position.y;
        p.theta = 2.0 * std::atan2(m->pose.pose.orientation.z, m->pose.pose.orientation.w);
        poses_.push_back(p);
        while (poses_.size() > 400) { poses_.pop_front(); }
      });
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/scan/points", rclcpp::SensorDataQoS(),
      std::bind(&FreeSpaceNode::on_cloud, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "free_space up: voxel %.2f m to %.1f m high, map %.0f m behind to %.0f m ahead, "
      "+-%.2f m, floor below %.2f m, pass margin %.2f m, proximity zone half ellipse "
      "%.1f m ahead by %.1f m each side, footprint free, one voxel erosion, "
      "missing returns ignored",
      vox_, z_max_, behind_, ahead_, half_width_, floor_z_, pass_margin_, zone_, zone_lat_);
  }

private:
  bool pose_at(double t, Pose2 & out) const
  {
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

  // Global voxel index along x; may be negative.
  int64_t gxi(double x) const { return static_cast<int64_t>(std::floor(x / vox_)); }

  int slot_of(int64_t g) const
  {
    const int64_t m = g % nslot_;
    return static_cast<int>(m < 0 ? m + nslot_ : m);
  }

  // Index of a voxel for writing: claims the slot for g, clearing what a
  // previous x left there. Returns -1 outside the lateral or vertical window.
  long wr(int64_t g, double y, double z)
  {
    if (g < win_lo_ || g > win_hi_) { return -1; }   // never evict the live window
    const int iy = static_cast<int>(std::floor((y + half_width_) / vox_));
    const int iz = static_cast<int>(std::floor(z / vox_));
    if (iy < 0 || iy >= ny_ || iz < 0 || iz >= nz_) { return -1; }
    const int s = slot_of(g);
    if (slot_tag_[s] != g) {
      std::fill_n(vox_flags_.begin() + static_cast<long>(s) * ny_ * nz_,
                  static_cast<long>(ny_) * nz_, 0);
      slot_tag_[s] = g;
    }
    return (static_cast<long>(s) * ny_ + iy) * nz_ + iz;
  }

  // State of one voxel column before erosion: 0 verified, 50 unknown,
  // 100 occupied or outside the lateral window. Footprint and zone are judged
  // at the column's centre, in base_link of the pose (px, py, c, s).
  int8_t raw_column(int64_t g, int iy, double px, double py, double c, double s,
                    bool zone_ok) const
  {
    if (iy < 0 || iy >= ny_) { return 100; }       // outside: unseen
    const double dx = (static_cast<double>(g) + 0.5) * vox_ - px;
    const double dy = -half_width_ + (iy + 0.5) * vox_ - py;
    const double xb = c * dx + s * dy;
    const double yb = -s * dx + c * dy;
    if (in_footprint(xb, yb)) { return 0; }          // free by definition
    bool floor_ok = zone_ok && in_zone(xb, yb);
    const int sl = slot_of(g);
    if (slot_tag_[sl] == g) {
      const long base = (static_cast<long>(sl) * ny_ + iy) * nz_;
      for (int iz = 0; iz < nz_; ++iz) {
        if (vox_flags_[base + iz] & OCC) { return 100; }
      }
      floor_ok = floor_ok || (vox_flags_[base] & (FREE | FLOOR));
    }
    return floor_ok ? 0 : 50;
  }

  // Published state of the column holding world (x, y): 0 drivable, 50 not
  // verified, 100 occupied or outside.
  int8_t column(double x, double y, double px, double py, double c, double s,
                bool zone_ok) const
  {
    const int64_t g = gxi(x);
    const int iy = static_cast<int>(std::floor((y + half_width_) / vox_));
    const int8_t own = raw_column(g, iy, px, py, c, s, zone_ok);
    if (own != 0) { return own; }
    for (int di = -1; di <= 1; ++di) {
      for (int dj = -1; dj <= 1; ++dj) {
        if ((di != 0 || dj != 0) && raw_column(g + di, iy + dj, px, py, c, s, zone_ok) != 0) {
          return 50;
        }
      }
    }
    return 0;
  }

  // Base_link point inside the proximity half ellipse (boundary included).
  bool in_zone(double xb, double yb) const
  {
    if (xb < bumper_) { return false; }
    const double u = (xb - bumper_) / zone_;
    const double v = yb / zone_lat_;
    return u * u + v * v <= 1.0;
  }

  bool in_footprint(double xb, double yb) const
  {
    return xb >= rear_ && xb <= bumper_ && std::abs(yb) <= body_hw_;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto t0 = std::chrono::steady_clock::now();
    Pose2 P;
    if (!pose_at(rclcpp::Time(msg->header.stamp).seconds(), P)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "no EKF pose within %.3f s of this scan: publishing nothing", pose_timeout_);
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("base_link", msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "transform unavailable: %s", ex.what());
      return;
    }
    // Sensor to base_link in double precision (tf2::doTransform on a cloud
    // works in float, which can move a beam sample across a voxel boundary).
    const auto & qr = tf.transform.rotation;
    const double qw = qr.w, qx_ = qr.x, qy_ = qr.y, qz_ = qr.z;
    const double R00 = 1 - 2 * (qy_ * qy_ + qz_ * qz_), R01 = 2 * (qx_ * qy_ - qz_ * qw), R02 = 2 * (qx_ * qz_ + qy_ * qw);
    const double R10 = 2 * (qx_ * qy_ + qz_ * qw), R11 = 1 - 2 * (qx_ * qx_ + qz_ * qz_), R12 = 2 * (qy_ * qz_ - qx_ * qw);
    const double R20 = 2 * (qx_ * qz_ - qy_ * qw), R21 = 2 * (qy_ * qz_ + qx_ * qw), R22 = 1 - 2 * (qx_ * qx_ + qy_ * qy_);
    const double c = std::cos(P.theta), s = std::sin(P.theta);
    const auto & tr = tf.transform.translation;      // sensor origin in base_link
    const double ox = P.x + c * tr.x - s * tr.y;
    const double oy = P.y + s * tr.x + c * tr.y;
    const double oz = tr.z;
    const double step = 0.5 * vox_;
    win_lo_ = gxi(P.x - behind_) + 1;
    win_hi_ = gxi(P.x + ahead_) - 1;

    // Pass 1: free voxels and floor; pass 2: occupied. Occupied is sticky:
    // a later pass never clears it.
    std::vector<long> occ;
    occ.reserve(1024);
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const double sxp = *ix, syp = *iy, szp = *iz;
      if (!std::isfinite(sxp) || !std::isfinite(syp) || !std::isfinite(szp)) { continue; }
      const double bx = R00 * sxp + R01 * syp + R02 * szp + tr.x;
      const double by = R10 * sxp + R11 * syp + R12 * szp + tr.y;
      const double bz = R20 * sxp + R21 * syp + R22 * szp + tr.z;
      const double px = P.x + c * bx - s * by;
      const double py = P.y + s * bx + c * by;
      const double pz = bz;
      const double dx = px - ox, dy = py - oy, dz = pz - oz;
      const double r = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (r < 1e-6) { continue; }
      const double ux = dx / r, uy = dy / r, uz = dz / r;
      for (double t = step; t < r - pass_margin_; t += step) {
        const double qx = ox + t * ux, qy = oy + t * uy, qz = oz + t * uz;
        if (qz < 0.0) { break; }
        const long k = wr(gxi(qx), qy, qz);
        if (k >= 0) { vox_flags_[k] |= FREE; }
      }
      if (pz < floor_z_) {
        const long k = wr(gxi(px), py, 0.0);
        if (k >= 0) { vox_flags_[k] |= FLOOR; }
      } else {
        const long k = wr(gxi(px), py, pz);
        if (k >= 0) { occ.push_back(k); }
      }
    }
    for (const long k : occ) { vox_flags_[k] |= OCC; }

    // Proximity zone: granted only while no occupied voxel, at any height,
    // has its centre inside the half ellipse.
    bool zone_ok = true;
    const double ecx = P.x + c * bumper_;                 // ellipse centre, world x
    const double reach = std::max(zone_, zone_lat_);
    for (int64_t g = gxi(ecx - reach); g <= gxi(ecx + reach) && zone_ok; ++g) {
      const int sl = slot_of(g);
      if (slot_tag_[sl] != g) { continue; }
      const double dx = (static_cast<double>(g) + 0.5) * vox_ - P.x;
      for (int j = 0; j < ny_ && zone_ok; ++j) {
        const double dy = -half_width_ + (j + 0.5) * vox_ - P.y;
        const double xb = c * dx + s * dy;
        const double yb = -s * dx + c * dy;
        if (!in_zone(xb, yb)) { continue; }
        const long base = (static_cast<long>(sl) * ny_ + j) * nz_;
        for (int k = 0; k < nz_; ++k) {
          if (vox_flags_[base + k] & OCC) { zone_ok = false; break; }
        }
      }
    }
    ++scans_;
    zone_granted_ += zone_ok ? 1 : 0;

    // Output grid in base_link, cell centres sampled from the voxel columns.
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = msg->header.stamp;
    grid.header.frame_id = "base_link";
    grid.info.resolution = static_cast<float>(res_);
    grid.info.width = gx_;
    grid.info.height = gy_;
    grid.info.origin.position.x = -0.5 * res_;
    grid.info.origin.position.y = -y_half_ - 0.5 * res_;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(static_cast<size_t>(gx_) * gy_);
    for (int j = 0; j < gy_; ++j) {
      const double yb = -y_half_ + j * res_;
      for (int i = 0; i < gx_; ++i) {
        const double xb = i * res_;
        const double wx = P.x + c * xb - s * yb;
        const double wy = P.y + s * xb + c * yb;
        int8_t v;
        if (in_footprint(xb, yb)) {
          v = 0;                                          // free by definition
        } else {
          v = column(wx, wy, P.x, P.y, c, s, zone_ok);
        }
        grid.data[static_cast<size_t>(j) * gx_ + i] = v;
      }
    }
    pub_->publish(grid);

    geometry_msgs::msg::PoseStamped used;
    used.header.stamp = msg->header.stamp;
    used.header.frame_id = "odom_ekf";
    used.pose.position.x = P.x;
    used.pose.position.y = P.y;
    used.pose.position.z = P.theta;
    used.pose.orientation.x = zone_ok ? 1.0 : 0.0;
    used.pose.orientation.y = static_cast<double>(scans_);
    pose_pub_->publish(used);
    std_msgs::msg::Float64MultiArray mount;
    mount.data = {R00, R01, R02, R10, R11, R12, R20, R21, R22, tr.x, tr.y, tr.z};
    mount_pub_->publish(mount);

    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    times_.push_back(ms);
    if (times_.size() % 200 == 0) {
      std::vector<double> v = times_;
      std::sort(v.begin(), v.end());
      RCLCPP_INFO(get_logger(),
        "update time over %zu scans: median %.3f ms, p99 %.3f ms, max %.3f ms; "
        "proximity zone granted on %zu of %zu",
        v.size(), v[v.size() / 2], v[std::min(v.size() - 1, v.size() * 99 / 100)], v.back(),
        zone_granted_, scans_);
    }
  }

  double vox_, z_max_, floor_z_, pass_margin_, half_width_, behind_, ahead_;
  double zone_, zone_lat_, body_hw_, rear_, bumper_, pose_timeout_, res_, x_max_, y_half_;
  int nz_ = 0, ny_ = 0, nslot_ = 0, gx_ = 0, gy_ = 0;
  std::vector<uint8_t> vox_flags_;
  std::vector<int64_t> slot_tag_;
  int64_t win_lo_ = 0, win_hi_ = 0;
  std::deque<Pose2> poses_;
  std::vector<double> times_;
  size_t scans_ = 0, zone_granted_ = 0;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mount_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FreeSpaceNode>());
  rclcpp::shutdown();
  return 0;
}
