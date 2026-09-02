// Corridor geometry extraction.
//
// Consumes one LiDAR scan, produces the vehicle's lateral offset from the
// corridor centreline and its heading relative to the corridor axis.
//
// Method:
//   1. transform the cloud into base_link
//   2. keep points with z in [z_min, z_max]: above the floor, below the
//      vehicle clearance envelope
//   3. split into left and right by the sign of y
//   4. least squares fit y = m*x + c to each side
//   5. the centreline is the mean of the two fitted lines; offset is its
//      value at x = 0, heading is its slope
//
// No outlier rejection. Deliberate: with a noiseless sensor in a bare
// corridor there are no outliers. RANSAC goes in only if a least squares
// fit is measured to be insufficient under noise, not before.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <corridor_msgs/msg/corridor_geometry.hpp>

#include <cmath>
#include <vector>

struct LineFit
{
  double slope = 0.0;
  double intercept = 0.0;
  double residual = 0.0;
  uint32_t n = 0;
  bool ok = false;
};

class CorridorGeometryNode : public rclcpp::Node
{
public:
  CorridorGeometryNode() : Node("corridor_geometry")
  {
    z_min_      = declare_parameter("z_min", 0.1);
    z_max_      = declare_parameter("z_max", 5.0);
    min_points_ = declare_parameter("min_points", 20);
    target_frame_ = declare_parameter("target_frame", std::string("base_link"));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_ = create_publisher<corridor_msgs::msg::CorridorGeometry>(
      "/corridor_geometry", 10);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/scan/points", rclcpp::SensorDataQoS(),
      std::bind(&CorridorGeometryNode::on_cloud, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "z band [%.2f, %.2f] m, min %ld points per side, target frame %s",
      z_min_, z_max_, min_points_, target_frame_.c_str());
  }

private:
  // Least squares fit of y = m*x + c. Returns RMS perpendicular distance
  // as the residual, not the vertical distance, since perpendicular is
  // the geometrically meaningful error for a wall.
  static LineFit fit_line(const std::vector<double> & xs,
                          const std::vector<double> & ys,
                          int64_t min_points)
  {
    LineFit f;
    f.n = static_cast<uint32_t>(xs.size());
    if (static_cast<int64_t>(xs.size()) < min_points) { return f; }

    const double n = static_cast<double>(xs.size());
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (size_t i = 0; i < xs.size(); ++i) {
      sx  += xs[i];
      sy  += ys[i];
      sxx += xs[i] * xs[i];
      sxy += xs[i] * ys[i];
    }

    const double denom = n * sxx - sx * sx;
    if (std::abs(denom) < 1e-9) { return f; }   // all points at one x

    f.slope     = (n * sxy - sx * sy) / denom;
    f.intercept = (sy - f.slope * sx) / n;

    double sum_sq = 0.0;
    for (size_t i = 0; i < xs.size(); ++i) {
      const double d = ys[i] - (f.slope * xs[i] + f.intercept);
      sum_sq += d * d;
    }
    // Vertical residual converted to perpendicular.
    f.residual = std::sqrt(sum_sq / n) / std::sqrt(1.0 + f.slope * f.slope);
    f.ok = true;
    return f;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
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

    std::vector<double> lx, ly, rx, ry;

    sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const double x = *it_x, y = *it_y, z = *it_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) { continue; }
      if (z < z_min_ || z > z_max_) { continue; }

      if (y > 0.0) { lx.push_back(x); ly.push_back(y); }
      else         { rx.push_back(x); ry.push_back(y); }
    }

    const LineFit left  = fit_line(lx, ly, min_points_);
    const LineFit right = fit_line(rx, ry, min_points_);

    corridor_msgs::msg::CorridorGeometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = target_frame_;
    out.left_points  = left.n;
    out.right_points = right.n;
    out.left_residual  = left.residual;
    out.right_residual = right.residual;
    out.valid = left.ok && right.ok;

    if (out.valid) {
      // Centreline is the mean of the two wall lines.
      const double m = 0.5 * (left.slope + right.slope);
      const double c = 0.5 * (left.intercept + right.intercept);

      // Offset: signed perpendicular distance from the origin to the
      // centreline. Positive means the centreline lies to the left, so
      // the vehicle is to the right of centre. Negate so that a positive
      // offset means the vehicle itself is left of centre.
      out.lateral_offset = -c / std::sqrt(1.0 + m * m);

      // Heading: the vehicle's x axis relative to the corridor axis.
      // The centreline slope is the corridor direction in vehicle frame,
      // so the vehicle points left of the corridor when the slope is
      // negative.
      out.heading_error = -std::atan(m);

      // Perpendicular separation of the two fitted lines, evaluated with
      // the mean slope.
      out.corridor_width =
        std::abs(left.intercept - right.intercept) / std::sqrt(1.0 + m * m);
    }

    pub_->publish(out);
  }

  rclcpp::Publisher<corridor_msgs::msg::CorridorGeometry>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double z_min_, z_max_;
  int64_t min_points_;
  std::string target_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CorridorGeometryNode>());
  rclcpp::shutdown();
  return 0;
}