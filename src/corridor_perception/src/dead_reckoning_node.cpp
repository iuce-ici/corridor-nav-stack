// Dead reckoned odometry. SYSTEM UNDER TEST.
// Must never subscribe to Gazebo ground truth. It is allowed, and required, to drift.
// Distance from rear wheel rotation, heading from IMU yaw rate integration.
// The plugin's own /odom is not used: it is computed from commanded motion and
// stays close to truth, which would silently destroy the drift curve.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <string>
#include <vector>

class DeadReckoning : public rclcpp::Node
{
public:
  DeadReckoning() : Node("dead_reckoning")
  {
    // 0.505 against a true 0.5: one percent high, so odometry over reports
    // distance. That is the direction wheel slip acts. A radius error and a
    // constant slip ratio are the same scale error on distance.
    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.505);
    left_joint_   = this->declare_parameter<std::string>("left_joint",  "rear_left_wheel_joint");
    right_joint_  = this->declare_parameter<std::string>("right_joint", "rear_right_wheel_joint");

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),
      std::bind(&DeadReckoning::onImu, this, std::placeholders::_1));

    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&DeadReckoning::onJointState, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_dr", 10);

    RCLCPP_INFO(this->get_logger(), "dead_reckoning up, wheel_radius %.4f", wheel_radius_);
  }

private:
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    double wl = 0.0, wr = 0.0;
    bool got_l = false, got_r = false;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i >= msg->velocity.size()) break;
      if (msg->name[i] == left_joint_)  { wl = msg->velocity[i]; got_l = true; }
      if (msg->name[i] == right_joint_) { wr = msg->velocity[i]; got_r = true; }
    }
    if (!got_l || !got_r) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "rear wheel joints not found in /joint_states");
      return;
    }
    // Mean of the pair. Through a turn the outer wheel runs faster and the inner
    // slower; averaging cancels the differential term, so distance comes from the
    // wheels and heading comes only from the gyro. No accidental second heading.
    v_ = 0.5 * (wl + wr) * wheel_radius_;
    have_speed_ = true;
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    if (!have_prev_) { prev_stamp_ = stamp; have_prev_ = true; return; }

    const double dt = (stamp - prev_stamp_).seconds();
    prev_stamp_ = stamp;
    if (dt <= 0.0 || dt > 0.5) return;   // guard against a reset or a stall
    if (!have_speed_) return;

    const double wz = msg->angular_velocity.z;   // no bias compensation, deliberately

    // Midpoint heading over the interval. Second order in dt, and free.
    const double th_mid = theta_ + 0.5 * wz * dt;
    x_ += v_ * std::cos(th_mid) * dt;
    y_ += v_ * std::sin(th_mid) * dt;
    theta_ += wz * dt;

    nav_msgs::msg::Odometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = "odom_dr";
    out.child_frame_id = "base_link";
    out.pose.pose.position.x = x_;
    out.pose.pose.position.y = y_;
    out.pose.pose.position.z = 0.0;
    out.pose.pose.orientation.z = std::sin(0.5 * theta_);
    out.pose.pose.orientation.w = std::cos(0.5 * theta_);
    out.twist.twist.linear.x = v_;
    out.twist.twist.angular.z = wz;
    odom_pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  double wheel_radius_;
  std::string left_joint_, right_joint_;
  double x_ = 0.0, y_ = 0.0, theta_ = 0.0, v_ = 0.0;
  bool have_speed_ = false, have_prev_ = false;
  rclcpp::Time prev_stamp_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeadReckoning>());
  rclcpp::shutdown();
  return 0;
}