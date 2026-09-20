// Extended Kalman filter, predict step only for now. SYSTEM UNDER TEST.
// Must never subscribe to Gazebo ground truth.
//
// State [x, y, theta, b]: pose in the odom_ekf frame and the gyro bias, rad/s.
// Inputs: rear wheel speed from /joint_states, yaw rate from /imu.
// Until the LiDAR update exists, the bias estimate stays at its initial 0 and
// the mean is identical to dead_reckoning_node. What this node adds is P, the
// filter's claim about its own error, propagated with the Jacobians F and G
// verified in corridor_experiments/scripts/ekf_predict_consistency.py.
//
// Publishes nav_msgs/Odometry on /odom_ekf with the x, y, yaw block of P in
// pose.covariance, and the full 4x4 P on /ekf/covariance for debugging. The
// cross terms with the bias do not fit in an Odometry message, and a sign
// error in the bias column changes no variance, only those cross terms.
// Topic only, no TF broadcast, for the same reason as dead_reckoning_node.
 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <vector>
 
class Ekf : public rclcpp::Node
{
public:
  Ekf() : Node("ekf")
  {
    // Believed values. Same wheel radius error as dead_reckoning_node, so the
    // two nodes integrate identical inputs identically.
    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.505);
    left_joint_   = this->declare_parameter<std::string>("left_joint",  "rear_left_wheel_joint");
    right_joint_  = this->declare_parameter<std::string>("right_joint", "rear_right_wheel_joint");
 
    // Gyro white noise the filter believes in, rad/s per sample at 100 Hz.
    // Matches the sensor definition.
    sigma_w_ = this->declare_parameter<double>("sigma_w", 1.45e-3);
    // Prior on the gyro bias, rad/s. 8.7266e-4 is 0.05 deg/s.
    const double sigma_b0 = this->declare_parameter<double>("sigma_b0", 8.7266e-4);
 
    // Initial pose known exactly, bias unknown: P = diag(0, 0, 0, sigma_b0^2)
    P_.setZero();
    P_(3, 3) = sigma_b0 * sigma_b0;
 
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),
      std::bind(&Ekf::onImu, this, std::placeholders::_1));
 
    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&Ekf::onJointState, this, std::placeholders::_1));
 
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_ekf", 10);
    cov_pub_  = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ekf/covariance", 10);
 
    RCLCPP_INFO(this->get_logger(),
      "ekf up, wheel_radius %.4f, sigma_w %.3e, sigma_b0 %.4e",
      wheel_radius_, sigma_w_, sigma_b0);
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
    // Mean of the pair: through a turn the differential term cancels, so
    // distance comes from the wheels and heading only from the gyro.
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
 
    const double wz = msg->angular_velocity.z;   // raw gyro reading, bias included
 
    // Predict step. F and G are evaluated at the state going into the step,
    // as in ekf_predict_consistency.py, so they are built before x_, y_ and
    // theta_ are updated.
    const double w = wz - b_;                      // bias corrected yaw rate
    const double th_mid = theta_ + 0.5 * w * dt;   // midpoint heading
    const double s = std::sin(th_mid);
    const double c = std::cos(th_mid);
 
    // F: how a small error in each state entering the step moves each state
    // leaving it. Rows and columns in the order x, y, theta, b.
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = -v_ * s * dt;
    F(0, 3) =  0.5 * v_ * s * dt * dt;
    F(1, 2) =  v_ * c * dt;
    F(1, 3) = -0.5 * v_ * c * dt * dt;
    F(2, 3) = -dt;
 
    // G: how this step's gyro jitter moves each state. The b entry is zero:
    // the bias does not depend on what the gyro reads.
    Eigen::Vector4d G;
    G << -0.5 * v_ * s * dt * dt,
          0.5 * v_ * c * dt * dt,
          dt,
          0.0;
 
    P_ = F * P_ * F.transpose() + sigma_w_ * sigma_w_ * G * G.transpose();
    // Rounding makes P drift very slightly asymmetric over thousands of steps.
    // Forcing symmetry keeps it a valid covariance and keeps the debug output,
    // which assumes symmetry, honest.
        // Copy the transpose first. Writing P_ while reading P_.transpose() in the
    // same expression is an Eigen aliasing error: later elements would read
    // values that were already overwritten.
    const Eigen::Matrix4d Pt = P_.transpose();
    P_ = 0.5 * (P_ + Pt);
    
    // Mean. Identical to dead_reckoning_node while b_ is 0.
    x_ += v_ * c * dt;
    y_ += v_ * s * dt;
    theta_ += w * dt;
    // b_ is unchanged by the predict step. Only the LiDAR update will move it.
 
    nav_msgs::msg::Odometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = "odom_ekf";
    out.child_frame_id = "base_link";
    out.pose.pose.position.x = x_;
    out.pose.pose.position.y = y_;
    out.pose.pose.position.z = 0.0;
    out.pose.pose.orientation.z = std::sin(0.5 * theta_);
    out.pose.pose.orientation.w = std::cos(0.5 * theta_);
 
    // pose.covariance is 6x6 row major over (x, y, z, roll, pitch, yaw).
    // State indices 0, 1, 2 (x, y, theta) map to rows and columns 0, 1, 5.
    // z, roll and pitch stay zero: the filter does not estimate them.
    const int idx[3] = {0, 1, 5};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        out.pose.covariance[idx[i] * 6 + idx[j]] = P_(i, j);
      }
    }
 
    out.twist.twist.linear.x = v_;
    out.twist.twist.angular.z = wz - b_;   // bias corrected yaw rate
    odom_pub_->publish(out);
 
    // Full P, 16 values. Eigen stores column major, but P is symmetric, so
    // row major and column major read identically.
    std_msgs::msg::Float64MultiArray pm;
    pm.data.assign(P_.data(), P_.data() + 16);
    cov_pub_->publish(pm);
  }
 
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cov_pub_;
 
  double wheel_radius_;
  double sigma_w_;
  std::string left_joint_, right_joint_;
  double x_ = 0.0, y_ = 0.0, theta_ = 0.0, b_ = 0.0, v_ = 0.0;
  Eigen::Matrix4d P_;
  bool have_speed_ = false, have_prev_ = false;
  rclcpp::Time prev_stamp_;
};
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ekf>());
  rclcpp::shutdown();
  return 0;
}