// Extended Kalman filter for the corridor vehicle. SYSTEM UNDER TEST.
// Must never subscribe to Gazebo ground truth.
//
// State [x, y, theta, b] in a corridor aligned frame: x along the corridor axis
// with an arbitrary origin, y lateral from the centreline (positive left),
// theta heading relative to the corridor axis, b the gyro bias in rad/s.
// The frame is fixed by the first valid LiDAR scan: the filter does nothing
// until then, and initialises y and theta from that scan.
//
// Predict, on every IMU message (100 Hz): wheel speed and bias corrected yaw
// rate, midpoint heading, as verified in predict only mode against
// dead_reckoning and against ekf_predict_consistency.py.
// Update, on every valid /corridor_geometry scan (20 Hz), on arrival:
// h(state) = [y, theta], so H is constant. Signs verified: offset by Part 5,
// heading by stationary spawns at +3 and -3 deg.
//
// R covers the extraction's systematic error with range noise off (about
// 0.44 mm and 0.06 mrad), not noise: see analyse_run.py. Because that error
// repeats identically every scan, the claimed lateral uncertainty will end up
// several times smaller than the actual lateral error. Expected, not a bug.
// Longitudinal position x is unobservable in a straight corridor, and the one
// percent wheel radius error is not in P, so P_xx will badly understate the
// actual x error. Also expected.
//
// Publishes /odom_ekf (pose and the x, y, yaw block of P) and /ekf/state
// (state and full P) on every predict step, and /ekf/update on every update.
 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <corridor_msgs/msg/corridor_geometry.hpp>
#include <corridor_msgs/msg/ekf_state.hpp>
#include <corridor_msgs/msg/ekf_update.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <string>
 
class Ekf : public rclcpp::Node
{
public:
  Ekf() : Node("ekf")
  {
    // Believed values. Same wheel radius error as dead_reckoning_node.
    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.505);
    left_joint_   = this->declare_parameter<std::string>("left_joint",  "rear_left_wheel_joint");
    right_joint_  = this->declare_parameter<std::string>("right_joint", "rear_right_wheel_joint");
 
    // Gyro white noise the filter believes in, rad/s per sample at 100 Hz.
    sigma_w_ = this->declare_parameter<double>("sigma_w", 1.45e-3);
    // Prior on the gyro bias, rad/s. 8.7266e-4 is 0.05 deg/s. The estimate
    // itself starts at 0: the filter must discover the bias, not be told it.
    sigma_b0_ = this->declare_parameter<double>("sigma_b0", 8.7266e-4);
    // Measurement standard deviations: lateral offset (m) and heading (rad).
    const double r_y = this->declare_parameter<double>("r_y", 5.0e-4);
    const double r_theta = this->declare_parameter<double>("r_theta", 1.0e-4);
 
    R_ << r_y * r_y, 0.0,
          0.0,       r_theta * r_theta;
    H_.setZero();
    H_(0, 1) = 1.0;   // measured offset  = y
    H_(1, 2) = 1.0;   // measured heading = theta
    P_.setZero();
 
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),
      std::bind(&Ekf::onImu, this, std::placeholders::_1));
 
    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&Ekf::onJointState, this, std::placeholders::_1));
 
    // Best effort matches a publisher of either reliability.
    geo_sub_ = this->create_subscription<corridor_msgs::msg::CorridorGeometry>(
      "/corridor_geometry", rclcpp::SensorDataQoS(),
      std::bind(&Ekf::onGeometry, this, std::placeholders::_1));
 
    odom_pub_   = this->create_publisher<nav_msgs::msg::Odometry>("/odom_ekf", 10);
    state_pub_  = this->create_publisher<corridor_msgs::msg::EkfState>("/ekf/state", 10);
    update_pub_ = this->create_publisher<corridor_msgs::msg::EkfUpdate>("/ekf/update", 10);
 
    RCLCPP_INFO(this->get_logger(),
      "ekf up, wheel_radius %.4f, sigma_w %.3e, sigma_b0 %.4e, r_y %.1e, r_theta %.1e",
      wheel_radius_, sigma_w_, sigma_b0_, r_y, r_theta);
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
    if (!have_speed_ || !initialised_) return;
 
    const double wz = msg->angular_velocity.z;   // raw gyro reading, bias included
 
    // Predict step. F and G are evaluated at the state going into the step.
    const double w = wz - b_;                      // bias corrected yaw rate
    const double th_mid = theta_ + 0.5 * w * dt;   // midpoint heading
    const double s = std::sin(th_mid);
    const double c = std::cos(th_mid);
 
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = -v_ * s * dt;
    F(0, 3) =  0.5 * v_ * s * dt * dt;
    F(1, 2) =  v_ * c * dt;
    F(1, 3) = -0.5 * v_ * c * dt * dt;
    F(2, 3) = -dt;
 
    Eigen::Vector4d G;
    G << -0.5 * v_ * s * dt * dt,
          0.5 * v_ * c * dt * dt,
          dt,
          0.0;
 
    P_ = F * P_ * F.transpose() + sigma_w_ * sigma_w_ * G * G.transpose();
    symmetrise();
 
    x_ += v_ * c * dt;
    y_ += v_ * s * dt;
    theta_ += w * dt;
 
    nav_msgs::msg::Odometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = "odom_ekf";   // corridor aligned since the update step
    out.child_frame_id = "base_link";
    out.pose.pose.position.x = x_;
    out.pose.pose.position.y = y_;
    out.pose.pose.position.z = 0.0;
    out.pose.pose.orientation.z = std::sin(0.5 * theta_);
    out.pose.pose.orientation.w = std::cos(0.5 * theta_);
    // pose.covariance is 6x6 row major over (x, y, z, roll, pitch, yaw).
    const int idx[3] = {0, 1, 5};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        out.pose.covariance[idx[i] * 6 + idx[j]] = P_(i, j);
      }
    }
    out.twist.twist.linear.x = v_;
    out.twist.twist.angular.z = w;
    odom_pub_->publish(out);
 
    corridor_msgs::msg::EkfState st;
    st.header = out.header;
    st.state = {x_, y_, theta_, b_};
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        st.covariance[i * 4 + j] = P_(i, j);
      }
    }
    state_pub_->publish(st);
  }
 
  void onGeometry(const corridor_msgs::msg::CorridorGeometry::SharedPtr msg)
  {
    if (!msg->valid) return;   // predict only until the next valid scan
 
    const Eigen::Vector2d z(msg->lateral_offset, msg->heading_error);
 
    if (!initialised_) {
      // Option A: the first valid scan defines the corridor aligned frame.
      x_ = 0.0;
      y_ = z(0);
      theta_ = z(1);
      b_ = 0.0;
      P_.setZero();
      P_(1, 1) = R_(0, 0);
      P_(2, 2) = R_(1, 1);
      P_(3, 3) = sigma_b0_ * sigma_b0_;
      initialised_ = true;
      RCLCPP_INFO(this->get_logger(),
        "ekf initialised from first valid scan: y %.4f m, theta %.6f rad", y_, theta_);
      return;
    }
 
    Eigen::Vector4d state(x_, y_, theta_, b_);
    const Eigen::Vector2d innovation = z - H_ * state;
    const Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
    const Eigen::Matrix<double, 4, 2> K = P_ * H_.transpose() * S.inverse();
 
    state += K * innovation;
    x_ = state(0);
    y_ = state(1);
    theta_ = state(2);
    b_ = state(3);
 
    // Joseph form: keeps P symmetric and positive definite under rounding.
    const Eigen::Matrix4d IKH = Eigen::Matrix4d::Identity() - K * H_;
    P_ = IKH * P_ * IKH.transpose() + K * R_ * K.transpose();
    symmetrise();
 
    corridor_msgs::msg::EkfUpdate u;
    u.header = msg->header;
    u.innovation = {innovation(0), innovation(1)};
    u.innovation_covariance = {S(0, 0), S(0, 1), S(1, 0), S(1, 1)};
    u.state_after = {x_, y_, theta_, b_};
    // prev_stamp_ only carries sim time once an IMU message has arrived;
    // subtracting times from different clocks throws.
    u.stamp_gap = have_prev_ ?
      (prev_stamp_ - rclcpp::Time(msg->header.stamp)).seconds() : 0.0;
    update_pub_->publish(u);
  }
 
  void symmetrise()
  {
    // Copy the transpose first: writing P_ while reading P_.transpose() in the
    // same expression is an Eigen aliasing error.
    const Eigen::Matrix4d Pt = P_.transpose();
    P_ = 0.5 * (P_ + Pt);
  }
 
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<corridor_msgs::msg::CorridorGeometry>::SharedPtr geo_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<corridor_msgs::msg::EkfState>::SharedPtr state_pub_;
  rclcpp::Publisher<corridor_msgs::msg::EkfUpdate>::SharedPtr update_pub_;
 
  double wheel_radius_;
  double sigma_w_;
  double sigma_b0_;
  std::string left_joint_, right_joint_;
  double x_ = 0.0, y_ = 0.0, theta_ = 0.0, b_ = 0.0, v_ = 0.0;
  Eigen::Matrix4d P_;
  Eigen::Matrix2d R_;
  Eigen::Matrix<double, 2, 4> H_;
  bool have_speed_ = false, have_prev_ = false, initialised_ = false;
  rclcpp::Time prev_stamp_;
};
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ekf>());
  rclcpp::shutdown();
  return 0;
}
 