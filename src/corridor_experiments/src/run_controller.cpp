// Run controller: test apparatus, NOT part of the vehicle.
//
// Drives the vehicle at a constant speed from its spawn position to a target
// x, then stops. Terminates on one of three conditions and reports which:
//   1. target x reached            (normal completion)
//   2. simulation time limit hit   (guard against a run that cannot finish)
//   3. lateral bound exceeded      (guard against grinding along a wall)
//
// This node subscribes to Gazebo ground truth pose. That is deliberate. Its
// job is to make every run identical so that the things under test can be
// compared. A test rig is permitted to know things the system under test
// cannot. No navigation or estimation code may use this topic.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

class RunController : public rclcpp::Node
{
public:
  RunController() : Node("run_controller")
  {
    target_x_    = declare_parameter("target_x", 140.0);
    speed_       = declare_parameter("speed", 2.0);
    steer_rate_  = declare_parameter("steer_rate", 0.0);
    max_abs_y_   = declare_parameter("max_abs_y", 3.0);
    time_limit_  = declare_parameter("time_limit", 300.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/model/vehicle/pose", 10,
      std::bind(&RunController::on_pose, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Run: target_x=%.1f speed=%.2f steer_rate=%.3f",
      target_x_, speed_, steer_rate_);
  }

private:
  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (finished_) { return; }

    const double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    const double x = msg->pose.position.x;
    const double y = msg->pose.position.y;

    if (!started_) {
      start_t_ = t;
      start_x_ = x;
      started_ = true;
    }

    if (x >= target_x_) {
      finish("target reached", x, y, t);
    } else if (std::abs(y) > max_abs_y_) {
      finish("ABORT: lateral bound exceeded", x, y, t);
    } else if (t - start_t_ > time_limit_) {
      finish("ABORT: time limit exceeded", x, y, t);
    } else {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x  = speed_;
      cmd.angular.z = steer_rate_;
      cmd_pub_->publish(cmd);
    }
  }

  void finish(const std::string & reason, double x, double y, double t)
  {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
    finished_ = true;

    RCLCPP_INFO(get_logger(),
      "%s | x=%.3f y=%.4f | travelled %.3f m in %.2f s sim time",
      reason.c_str(), x, y, x - start_x_, t - start_t_);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  double target_x_, speed_, steer_rate_, max_abs_y_, time_limit_;
  double start_t_ = 0.0, start_x_ = 0.0;
  bool started_ = false, finished_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RunController>());
  rclcpp::shutdown();
  return 0;
}