#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

class PoseWatcher : public rclcpp::Node
{
public:
  PoseWatcher() : Node("pose_watcher"), count_(0)
  {
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      std::bind(&PoseWatcher::pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "pose_watcher started");
  }

private:
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    count_++;
    if (count_ % 20 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "msg %zu | x=%.2f y=%.2f theta=%.2f v=%.2f",
        count_, msg->x, msg->y, msg->theta, msg->linear_velocity);
    }
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseWatcher>());
  rclcpp::shutdown();
  return 0;
}
