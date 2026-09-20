# EKF state and covariance after a predict step. Debug and analysis only.
# Published on every IMU step, so one message per 10 ms of sim time.

std_msgs/Header header

float64[4] state          # x (m), y (m), theta (rad), b (rad/s), corridor aligned frame
float64[16] covariance    # P, row major, same state order