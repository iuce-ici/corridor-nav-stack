# One EKF measurement update from a valid corridor geometry scan.
# Debug and analysis only.

std_msgs/Header header              # stamp of the LiDAR scan used

float64[2] innovation               # z minus h(state): offset (m), heading (rad)
float64[4] innovation_covariance    # S = H P H^T + R, row major
float64[4] state_after              # x, y, theta, b after the update
float64 stamp_gap                   # filter time minus scan stamp (s): the cost of updating on arrival