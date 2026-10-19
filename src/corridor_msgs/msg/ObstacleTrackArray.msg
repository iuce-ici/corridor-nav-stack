# All confirmed tracks at one scan. Nothing is published when the EKF pose
# for that scan is missing or stale, which is not the same as no obstacles.

std_msgs/Header header       # frame base_link, stamp of the scan

ObstacleTrack[] tracks