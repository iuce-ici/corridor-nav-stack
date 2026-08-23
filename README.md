# Corridor Navigation Stack

GPS denied navigation and control for an Ackermann vehicle in a confined
rectangular corridor. ROS 2 Jazzy, Gazebo Harmonic, C++ nodes.

Localisation by EKF fusing wheel odometry, IMU yaw rate, and a LiDAR derived
lateral offset and heading relative to the corridor centreline. No prior map,
no GNSS, no SLAM.

Three lateral controllers (pure pursuit, Stanley, LQR on a bicycle model state
space) are compared on a single metric harness: cross track error RMS and max,
heading error, control effort, compute time, and degradation under injected
sensor noise and disturbance.

Status: Phase 0, in progress. The numbers in this README are filled in as each
phase closes, and are absent until then.

## Milestones

| Phase | Content | Exit criterion | Status |
|---|---|---|---|
| 0 | Tooling floor, off the shelf robot | C++ node written by hand, rosbag recorded and replayed, repository exists | in progress |
| 1 | Vehicle and corridor build, corridor geometry extraction, dead reckoned odometry | Offset and heading against Gazebo ground truth with error statistics and timing, documented odometry drift curve | not started |
| 2 | EKF localisation | Pose RMSE against ground truth, with and without the LiDAR update, written account of Q and R selection and of behaviour under mistuning | not started |
| 3 | Pure pursuit and Stanley, metric harness | Both closed loop on the EKF estimate, full metric table, lookahead swept | not started |
| 4 | LQR on own bicycle model state space | Three way comparison table including noise and disturbance cases, written verdict | not started |
| 5 | Ship | README with all numbers, architecture diagram, Docker, CI, demo video, one command clone and run | not started |

Target for v1.0: 15 November 2026.

## Build

    colcon build
    source install/setup.bash
