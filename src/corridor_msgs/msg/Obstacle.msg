# One obstacle cluster from a single LiDAR scan, in base_link.
# Positions are per scan measurements, not tracked across scans.

float64 x          # centre along the vehicle axis, metres
float64 y          # centre lateral, positive left
float64 length     # extent along x
float64 width      # extent along y
float64 min_x      # nearest face: the distance the planner acts on
uint32 n_points    # returns in the cluster; a confidence proxy at range