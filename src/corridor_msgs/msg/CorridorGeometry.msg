# Lateral offset and heading of the vehicle relative to the corridor
# centreline, derived from a single LiDAR scan.
#
# Frame: base_link. Offset is signed, positive to the left (+y).
# Heading is signed, positive when the vehicle points left of the
# corridor axis.

std_msgs/Header header

float64 lateral_offset      # metres, signed
float64 heading_error       # radians, signed
float64 corridor_width      # metres, distance between the two fitted walls

uint32 left_points          # points used in the left wall fit
uint32 right_points         # points used in the right wall fit

float64 left_residual       # RMS perpendicular distance to the left fit, metres
float64 right_residual      # RMS perpendicular distance to the right fit, metres

bool valid                  # false if either side had too few points
