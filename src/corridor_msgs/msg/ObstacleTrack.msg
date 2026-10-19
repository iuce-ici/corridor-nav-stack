# One tracked obstacle, carried across scans by the EKF motion estimate.
# Position is in base_link at the stamp of the array that carries it.

uint32 id          # stable for the life of the track
float64 x          # nearest face, tracked
float64 y          # lateral centre, tracked
float64 length     # from the most recent detection
float64 width
float64 min_x      # the most recent raw measurement, untracked
uint32 n_points

uint32 age         # scans since the track was created
uint32 hits        # scans in which it was matched
uint32 misses      # consecutive scans without a match
bool confirmed     # only confirmed tracks are published