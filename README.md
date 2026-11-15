# Corridor Navigation Stack

GPS denied navigation and control for an Ackermann vehicle in a confined
rectangular corridor. ROS 2 Jazzy, Gazebo Harmonic, C++ nodes.

**Dead reckoning accumulates 2.65 m of position error over 100 m of straight
travel in this vehicle, of which 2.47 m is lateral.** The lateral term grows as
the square of distance while the longitudinal term grows linearly, so the error
that matters in an 8 m wide bore is the one that gets worse fastest. That is the
problem this stack exists to solve.

Localisation by EKF fusing wheel odometry, IMU yaw rate, and a LiDAR derived
lateral offset and heading relative to the corridor centreline. No prior map, no
GNSS, no SLAM.

Obstacles are detected in the same LiDAR scan, tracked across scans on the EKF's
motion estimate, and turned into a cost grid that an A\* planner searches every
scan. A pure pursuit controller then follows the planned path on the EKF
estimate. A comparison of pure pursuit against Stanley and LQR on one metric
harness follows v1.0.

**This is not a wall follower.** The wall measurement corrects an estimate; the
controller tracks a path using that estimate and never sees the LiDAR. In
localisation, only the measurement model, about 100 lines, is corridor
specific. The EKF, process model and planner search are environment
independent.

**Status, 15 November 2026: Phases 0 to 2 complete. Phase 3 in progress:
obstacle scenarios, extraction, tracking, cost grid and A\* planning are
complete and verified; bounded obstacle shadows, path following and the demo
run come next.** Numbers in this README are filled in as each part closes and
are absent until then. Everything reported here is measured in simulation.
Simulation exercises the algorithms; it does not validate them against
hardware, and no claim here should be read as a hardware result.

## Milestones

| Phase | Content | Exit criterion | Status |
|---|---|---|---|
| 0 | Tooling floor, off the shelf robot | C++ node written by hand, rosbag recorded and replayed, repository exists | complete |
| 1 | Vehicle and corridor build, corridor geometry extraction, dead reckoned odometry | Offset and heading against Gazebo ground truth with error statistics and timing, documented odometry drift curve | **complete** |
| 2 | EKF localisation | Pose RMSE against ground truth, with and without the LiDAR update, written account of Q and R selection and of behaviour under mistuning | **complete** |
| 3 | Obstacles, planning and path following | Per part criteria written before the runs that test them (table below) | **in progress** |
| 4 | Controller comparison: pure pursuit, Stanley, LQR on own bicycle model state space | Three way comparison table including noise and disturbance cases, written verdict | after v1.0 |
| 5 | Ship | Docker, CI, one command clone and run | after v1.0 |

The original plan compared the three lateral controllers before handling
obstacles. Obstacle avoidance was moved first, on 19 October 2026, because a
vehicle that cannot pass an obstacle cannot be demonstrated in a tunnel, and the
controller comparison is more meaningful on paths that actually bend.

### Phase 3 parts

| Part | Content | Status |
|---|---|---|
| 3a | Obstacle scenarios in the world generator | complete |
| 3b | Obstacle extraction from the point cloud | complete |
| 3c | Obstacle tracking, and a detector fragmentation fix | complete |
| 3d-1 | Cost grid, and a Gazebo LiDAR defect found and fixed | complete |
| 3d-2 | A\* planner with a side rule, and two message ordering races fixed | complete |
| 3d-3 | Rolling path buffer, validate or extend, bounded obstacle shadows | next |
| 3d-4 | Path smoothing and curvature check | |
| 3e | Pure pursuit on the EKF estimate | |
| 3f | Metrics, demo run | |

**v1.0 is complete when** the vehicle drives itself from A to B through the
8 m by 8 m corridor with static obstacles on both sides, including obstacles on
opposite sides in view at the same time, with a video of the run and the
measured clearance, tracking and localisation numbers behind it.

## Phase 1 results

### Dead reckoned odometry: the drift curve

Distance from rear wheel rotation, heading from IMU yaw rate integration, no
correction of any kind. The node deliberately does not use the Ackermann
plugin's own odometry, which is computed from commanded wheel motion, stays
close to ground truth and would not drift.

Two error sources are modelled. A one percent wheel radius error, where the node
believes 0.505 m against a true 0.500 m, so odometry over reports distance in
the direction wheel slip acts. A residual turn on gyro bias of 0.05 deg/s on the
z axis, with white noise of 1.45e-3 rad/s per axis. Both live in the sensor
definition or as a node parameter, not as noise injected by the node itself.

Straight run at 2.0 m/s, ground truth lateral position constant at zero
throughout, so all reported lateral error belongs to the estimator. Errors are
measured from the first moving sample. The heading at 0 m is what accumulated
while the vehicle stood still before motion.

| Distance travelled | Total error | Longitudinal | Lateral | Heading |
|---|---|---|---|---|
| 0 m | 0.000 m | +0.000 m | +0.000 m | +0.142 deg |
| 10 m | 0.120 m | +0.109 m | +0.051 m | +0.409 deg |
| 20 m | 0.252 m | +0.208 m | +0.143 m | +0.642 deg |
| 30 m | 0.414 m | +0.307 m | +0.278 m | +0.881 deg |
| 40 m | 0.611 m | +0.406 m | +0.457 m | +1.139 deg |
| 50 m | 0.846 m | +0.503 m | +0.680 m | +1.385 deg |
| 60 m | 1.116 m | +0.600 m | +0.941 m | +1.599 deg |
| 70 m | 1.428 m | +0.695 m | +1.247 m | +1.863 deg |
| 80 m | 1.780 m | +0.789 m | +1.595 m | +2.105 deg |
| 90 m | 2.174 m | +0.882 m | +1.988 m | +2.350 deg |
| 100 m | 2.611 m | +0.972 m | +2.423 m | +2.587 deg |

Reported distance 102.026 m against a true 101.024 m at the end of the run,
which includes about 1 m of braking past the target: the one percent scale
error propagating as expected. Heading is reported at distance, not at the end
of the recording, because the bias keeps integrating while the vehicle stands
still after the run.

**Against closed form.** Longitudinal error was predicted at 1.00 m and
measured at 0.972 m. Lateral error from a constant gyro bias should follow
`b*d^2/(2v)`, predicting 2.18 m, and measured 2.423 m at 100 m. The excess has
the shape of a heading offset present from the start, which adds a term linear
in distance. A fit of lateral = a*d + c*d^2 matches every row to within
2.2 mm. The quadratic coefficient is 2.133e-4 per metre against 2.182e-4
predicted from the bias, 2 percent low, consistent with this run's gyro noise.
The linear coefficient is 0.166 deg: the 0.142 deg of heading measured at the
start of motion, plus 0.025 deg from the half second the acceleration ramp
adds to the time taken to reach any distance.

**Correction, 20 September 2026.** An earlier version of this table was
computed from messages in the order the bag returned them, which is recorder
receive order, not publish order. That moved the starting reference 0.85 m
into the run and shifted every row; the 100 m lateral figure was published as
2.459 m. The analysis now sorts by header stamp.

### Corridor geometry extraction

Lateral offset and heading from a single scan: transform to `base_link`, filter
to a z band, split left from right by the sign of y, and fit a line to each wall
by fit, reject points beyond 0.30 m perpendicular, refit.

Measured at a 1.5 m lateral offset in an 8 m corridor. The run was deliberately
continued past the clean region so the end wall degradation could be
characterised.

| Analysis bucket, `base_link` x | Offset error | Width error | Flag |
|---|---|---|---|
| 10.0 to 114.8, end wall beyond sensor range | 0.44 mm | 0.47 mm | valid |
| 114.8 to 127.9, end wall entering the fits from x = 115.8 | 104 mm | 289 mm | invalid from onset, a few scans just past 115.8 still valid |
| 127.9 to 141.0, end wall dominating the fits | 751 mm | 2216 mm | invalid |

Rows are equal width analysis buckets, x from 10 to 141 in ten steps, with the
first eight identical and merged. The degradation onset is analytic at x = 115.8:
end wall at 150 m, minus the LiDAR mount at 4.2 m, minus the 30 m maximum range.
The last bucket excludes the scans at the final position, where the vehicle
stood closest to the wall. Degraded values are signed means.
**Corrected 20 September 2026:** the rows were previously labelled with the
boundaries 115.8 and 128, which were not the boundaries of these numbers.

Across the clean regime, 105 m of travel, the reported offset is constant to two
decimals, with point counts and fit residuals identical bucket to bucket.

**The limitation, stated in full.** A wall ahead within the sensor's maximum
range is a large planar return spanning the corridor width and lying inside the
z band, so its points enter both wall fits. The rejection pass removes many of
them but is overwhelmed rather than failing gracefully. Onset is when the
sensor, not the vehicle origin, comes within maximum range of the wall:
`wall_x - lidar_x - max_range`, which for this configuration is `base_link` at
x = 115.8, not 120. The right wall degrades faster than the left when the
vehicle is offset to the left, because the further wall's returns are
intercepted first.

No near range filter is applied, deliberately. Rejecting near range returns
would remove exactly the points needed for obstacle response and for turning.
The failure is documented and bounded rather than patched. In a real bore under
construction there is a face at one end, so this is a genuine operating
condition rather than a simulation artefact.

`valid` gates on fit residual, but degraded values are still published. A zero
is a plausible number that a lateral controller could read as perfectly centred,
which is the most dangerous possible wrong answer. Fitting succeeded and
trustworthy are two different conditions and the code distinguishes them.

### Other verified measurements

| Quantity | Value |
|---|---|
| LiDAR rate | 20 Hz simulation time, minimum interval 47 ms, no drops |
| IMU rate | 97.6 Hz against 100 commanded |
| Static drift over 617 s of simulation time | zero to sixteen significant figures |
| Straight run lateral deviation over 131 m | 8.6 micrometres |
| Steering, front left and right at v = 2.0, omega = 0.15 | 0.20472 and 0.19512 rad, implied radii 13.63 m and 14.35 m against 13.33 m commanded |
| Valid LiDAR returns, static, centred | 5611 of 5776; the missing 165 are near axial beams down 135 m of empty corridor |
| LiDAR reconstruction noise | plus or minus 10 mm near field, 30 mm at 23 m, no systematic bias |
| Fit residual, normal operation | 0.013 m |
| Gyro bias, measured over 5865 samples | 0.05084 deg/s against 0.05000 configured |

## Phase 2 results

Dead reckoning drifts because nothing corrects it. The corridor walls are a
measurement that is always available in a tunnel, and they observe exactly the
two quantities that matter for staying in the bore: how far the vehicle is from
the centreline, and which way it points relative to the corridor axis. Phase 2
builds an extended Kalman filter around that measurement and quantifies what it
is worth.

### The filter

State `[x, y, theta, b]`: position along the corridor, lateral offset from the
centreline, heading relative to the corridor axis, and the gyro bias in rad/s.

The bias is a state because the Phase 1 drift curve is dominated by it. Gyro
white noise at the configured density accounts for about 6 cm of lateral
standard deviation over 100 m; the measured drift was 2.4 m, roughly forty
times more. A three state filter would claim 6 cm while being metres wrong, and
would therefore under weight every LiDAR correction. With the bias in the
state, the filter's own covariance grows the way the real error grows, and the
heading residuals let it work the bias out.

**Predict**, at 100 Hz from the wheels and the gyro: the same midpoint heading
integration as the dead reckoning node, with the bias estimate subtracted.

**Update**, at 20 Hz from each valid corridor geometry scan. The frame is fixed
by the first valid scan, so `h(state) = [y, theta]` and `H` is constant. The
covariance update is in Joseph form, which keeps `P` symmetric and positive
definite under rounding in a matrix whose entries span many orders of
magnitude.

The gyro bias estimate starts at zero. Its magnitude and its sign are both
learned from the LiDAR, never configured.

### Measurement noise, measured rather than chosen

`R` is the scan to scan spread of the extracted offset and heading at a
constant true pose, measured on a dedicated run:

| With 3 cm LiDAR range noise | Value |
|---|---|
| Per point range noise | 30 mm |
| Spread of the fitted lateral offset | 0.73 mm |
| Spread of the fitted heading | 0.0546 mrad |
| Lag one correlation of the offset error | -0.04 |
| Fit residual | 0.0253 m, against a 0.05 m validity threshold |
| Valid fraction in the clean regime | 1.000 |

Each wall fit uses about 2070 points, so the line fit averages the per point
noise down by roughly the square root of that, a factor of 45. The measured
reduction is 41. The near zero lag one correlation means the scan to scan
errors are independent, which is what the filter assumes.

The fit residual is the scatter of individual returns about the fitted line. It
is not the noise on the fitted offset and it is not `R`.

### Results, ten seeds

Ten runs of 101 m at 2 m/s, each a different Gazebo seed and so a different
noise realisation. Errors are against ground truth over the motion window.

| | Mean | Standard deviation | Worst run |
|---|---|---|---|
| EKF lateral RMSE | 0.033 mm | 0.012 mm | 0.061 mm |
| EKF lateral max | 0.115 mm | 0.057 mm | 0.243 mm |
| EKF heading RMSE | 0.0447 mrad | 0.0014 mrad | 0.0466 mrad |
| EKF heading max | 0.189 mrad | 0.034 mrad | 0.277 mrad |
| Dead reckoning lateral max | 2556 mm | 126 mm | 2704 mm |
| Gyro bias error | +0.27 percent | 1.78 percent | 3.45 percent (1.86 sigma) |
| Longitudinal error at 101 m | 1.010 m | 0.000 m | 1.011 m |

Two comparisons, because only the pair of them is honest. Against dead
reckoning, the lateral error falls from 2.56 m to about a tenth of a
millimetre: that is what having the walls is worth. Against a single scan,
0.73 mm to 0.033 mm, a factor of 22: that is what the filter itself is worth
over simply trusting each scan, and it comes from combining many scans through
a motion model. Heading improves by only a factor of 1.2, because between
updates it rides on the gyro.

The simulator sets the gyro bias sign from the seed. Five runs came out
positive and five negative, and every one was estimated within 3.45 percent and
1.86 sigma of its own sign.

The worst run is seed 404: heading max 0.277 mrad against a 0.28 mrad limit,
mean NIS 2.165, bias error 2.96 percent. One run in ten brushing a six sigma
bound is about what correct Gaussian behaviour produces over this many samples.

### Consistency

Mean normalised innovation squared is 2.053 across the ten seeds, standard
deviation 0.055, range 1.976 to 2.165. A consistent filter gives the
measurement dimension, here 2. The filter's claimed uncertainty therefore
matches its actual error, which is what makes that claimed uncertainty usable
by anything downstream.

### Behaviour under mistuning

The same stack was run with LiDAR range noise switched off, where the
extraction is accurate to about 2 microradians in heading but `R` was still set
to 0.1 mrad, overstating the measurement noise by roughly fifty times. The
filter then under weights every scan, and the heading estimate wanders on the
gyro between updates: standard deviation 0.042 mrad with peaks near 0.16 mrad,
against a measurement whose own error never exceeded 0.0022 mrad. The
correlation between the measurement error and the filter error was 0.02, which
is how the wander was attributed to the filter rather than to the sensor. Mean
NIS in that configuration is 0.16 rather than 2, which is the signature of an
`R` that is too large.

That configuration is kept for isolating problems, since a noiseless sensor
removes one variable, and is not used for reporting results.

### What the filter cannot do

Longitudinal position is unobservable in a straight corridor. The walls look
the same at 30 m as at 80 m, so nothing measures distance along the bore. The
error stays at 1.010 m over 101 m with zero spread across all ten seeds, which
is exactly the one percent wheel radius error propagating, and the filter's
covariance for `x` keeps growing. That is correct behaviour rather than a
defect, and it is why the covariance on `x` must not be read as a position
accuracy. Closing it needs a longitudinal feature: a cross passage, a known
marker, or wheel odometry calibrated against one.

The claimed lateral uncertainty also excludes the extraction's own systematic
error, which is under 0.5 mm and depends on pose. No sensor on this vehicle can
separate a small constant offset in the measurement from the vehicle genuinely
sitting that far off the centreline.

### How it was verified

Each step had its exit criteria written before the run that tested it.

| Step | Check | Result |
|---|---|---|
| Predict, in Python | `P` against 5000 Monte Carlo runs, three cases | Every standard deviation within 5 percent, every correlation within 0.05 |
| Predict, in C++ | Mean against the dead reckoning node | Heading bit identical over 6000 steps, position within 0.020 mm |
| Predict, in C++ | `P` against a Python replay of the node's own inputs | Agreement to 3.4e-14 |
| Measurement signs | Stationary spawns at +3 and -3 degrees | Published heading matched true yaw within 0.02 mrad, same sign |
| Update | Five criteria on one run, then six across ten seeds | All pass; see the tables above |

The Monte Carlo check was itself checked against three deliberately broken
versions of the filter, each caught only by the case built to see it. A sign
error in the bias column changes no variance at all and shows up only in the
correlations. A sign error on `x` is invisible at zero heading and needs the 60
degree case. A four times too large `Q` is hidden by the bias in both of those
and needs the case with no bias at all.

## Phase 3 results so far

Every part below had its exit criteria written before the runs that tested
them, and every checker was first fed deliberately broken synthetic data to
show it could fail.

### 3a: obstacle scenarios

`generate_corridor.py` takes a scenario name and writes the world plus a JSON
sidecar of the obstacle truth, so every analysis reads placements from one
place. With no argument it reproduces `corridor.sdf` byte for byte, so the
Phase 1 and 2 results stay reproducible.

Obstacles are static boxes, 1.2 by 1.2 by 1.5 m unless a scenario says
otherwise, at least 1 m clear of both walls. Height 1.5 m against a LiDAR at
z = 1.0 means a far obstacle returns a handful of points on one or two layers,
which is the realistic case.

| Scenario | Obstacles (x, y of centre) | Tests |
|---|---|---|
| `single_left`, `single_right` | (50, 0.6), (50, -0.6) | side decision, one obstacle |
| `slalom` | (40, 0.8), (70, -0.8) | fragmentation, side faces |
| `narrow`, `late` | (50, 1.0), (18, 0.0) | tight gap; obstacle too close to steer round |
| `blocked` | three boxes at x 50 wall to wall | no path |
| `centre` | (50, 0.0), 0.6 m wide | an exact tie between left and right |
| `pair` | (40, 2.2), (52, 0.5) | two obstacles in view at once |
| `tandem` | (40, 0.6), (48, 0.6) | second obstacle hidden behind the first |

### 3b: obstacle extraction

Per scan, in `base_link`: keep points in a z band of 0.10 to 4.5 m, drop those
within 0.30 m of either wall using the published corridor geometry rather than
refitting, and connect occupied cells of a 0.25 m grid. A uniform grid rather
than DBSCAN: an obstacle returns 5 points at 30 m and about 40 at 10 m, so
there is nothing for a density method to earn. Nothing is published when the
geometry is missing or invalid, since an empty array would assert the corridor
is clear.

`single_left`, 3 cm range noise: first detection at 29.94 m with 5 points,
against a sensor geometry prediction of 30 m. Detection rate 1.00 at every
range inside 30 m, no false positives, lateral error within 54 mm.

The near face is reported as the minimum over N noisy points, which is biased
low by about sigma times the square root of 2 ln N: 82 mm measured at 217
points, 36 mm at 6. It reports the obstacle nearer than it is, which is the
correct direction for avoidance.

### 3c: tracking, and a fragmentation fix

A two state Kalman filter per track in `base_link`, carried between scans by
the EKF's motion estimate. No velocity state: obstacles are static, so the only
motion to model is the vehicle's own. Mahalanobis gating, greedy association,
confirmation after 3 hits, deletion after 10 misses.

An obstacle offset laterally shows its side face to the sensor at grazing
incidence, which returns isolated vertical columns about 0.7 m behind the near
face rather than a connected surface. The clustering reported one obstacle as
two, and the fragment became a phantom track. A merge step was added whose
threshold is the narrowest gap the vehicle could pass through: two clusters
closer than that are one obstacle by definition. The defect was invisible in
`single_left`, where the side face lies exactly in the sensor plane, and
appeared at once in `slalom`. It was found by running a second scenario, not by
re-reading the code.

| Measure | Before | After |
|---|---|---|
| Split fraction, 5 to 10 m | 0.73 | 0.00 in every range band |
| Phantom tracks | 151 | 0 |
| Track ids per obstacle | several | one |
| Lateral jitter | 13 mm | 5 mm |
| Forced detection outages | | 93 coasting scans, no track lost, worst error 92 mm |

### 3d-1: cost grid

Rebuilt every scan in `base_link`: x 0 to 30 m, y plus or minus 6 m, 0.25 m
cells centred on multiples of the cell size, so the vehicle origin is a cell
centre. Values: 100 lethal, 99 blocked for new paths only, 0 to 98 a lateral
cost quadratic in distance from the fitted centreline, so the cheapest path
through an empty corridor is the centre and the vehicle returns there by
itself.

Radii: tight 1.0 m (0.9 m half width at the outer wheel faces plus 0.1 m of
steered wheel swing at 12 degrees), loose 1.3 m (plus 0.3 m clearance). The
same radii apply to walls and obstacles; the 0.3 m between them is the replan
hysteresis.

Two corrections to obstacles, both measured on the 3c runs rather than assumed:

- **Depth is unobservable.** The reported length stayed at 0.07 to 0.16 m at
  every range against a true 1.2 m. Each obstacle is therefore treated as
  extending from its near face to the end of the grid. Unseen is not free.
- **Width is short.** An edge on corner loses about one azimuth step; an
  ordinary edge lost up to 1.28 steps at 10 to 20 m, mechanism not identified.
  The worst per side loss over every scan was 0.22 m, so each side is padded
  0.5 m.

| Criterion, against ground truth | Result |
|---|---|
| C1: every minimum cost cell within 0.25 m of the true centreline, clear corridor, 1 m offset spawn | PASS, 320 grids, worst 0.000 m |
| C2: wall bands within 0.02 m of truth | PASS, 320 grids |
| C3: the true box lethal, no lethal cell beyond tight + 0.15 m in front or tight + 0.6 m beside | PASS, 201 grids, worst excess 0.000 m |
| C4: build time p99 under 5 ms | PASS, p99 1.43 and 1.99 ms |

### A defect in Gazebo's LiDAR, found by C2

C2 failed in an empty corridor with the vehicle 1 m off centre: a confirmed
obstacle where there was none, 70 track ids in 15 s, always at y = -3.0. At
y = 0 the phantom fell on the opposite wall and was discarded as wall, which is
why 3b and 3c never saw it. Varying the spawn offset turned it into a
falsifiable hypothesis: at 1.5 m off centre the phantom was predicted at
y = -2.5 and measured at -2.506.

With 361 horizontal samples over plus or minus 90 degrees, Gazebo's
`gpu_lidar` returned, in every scan, at azimuth exactly -45 degrees, the ranges
that belong to +45 degrees, on every layer that reaches a wall, and only in that
direction. The returns were found in the raw point cloud, before any node in
this stack, and confirmed against exact ray cast truth for every beam: one
column, about 3 m of error, everything else within the known range noise. It
depends on a beam lying exactly on 45 degrees. With 360 samples (0.5014 degree
step) there are 0 impossible returns and no new bad column. 360 is now locked,
with the reason in the sensor description, and
`analyse_impossible_points.py` is a standing regression check. No public report
of this behaviour was found; it may be specific to this configuration.

### 3d-2: A\* planner and the side decision

A\* over the cost grid, rebuilt every scan, from the vehicle origin to any cell
of the last column: no goal pose and no map. Moves are forward only, one row to
either side per column, so the path has one y per x and the side on which it
passes an obstacle is unambiguous. Cells at 99 and 100 cannot be entered. An
empty path means blocked; nothing published means the walls are not known.

**Side flips came from exact ties.** With an obstacle on the centreline and
plain A\*, the path changed side 144 times in 391 grids. 389 of those grids were
exact ties: the cells hold integers, so a symmetric layout gives two paths of
identical cost, and the side was decided by the order of the priority queue.
3 cm range noise moves the obstacle's near face within a 25 mm band even with
the vehicle standing still, enough to push cells across a threshold and reorder
the queue. The planner itself is deterministic: identical grids gave identical
paths in 85 of 85 cases.

**The side rule**, per confirmed track, nearest first: if only one side of the
obstacle has any open cell, take it without searching; otherwise search both
and go **right unless left is cheaper by more than 3 percent**, following right
hand traffic. A side once chosen is kept unless the other becomes cheaper by
more than 10 percent or it closes, remembered per track id. A sweep of obstacle
positions showed side margins are either under 1 percent or over 6 percent on
this grid, with nothing in between, so 3 percent sits in the empty range and
10 percent allows at most one switch per obstacle. Result: 0 flips on the same
scenario, in three runs.

**Two message ordering races.** Two topics computed from the same scan carry no
ordering guarantee between them. The planner received a grid before that
scan's tracks on about 3 percent of scans, and then planned without the rule.
The cost grid received a scan's tracks before that scan's geometry on about 60
percent, and so built most grids from the previous scan's geometry, once
publishing a grid on a scan whose geometry was invalid. Both now pair messages
by exact stamp and hold the earlier one until its partner arrives.

**Timing budget.** Planner compute time was first held to a fixed 5 ms, set
when the planner ran one search; the side rule can run more, and three runs
exceeded it (5.3 to 6.2 ms). The commit that first closed this part reported a
pass; that was wrong, and the next commit says so. The limit is now derived
from the measured pipeline: the path must describe a world no older than one
scan, so scan to path p99 at most 50 ms, and the planner's share is
(50 ms - U99 - 5 ms reserved for the path follower) / 2, where U99 is the p99
from scan to grid.

| Stage, after the scan | median | p99 |
|---|---|---|
| geometry | 9.96 ms | 15.46 ms |
| obstacles | 9.47 ms | 14.33 ms |
| tracks | 9.80 ms | 14.96 ms |
| cost grid | 10.82 ms | 16.40 ms |
| path | 12.31 ms | 17.73 ms |

These are recorder receive times, so the transport of the point cloud is inside
every number: an upper bound. U99 16.40 ms gives a planner limit of 14.30 ms.

| Criterion | Result |
|---|---|
| P1 validity: legal moves, no cell at 99 or above, reaches the last column | PASS, every run |
| P2 optimality: cost equals an independent exact search under the sides the rule picks, rule re-applied independently | PASS, every run |
| P3 centreline: from 10 m ahead, within 0.25 m of the true centreline | PASS, worst 0.000 m |
| P4 side: expected side on every confirmed grid, no flips | PASS: forced sides, the centred tie (0 flips), two obstacles in view at once |
| P5 no path while the corridor is blocked | PASS, 218 of 218 grids |
| P6 start at the vehicle origin | PASS |
| P7 planner p99 under 14.30 ms; scan to path p99 under 50 ms | PASS, at most 6.2 ms; 17.73 ms |
| P8 silent on invalid geometry | PASS, 46 invalid scans, 0 paths |

**Known limit, to be removed in 3d-3.** Because an obstacle's shadow runs to the
end of the grid, two obstacles visible at once on opposite sides cover the whole
plannable width, and the vehicle would stop before the first. The candidate
fixes are clearing the shadow where LiDAR rays have passed behind the obstacle
to the far wall, an assumed maximum obstacle length, or a side looking short
range sensor that measures length while passing.

## Configuration

Every number below is fixed and any change to one invalidates results measured
under the previous value.

### Corridor

8.0 m wide, 8.0 m high, 150 m long, dimensions to the inner faces. Entrance at
x = 0, centreline exactly y = 0, floor z = 0. Wall thickness 0.2 m with centres
at 4.1 m so the inner face lands on 4.0 m. The sensor only ever sees inner
faces. Generated parametrically by `corridor_sim/worlds/generate_corridor.py`.

### Vehicle

Overall length 5.0 m with 1 m overhang each end, wheelbase 3.0 m, track width
1.5 m, wheel radius 0.5 m, height 4.0 m. Maximum steering 12 degrees, giving a
14.1 m turning radius. `base_link` is the rear axle centre projected to ground.
The drivetrain is modelled as a rate limited velocity source rather than through
`ros2_control`: acceleration capped at 2 m/s squared, steering rate at 1 rad/s,
both kinematic.

### LiDAR

Mounted at x 4.2, y 0, z 1.0 in `base_link`, with a 0.2 m standoff ahead of the
front face so the outermost beams do not graze the bodywork. Pitched 12 degrees
nose down to centre an asymmetric vertical field of view. Sixteen layers,
uniform spacing, sensor frame -7.2 to +34.3 degrees. 180 degrees horizontal, 360
samples at 0.5014 degrees, the front mount being occluded to the rear by its own
vehicle. 360 rather than 361 avoids a beam exactly on 45 degrees, where Gazebo
returns the wrong ranges (Phase 3, 3d-1). Phases 1 and 2 were measured with 361
samples. The affected column is about 14 returns against about 2000 per wall
fit; at a centred spawn it lands on the opposite wall itself, and off centre it
lies well outside the 0.30 m rejection band. Its effect on those results is
expected to be negligible and has not been re-measured. 20 Hz, giving the 50 ms frame period used in service. Maximum range
30.0 m, capped well below corridor length so the extraction sees the same class
of scene throughout. Range noise is a launch argument, `lidar_noise`, standard deviation in metres,
default 0.03. That is the figure for a sixteen layer sensor of this class.
Zero is available for isolating problems: it makes the extraction nearly exact,
which removes one variable when something else is being diagnosed, but it is
not used for reporting results.

Uniform layer spacing is a stated modelling simplification. Real spacing is non
uniform and cannot be expressed in a single Gazebo sensor.

### IMU

100 Hz. Gyro white noise 1.45e-3 rad/s standard deviation per axis, about 0.5
degrees per root hour at this rate. Gyro bias 8.7266e-4 rad/s, 0.05 deg/s, on
the z axis only, with `bias_stddev` zero so it is a constant turn on bias rather
than a random walk. Bias on x and y is omitted because a planar heading does not
use them. Accelerometer noise zero: nothing in this stack integrates
acceleration.

### Simulator seed

A launch argument, `seed`, default 42, passed to Gazebo. This is part of the
configuration, not an implementation detail: the seed sets the sign of the gyro
bias as well as the noise sequence, even when `bias_stddev` is zero, so two runs
on different seeds can drift in opposite directions. Across the Phase 2 sweep of
ten seeds the sign came out positive five times and negative five times. With the seed fixed, the noise
sequence is reproducible to 3.4e-12 rad/s across runs; the residual is physics
solver thread ordering, not the random source.

A fixed seed means every run shares one noise realisation, so a single result is
one draw rather than a statistic. Development and comparison run on seed 42.
Headline figures from Phase 2 onward are taken over a sweep of ten seeds, run by
`sweep_seeds.sh` and summarised by `aggregate_sweep.py`.

### Extraction parameters

Wall z band 0.1 to 5.0 m in `base_link`. The lower bound carries over a 10 cm
minimum obstacle height; the upper is the tunnel vehicle clearance envelope, so
every point used is one that matters operationally. Outlier rejection at 0.30 m
perpendicular, ten times the far field reconstruction noise and two orders of
magnitude below the arc separation. Maximum residual for a `valid` flag 0.05 m,
against 0.013 m in normal operation and 0.16 m when degraded.

### Obstacle pipeline parameters

| Node | Parameter | Value |
|---|---|---|
| `obstacle_extraction` | z band, wall margin, cell, minimum points, merge gap, maximum x | 0.10 to 4.5 m, 0.30 m, 0.25 m, 4, 1.0 m, 30 m |
| `obstacle_tracking` | process noise, measurement noise, gate, confirm, drop | 0.020 m, 0.050 m, 9.21, 3 hits, 10 misses |
| `cost_grid` | resolution, extent, tight and loose radii, lateral pad, hold limit | 0.25 m, x 0 to 30 by y plus or minus 6 m, 1.0 and 1.3 m, 0.5 m, 0.5 s |
| `path_planner` | centre weight, blocked from, side bias, side switch | 1.0, 99, 3 percent, 10 percent |

The 4.5 m upper z bound is the vehicle roof at 4.0 m plus 0.5 m. It is a design
decision not yet verified: no scenario has an overhead obstacle that could tell
4.5 from 5.0.

## Build and run

```
colcon build
source install/setup.bash
```

A single launch brings up the world, the vehicle, transforms, the bridge, the
localisation, perception and planning nodes, and the run controller. The nodes
start from the spawn completion event rather than from a launch timer, so
ordering holds regardless of how long Gazebo takes to load.

```
# terminal 1: start recording before the launch so nothing is missed
cd bags
ros2 bag record -o NAME /ekf/state /ekf/update /corridor_geometry \
  /odom_ekf /odom_dr /imu /model/vehicle/pose /clock

# terminal 2: one command runs everything
ros2 launch tunnel_vehicle simulation.launch.py \
  y:=0.0 target_x:=110.0 speed:=2.0 lidar_noise:=0.03 seed:=42
```

The obstacle worlds are generated and committed (`corridor_<scenario>.sdf`);
select one by name. After editing a scenario, rerun `generate_corridor.py
<scenario>` in `src/corridor_sim/worlds` and rebuild `corridor_sim`.

```
ros2 launch tunnel_vehicle simulation.launch.py world:=corridor_centre target_x:=40.0

ros2 bag record -s mcap -o bags/NAME /scan/points /obstacles /cost_grid \
  /planned_path /model/vehicle/pose /obstacle_tracks /corridor_geometry

python3 src/corridor_experiments/scripts/analyse_planner.py bags/NAME \
  --obstacles 50,0 --sides A
python3 src/corridor_experiments/scripts/analyse_latency.py bags/NAME
```

Arguments: `world` selects the generated world, `y` sets the lateral spawn
offset, `yaw` the spawn heading in radians, `target_x` the absolute x at which
the run terminates, `speed` the commanded
forward speed, `lidar_noise` the LiDAR range noise standard deviation in metres,
and `seed` the Gazebo random seed. The vehicle spawns at x = 10, so
`target_x:=110.0` gives 100 m of travel.

The seed sweep that produces the Phase 2 headline figures runs ten seeds one at a
time, recording one bag each:

```
./src/corridor_experiments/scripts/sweep_seeds.sh
python3 src/corridor_experiments/scripts/aggregate_sweep.py bags/ekf_sweep_s*
```

Nothing else may be running while it does: a node left over from an earlier
launch publishes on the same topics, and the bag then holds two nodes' messages
interleaved. `ros2 node list` must be empty before a run. The sweep clears stale
processes itself and the aggregator refuses a bag with duplicate timestamps.

The run controller terminates on one of three conditions and reports which:
target x reached, simulation time limit exceeded, or lateral bound exceeded. The
last two are guards. A run ending that way is a failed run and must not be
averaged into results. Read the termination line before analysing the bag.

Gazebo is closed manually so a failed run can be inspected.

**Measurement runs must terminate before x = 115.8**, the onset of end wall
degradation. `target_x:=110.0` stops commanding motion at x = 110.0; the vehicle coasts
about 1 m further while braking, which leaves about 4.8 m of margin. Feeding the degraded
region into an estimator would make the resulting error a mixture of filter
behaviour and known sensor degradation.

## Package layout

```
src/
  cpp_intro/            Phase 0 exercise
  corridor_sim/         world
    worlds/generate_corridor.py    parametric generator
    worlds/corridor.sdf            generated, committed
  tunnel_vehicle/       vehicle description, sensors, launch, bridge
  corridor_msgs/        custom interfaces
  corridor_perception/  system under test
    src/corridor_geometry_node.cpp
    src/dead_reckoning_node.cpp
    src/ekf_node.cpp
    src/obstacle_extraction_node.cpp
    src/obstacle_tracking_node.cpp
    src/cost_grid_node.cpp
    src/path_planner_node.cpp
  corridor_experiments/ test apparatus
    src/run_controller.cpp
    scripts/ekf_predict_consistency.py   predict step against Monte Carlo
    scripts/analyse_drift.py             Phase 1 drift curve
    scripts/analyse_run.py               corridor geometry against truth
    scripts/analyse_ekf_predict.py       EKF predict only checks
    scripts/analyse_ekf_update.py        EKF criteria for one run
    scripts/sweep_seeds.sh               ten seed sweep
    scripts/aggregate_sweep.py           sweep summary
    scripts/analyse_cost_grid.py         3d-1 criteria C1 to C3
    scripts/analyse_impossible_points.py LiDAR regression check
    scripts/analyse_column_errors.py     every beam against ray cast truth
    scripts/analyse_planner.py           3d-2 criteria P1 to P8
    scripts/analyse_latency.py           stage latency and the P7 budget
    scripts/analyse_*.py                 further diagnostics, one question each
```

The separation is deliberate. `corridor_experiments` is test apparatus and is
allowed to subscribe to Gazebo ground truth. `corridor_perception` is the system
under test and must not.

## Design decisions

**RANSAC excluded.** With a noiseless sensor in a bare corridor there are no
outliers, so it would solve a problem that does not exist. A single rejection
pass suffices while wall points outnumber contaminants roughly ten to one.
Revisit if sensor noise is enabled and proves it insufficient.

**Vehicle frame, not corridor frame.** Using the corridor frame would assume the
localisation this project exists to produce.

**Height band, not the range image angle criterion.** The angle criterion
separates ground from non ground. A wall and an obstacle are both non ground, so
it answers the wrong question. The organised 16 by 360 structure and the `ring`
field are preserved regardless, because the range image is the fallback if the
height band proves insufficient.

**A uniform grid and A\*, not a sampling planner.** The corridor is narrow and
the decision that matters is which side of each obstacle to pass, which a grid
search over a cost field answers exactly, and an independent exact search can
check every plan.

**Messages from one scan are paired by stamp.** Two topics computed from the
same scan arrive in either order. Every node that consumes two of them matches
by exact stamp and holds the earlier one, rather than taking the latest.

**Straight corridor only.** Curved bore geometry is out of scope for this
project.

**Plumbing borrowed, algorithms written.** Complete stacks exist that ship
`robot_localization`, SLAM Toolbox and Nav2. They are read for URDF and SDF
structure and never built upon. The EKF, the obstacle pipeline, the planner and
the controllers are the project; Nav2 is not used.

## Environment

Developed under WSL2 on Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 8.11.0
through the ROS vendor packages.

OpenGL runs on Mesa llvmpipe software rendering. The host has an NVIDIA RTX 3070
which is available for CUDA inside WSL, but the WSLg X server does not advertise
DRI3, so the Mesa d3d12 driver falls back to software. Measured real time factor
is 1.0 headless, 0.99 with the GUI on an empty world, and about 0.90 with the
LiDAR running. Since the obstacle pipeline was added it is about 0.6.

No deliverable depends on frame rate. Simulation time is decoupled from wall
clock, and node compute time is measured on the node rather than end to end
through the simulator. Wall clock under WSL2 is not trustworthy for duration
measurement: a backward jump of 2.6 s has been observed. Simulated durations
come from message header stamps. Node compute times are measured inside each
node on the monotonic clock. The Phase 3 pipeline latency uses the recorder's
receive times, which are wall clock, so it is reported as an upper bound
rather than an exact figure.

## Known limitations

- End wall degradation within 30 m of the bore face, characterised above and
  bounded rather than patched.
- `JointStatePublisher` runs at the 1 ms physics rate, publishing joint state at
  roughly 980 Hz where 100 Hz would serve. Capping it is pending.
- Drivetrain modelled kinematically rather than through `ros2_control`. Actuator
  lag is planned as a Phase 4 disturbance case.
- Articulated double chassis vehicles are out of scope.
- Longitudinal position is unobservable in a straight corridor and drifts with
  the wheel scale error, 1.010 m over 101 m. The filter's covariance on `x`
  grows accordingly and is not a position accuracy.
- The claimed lateral uncertainty excludes the extraction's own systematic
  error, which is under 0.5 mm and varies with pose. No sensor here can
  separate it from a genuine offset.
- Corridor geometry under occlusion: with an obstacle 5 to 10 m ahead close to
  one wall, that wall's fit drops from about 2000 points to about 340, the
  heading error reaches 4 to 5 mrad and the scan is still marked valid. To be
  fixed before the path following results.
- Obstacle shadows are unbounded, so obstacles on opposite sides in view at
  once leave no path (3d-2). To be removed in 3d-3.
- `obstacle_extraction` still takes the latest corridor geometry rather than the
  same scan's; the race fixed in the cost grid remains there.
- Static obstacles only. The tracker has no velocity state.
- Every number is measured in simulation. Simulation exercises the algorithms;
  it does not validate them against hardware.

## How this was built

Every phase was planned as a sequence of small steps, each with an exit
criterion written before the run that tested it, and a prediction made before
each measurement. The design decisions, the derivations, the criteria and the
debugging are the author's. Most of the C++ and Python was written with an AI
coding assistant working under that direction, and every result reported here
was checked against those criteria rather than taken on trust.
