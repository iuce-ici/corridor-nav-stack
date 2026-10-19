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

Three lateral controllers (pure pursuit, Stanley, LQR on a bicycle model state
space) are compared on a single metric harness: cross track error RMS and max,
heading error, control effort, compute time, and degradation under injected
sensor noise and disturbance.

**This is not a wall follower.** The wall measurement corrects an estimate; the
controller tracks the centreline using that estimate and never sees the LiDAR.
Only the measurement model, about 100 lines, is corridor specific. The EKF,
process model, controllers, bicycle model and metric harness are environment
independent.

**Status: Phase 1 complete, 4 September 2026. Phase 2 next.** Numbers in this
README are filled in as each phase closes and are absent until then. Everything
reported here is measured in simulation. Simulation exercises the algorithms; it
does not validate them against hardware, and no claim here should be read as a
hardware result.

## Milestones

| Phase | Content | Exit criterion | Status |
|---|---|---|---|
| 0 | Tooling floor, off the shelf robot | C++ node written by hand, rosbag recorded and replayed, repository exists | complete |
| 1 | Vehicle and corridor build, corridor geometry extraction, dead reckoned odometry | Offset and heading against Gazebo ground truth with error statistics and timing, documented odometry drift curve | **complete** |
| 2 | EKF localisation | Pose RMSE against ground truth, with and without the LiDAR update, written account of Q and R selection and of behaviour under mistuning | **complete** |
| 3 | Pure pursuit and Stanley, metric harness | Both closed loop on the EKF estimate, full metric table, lookahead swept | not started |
| 4 | LQR on own bicycle model state space | Three way comparison table including noise and disturbance cases, written verdict | not started |
| 5 | Ship | README with all numbers, architecture diagram, Docker, CI, demo video, one command clone and run | not started |

Target for v1.0: 15 November 2026.

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
uniform spacing, sensor frame -7.2 to +34.3 degrees. 180 degrees horizontal, 361
samples at 0.5 degrees, the front mount being occluded to the rear by its own
vehicle. 20 Hz, giving the 50 ms frame period used in service. Maximum range
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

## Build and run

```
colcon build
source install/setup.bash
```

A single launch brings up the world, the vehicle, transforms, the bridge, dead
reckoning and the run controller. Both nodes start from the spawn completion
event rather than from a launch timer, so ordering holds regardless of how long
Gazebo takes to load.

```
# terminal 1: start recording before the launch so nothing is missed
cd bags
ros2 bag record -o NAME /ekf/state /ekf/update /corridor_geometry \
  /odom_ekf /odom_dr /imu /model/vehicle/pose /clock

# terminal 2: one command runs everything
ros2 launch tunnel_vehicle simulation.launch.py \
  y:=0.0 target_x:=110.0 speed:=2.0 lidar_noise:=0.03 seed:=42
```

Arguments: `y` sets the lateral spawn offset, `yaw` the spawn heading in radians,
`target_x` the absolute x at which the run terminates, `speed` the commanded
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
  corridor_experiments/ test apparatus
        src/run_controller.cpp
    scripts/ekf_predict_consistency.py   predict step against Monte Carlo
    scripts/analyse_drift.py             Phase 1 drift curve
    scripts/analyse_run.py               corridor geometry against truth
    scripts/analyse_ekf_predict.py       EKF predict only checks
    scripts/analyse_ekf_update.py        EKF criteria for one run
    scripts/sweep_seeds.sh               ten seed sweep
    scripts/aggregate_sweep.py           sweep summary
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
it answers the wrong question. The organised 16 by 361 structure and the `ring`
field are preserved regardless, because the range image is the fallback if the
height band proves insufficient.

**Straight corridor only.** Curved bore geometry is out of scope for this
project.

**Plumbing borrowed, algorithms written.** Complete stacks exist that ship
`robot_localization`, SLAM Toolbox and Nav2. They are read for URDF and SDF
structure and never built upon. The EKF and the controllers are the project.

## Environment

Developed under WSL2 on Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 8.11.0
through the ROS vendor packages.

OpenGL runs on Mesa llvmpipe software rendering. The host has an NVIDIA RTX 3070
which is available for CUDA inside WSL, but the WSLg X server does not advertise
DRI3, so the Mesa d3d12 driver falls back to software. Measured real time factor
is 1.0 headless, 0.99 with the GUI on an empty world, and about 0.90 with the
LiDAR running.

No deliverable depends on frame rate. Simulation time is decoupled from wall
clock, and node compute time is measured on the node rather than end to end
through the simulator. Wall clock under WSL2 is not trustworthy for duration
measurement: a backward jump of 2.6 s has been observed. All timing in this
project comes from message header stamps.

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
- Every number is measured in simulation. Simulation exercises the algorithms;
  it does not validate them against hardware.
