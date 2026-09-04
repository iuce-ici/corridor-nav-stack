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
| 2 | EKF localisation | Pose RMSE against ground truth, with and without the LiDAR update, written account of Q and R selection and of behaviour under mistuning | next |
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

Straight run, 100.176 m travelled at 2.0 m/s, ground truth lateral position
constant at zero throughout, so all reported lateral error belongs to the
estimator.

| Distance travelled | Total error | Longitudinal | Lateral |
|---|---|---|---|
| 10 m | 0.127 m | +0.114 m | +0.055 m |
| 20 m | 0.261 m | +0.214 m | +0.150 m |
| 30 m | 0.426 m | +0.313 m | +0.289 m |
| 40 m | 0.626 m | +0.411 m | +0.471 m |
| 50 m | 0.863 m | +0.509 m | +0.698 m |
| 60 m | 1.137 m | +0.605 m | +0.962 m |
| 70 m | 1.452 m | +0.700 m | +1.272 m |
| 80 m | 1.808 m | +0.794 m | +1.624 m |
| 90 m | 2.206 m | +0.887 m | +2.020 m |
| 100 m | 2.644 m | +0.972 m | +2.459 m |

Reported distance 101.176 m against a true 100.176 m, which is the one percent
scale error propagating exactly as expected. Final integrated heading 3.298 deg
against a true heading of zero.

**Against closed form.** Longitudinal error was predicted at 1.00 m and measured
at 0.970 m. Lateral error from a constant gyro bias should follow `b*d^2/(2v)`,
predicting 2.18 m, and measured 2.467 m. The 13 percent excess has a named
cause: the node integrates for three seconds before motion begins, and the run
controller ramps to speed under a 2 m/s squared acceleration cap, so heading
accumulates over a period in which less ground is covered than the constant
speed formula assumes. The measured lateral to distance ratio falls short of a
pure quadratic in the same direction, which is the signature of that cause
rather than of noise.

### Corridor geometry extraction

Lateral offset and heading from a single scan: transform to `base_link`, filter
to a z band, split left from right by the sign of y, and fit a line to each wall
by fit, reject points beyond 0.30 m perpendicular, refit.

Measured at a 1.5 m lateral offset in an 8 m corridor. The run was deliberately
continued past the clean region so the end wall degradation could be
characterised.

| Regime | `base_link` x | Offset error | Width error | Flag |
|---|---|---|---|---|
| End wall beyond sensor range | 0 to 115.8 | 0.44 mm | 0.47 mm | valid |
| End wall entering the fits | 115.8 to 128 | 104 mm | 289 mm | invalid |
| End wall dominating the fits | 128 to 141 | 751 mm | 2216 mm | invalid |

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
of scene throughout. Range noise zero for the current baseline.

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

Fixed at 42 through `--seed 42` in `gz_args`. This is part of the configuration,
not an implementation detail. Without it, Gazebo draws the sign of the gyro bias
independently on each run even when `bias_stddev` is zero, so two otherwise
identical runs drift in opposite directions. With the seed fixed, the noise
sequence is reproducible to 3.4e-12 rad/s across runs; the residual is physics
solver thread ordering, not the random source.

A fixed seed means every run shares one noise realisation, so a single result is
one draw rather than a statistic. Development and comparison run on seed 42.
Headline figures from Phase 2 onward will be taken over a sweep of seeds.

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
ros2 bag record -o NAME /odom_dr /model/vehicle/pose /clock

# terminal 2: one command runs everything
ros2 launch tunnel_vehicle simulation.launch.py y:=0.0 target_x:=110.0 speed:=2.0
```

Arguments: `y` sets the lateral spawn offset, `target_x` the absolute x at which
the run terminates, `speed` the commanded forward speed. The vehicle spawns at
x = 10, so `target_x:=110.0` gives 100 m of travel.

The run controller terminates on one of three conditions and reports which:
target x reached, simulation time limit exceeded, or lateral bound exceeded. The
last two are guards. A run ending that way is a failed run and must not be
averaged into results. Read the termination line before analysing the bag.

Gazebo is closed manually so a failed run can be inspected.

**Measurement runs must terminate before x = 115.8**, the onset of end wall
degradation. `target_x:=110.0` leaves 5.8 m of margin. Feeding the degraded
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
  corridor_experiments/ test apparatus
    src/run_controller.cpp
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
- The drift analysis script is not yet in the repository, so the published drift
  curve is not currently reproducible by a reader.
- Drivetrain modelled kinematically rather than through `ros2_control`. Actuator
  lag is planned as a Phase 4 disturbance case.
- Articulated double chassis vehicles are out of scope.
