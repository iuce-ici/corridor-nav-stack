#!/usr/bin/env python3
"""Generate a straight rectangular corridor world for Gazebo Harmonic.

Corridor runs along +x. Centreline is y = 0. Floor is z = 0.
Entrance is at x = 0.
"""

WIDTH = 8.0
LENGTH = 150.0
HEIGHT = 8.0
WALL_THICKNESS = 0.2

# Obstacles. Each is (x, y) of its centre; size and height are common.
# Height 1.5 m against a LiDAR at z = 1.0, so only the upper layers see it and
# a far obstacle is a handful of returns: that is the realistic case.
# Every obstacle stays at least 1 m clear of both walls, so its points fall
# outside the 0.30 m outlier band of the wall fit and cannot corrupt it.
OBSTACLE_SIZE = (1.2, 1.2, 1.5)

SCENARIOS = {
    "clear":        [],
    "single_left":  [(50.0,  0.6)],
    "single_right": [(50.0, -0.6)],
    "slalom":       [(40.0,  0.8), (70.0, -0.8)],
    "narrow":       [(50.0,  1.0)],
    "late":         [(18.0,  0.0)],
    # Wall to wall, for the 3d-2 "no path" criterion. Gaps of 0.3 m sit under
    # the extraction merge gap of 1.0 m, so the three join into one box about
    # 4.2 m wide; padded and inflated it blocks the whole corridor. Outer
    # edges at +-2.1 m keep 1.9 m clear of both walls.
    "blocked":      [(50.0, -1.5), (50.0,  0.0), (50.0,  1.5)],
    # Two on the same side, 8 m apart, for the 3d-3 replan branch. The second
    # sits behind the first at the same lateral position, so for most of the
    # approach it is in the first one's shadow. 8 m is shorter than the
    # roughly 10 m S curve back to the centreline, so a return started after
    # passing the first would run into the second.
    "tandem":       [(40.0,  0.6), (48.0,  0.6)],
    # Dead centre, for the 3d-2 side flip check. Plannable width is +-2.7 and
    # an obstacle blocks its width plus 2 x 1.8 m, so a 1.2 m box on the
    # centreline leaves 0.3 m gaps, one cell or none. At 0.6 m wide (size
    # below) each gap is 0.6 m on both sides: an exact geometric tie, so only
    # measurement noise picks the side.
    "centre":       [(50.0,  0.0)],
    # Two tracks in the grid at once, for the side rule's nearest first
    # ordering. Both force right: the near box, close to the left wall,
    # blocks y -0.2 to +4.6; the far box, close to the centreline, blocks
    # -1.9 to +2.9, so it is the far box that binds the path. Opposite sides
    # cannot be used: with shadows from the near face to the grid end, two
    # opposite bands cover the whole plannable width. The far box is in
    # direct view: every ray to it passes below y 1.1 and the near box starts
    # at 1.6, so its whole face shows from vehicle x 21.4 (where it enters the
    # 30 m detection range). A first layout with the far box further out,
    # (40, 0.45) and (52, 2.2), left it 3 to 6 returns and it never became a
    # track. Run with target_x 24.
    "pair":         [(40.0,  2.2), (52.0,  0.5)],
}

# Per scenario size overrides (x depth, y width, z height). Depth stays 1.2 so
# the near face is still at x - 0.6, as the analysis scripts assume.
SCENARIO_SIZE = {
    "centre": (1.2, 0.6, 1.5),
}


def obstacle_model(index, x, y, size=OBSTACLE_SIZE):
    sx, sy, sz = size
    return f"""    <model name="obstacle_{index}">
      <static>true</static>
      <link name="body">
        <pose>{x} {y} {sz / 2.0} 0 0 0</pose>
        <collision name="body_collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="body_visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>0.8 0.35 0.1 1</ambient>
            <diffuse>0.8 0.35 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

OUTPUT = "corridor.sdf"


def box_link(name, size, pose, colour):
    sx, sy, sz = size
    r, g, b = colour
    return f"""    <link name="{name}">
      <pose>{pose}</pose>
      <collision name="{name}_collision">
        <geometry>
          <box><size>{sx} {sy} {sz}</size></box>
        </geometry>
      </collision>
      <visual name="{name}_visual">
        <geometry>
          <box><size>{sx} {sy} {sz}</size></box>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
"""


def build(obstacles=(), size=OBSTACLE_SIZE):
    half_w = WIDTH / 2.0
    half_t = WALL_THICKNESS / 2.0
    mid_x = LENGTH / 2.0

    links = []

    # Floor: inner face (top) at z = 0.
    links.append(box_link(
        "floor",
        (LENGTH, WIDTH, WALL_THICKNESS),
        f"{mid_x} 0 {-half_t} 0 0 0",
        (0.4, 0.4, 0.4)))

    # Side walls: inner faces at y = +half_w and y = -half_w.
    links.append(box_link(
        "wall_left",
        (LENGTH, WALL_THICKNESS, HEIGHT),
        f"{mid_x} {half_w + half_t} {HEIGHT / 2.0} 0 0 0",
        (0.6, 0.6, 0.55)))

    links.append(box_link(
        "wall_right",
        (LENGTH, WALL_THICKNESS, HEIGHT),
        f"{mid_x} {-(half_w + half_t)} {HEIGHT / 2.0} 0 0 0",
        (0.6, 0.6, 0.55)))

    # Ceiling: inner face (bottom) at z = HEIGHT.
    links.append(box_link(
        "ceiling",
        (LENGTH, WIDTH, WALL_THICKNESS),
        f"{mid_x} 0 {HEIGHT + half_t} 0 0 0",
        (0.5, 0.5, 0.5)))

    # End caps: inner faces at x = 0 and x = LENGTH.
    links.append(box_link(
        "end_near",
        (WALL_THICKNESS, WIDTH, HEIGHT),
        f"{-half_t} 0 {HEIGHT / 2.0} 0 0 0",
        (0.45, 0.45, 0.45)))

    links.append(box_link(
        "end_far",
        (WALL_THICKNESS, WIDTH, HEIGHT),
        f"{LENGTH + half_t} 0 {HEIGHT / 2.0} 0 0 0",
        (0.45, 0.45, 0.45)))

    body = "".join(links)

    obstacle_xml = "".join(
        obstacle_model(i, x, y, size) for i, (x, y) in enumerate(obstacles))

    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="corridor">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system"
                name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="corridor_structure">
      <static>true</static>
{body}    </model>
{obstacle_xml}
  </world>
</sdf>
"""


if __name__ == "__main__":
    import json
    import sys

    name = sys.argv[1] if len(sys.argv) > 1 else None
    if name is not None and name not in SCENARIOS:
        sys.exit(f"unknown scenario {name}. Known: {', '.join(SCENARIOS)}")

    obstacles = SCENARIOS[name] if name else []
    size = SCENARIO_SIZE.get(name, OBSTACLE_SIZE)
    out = OUTPUT if name is None else f"corridor_{name}.sdf"
    with open(out, "w") as f:
        f.write(build(obstacles, size))

    if name is not None:
        # Sidecar so the analysis reads obstacle truth from one place rather
        # than duplicating these constants.
        with open(f"corridor_{name}.obstacles.json", "w") as f:
            json.dump({"size": size,
                       "obstacles": [{"x": x, "y": y} for x, y in obstacles]},
                      f, indent=2)

    print(f"Wrote {out}: {LENGTH} m long, {WIDTH} m wide, {HEIGHT} m high, "
          f"{len(obstacles)} obstacles")