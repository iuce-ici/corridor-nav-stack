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
}


def obstacle_model(index, x, y):
    sx, sy, sz = OBSTACLE_SIZE
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


def build(obstacles=()):
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
        obstacle_model(i, x, y) for i, (x, y) in enumerate(obstacles))

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
    out = OUTPUT if name is None else f"corridor_{name}.sdf"
    with open(out, "w") as f:
        f.write(build(obstacles))

    if name is not None:
        # Sidecar so the analysis reads obstacle truth from one place rather
        # than duplicating these constants.
        with open(f"corridor_{name}.obstacles.json", "w") as f:
            json.dump({"size": OBSTACLE_SIZE,
                       "obstacles": [{"x": x, "y": y} for x, y in obstacles]},
                      f, indent=2)

    print(f"Wrote {out}: {LENGTH} m long, {WIDTH} m wide, {HEIGHT} m high, "
          f"{len(obstacles)} obstacles")