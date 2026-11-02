#!/usr/bin/env python3
"""LinkedIn figure: the Gazebo gpu_lidar -45 degree mirror, from a real bag.

Three panels, all in base_link (x forward, y left), from ONE scan of a run
recorded with the 361 sample sensor, before the fix:
  1. the scene: corridor walls, vehicle, sensor, and the +45 / -45 beams
  2. what the LiDAR returned: the detector's height band in bird's eye view,
     with the -45 degree column highlighted, and the confirmed phantom tracks
  3. what the planner saw: the cost grid, with the phantom's lethal region
plus a footer with the before and after counts measured on 2 Nov 2026.

Usage:
  python3 make_mirror_figure.py bags/grid_clear_y15 [OUT.png]
"""
import sys
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
from matplotlib.colors import ListedColormap, BoundaryNorm

# Light mode slots from the dataviz reference palette.
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#8a8984"
GRID_LINE = "#e6e5e1"
NORMAL = "#2a78d6"      # slot 1: returns that are right
FAULT = "#eb6834"       # slot 2: the corrupted column and what it causes

LIDAR = np.array([4.2, 0.0, 1.0])
PITCH = 0.20944
Z_BAND = (0.1, 4.5)     # obstacle_extraction height band
X_MAX = 30.0
FAULT_AZ = -45.0
HALF_W = 4.0

# Footer numbers, measured 2 Nov 2026 with analyse_impossible_points.py at a
# y -1 spawn, where the copied ranges land behind the right wall.
BEFORE = "361 samples: 14 returns per scan from behind the wall, in 878 of 878 scans"
AFTER = "360 samples: 0 returns from behind the wall, in 0 of 170 scans"


# ---------------------------------------------------------------- loading

def load_from_bag(uri):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2

    topics = ["/scan/points", "/obstacle_tracks", "/cost_grid", "/corridor_geometry"]
    reader = None
    for sid in ("", "mcap", "sqlite3"):
        try:
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid),
                        rosbag2_py.ConverterOptions("", ""))
            break
        except Exception:
            reader = None
    if reader is None:
        sys.exit(f"could not open {uri}")
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in types]
    if missing:
        sys.exit(f"missing topics {missing}")
    cls = {t: get_message(types[t]) for t in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    out = {t: [] for t in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        m = deserialize_message(data, cls[topic])
        out[topic].append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, m))
    for t in topics:
        out[t].sort(key=lambda p: p[0])

    # Pick a scan in the middle of the stretch where a phantom track exists.
    with_tracks = [s for s, m in out["/obstacle_tracks"] if m.tracks]
    if not with_tracks:
        sys.exit("no confirmed tracks in this bag: nothing to show")
    target = with_tracks[len(with_tracks) // 2]

    def nearest(topic):
        stamps = np.array([s for s, _ in out[topic]])
        return out[topic][int(np.argmin(np.abs(stamps - target)))][1]

    cloud = nearest("/scan/points")
    tracks = nearest("/obstacle_tracks")
    grid = nearest("/cost_grid")
    geo = nearest("/corridor_geometry")

    raw = point_cloud2.read_points(cloud, field_names=["x", "y", "z"], skip_nans=True)
    try:
        p = np.column_stack([np.asarray(raw[k], float) for k in ("x", "y", "z")])
    except (TypeError, IndexError, ValueError):
        p = np.array([tuple(float(v) for v in q) for q in raw])
    p = p[np.all(np.isfinite(p), axis=1)]

    g = np.array(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)
    grid_info = (grid.info.origin.position.x, grid.info.origin.position.y,
                 grid.info.resolution)
    trk = [(t.x, t.y, t.length, t.width) for t in tracks.tracks]
    return p, trk, g, grid_info, geo.lateral_offset


# ---------------------------------------------------------------- geometry

def to_base(p_sensor):
    c, s = math.cos(PITCH), math.sin(PITCH)
    R = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    return (R @ p_sensor.T).T + LIDAR


def walls(offset):
    centre = -offset
    return centre + HALF_W, centre - HALF_W, centre


# ---------------------------------------------------------------- drawing

def style_axis(ax, title, ylim):
    ax.set_facecolor(SURFACE)
    ax.set_xlim(-1.5, X_MAX)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.set_title(title, loc="left", fontsize=12.5, color=INK, fontweight="bold", pad=8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=INK_2, labelsize=9)
    ax.set_xlabel("x ahead of the rear axle (m)", color=INK_2, fontsize=9)
    ax.set_ylabel("y, left + (m)", color=INK_2, fontsize=9)
    ax.grid(True, color=GRID_LINE, linewidth=0.6)
    ax.set_axisbelow(True)


def draw_walls(ax, left, right, centre, label=True):
    for y in (left, right):
        ax.plot([-1.5, X_MAX], [y, y], color=INK, linewidth=2.2, solid_capstyle="butt")
    ax.plot([-1.5, X_MAX], [centre, centre], color=MUTED, linewidth=1, linestyle=(0, (4, 4)))
    if label:
        ax.text(X_MAX - 0.3, left + 0.25, "left wall", ha="right", va="bottom",
                fontsize=9, color=INK_2)
        ax.text(X_MAX - 0.3, right - 0.25, "right wall", ha="right", va="top",
                fontsize=9, color=INK_2)
        ax.text(X_MAX - 0.3, centre + 0.15, "centreline", ha="right", va="bottom",
                fontsize=8.5, color=MUTED)


def draw_vehicle(ax):
    ax.add_patch(Rectangle((-1.0, -0.7), 5.0, 1.4, facecolor="#d9d8d3",
                           edgecolor=INK_2, linewidth=1.2, zorder=3))
    for x in (0.0, 3.0):
        for y in (-0.75, 0.75):
            ax.add_patch(Rectangle((x - 0.5, y - 0.15), 1.0, 0.3,
                                   facecolor=INK_2, edgecolor="none", zorder=4))
    ax.plot(LIDAR[0], 0.0, marker="o", markersize=7, color=INK, zorder=5)
    ax.text(LIDAR[0] - 0.2, 0.95, "LiDAR", ha="right", fontsize=9, color=INK, zorder=5)


def draw(p_sensor, tracks, grid, grid_info, offset, out_path):
    left, right, centre = walls(offset)
    ylim = (right - 0.8, left + 0.8)

    pb = to_base(p_sensor)
    az = np.degrees(np.arctan2(p_sensor[:, 1], p_sensor[:, 0]))
    band = (pb[:, 2] >= Z_BAND[0]) & (pb[:, 2] <= Z_BAND[1]) & (pb[:, 0] >= 0) & (pb[:, 0] <= X_MAX)
    fault = np.abs(az - FAULT_AZ) < 0.2

    fig = plt.figure(figsize=(9, 11.25), dpi=120, facecolor=SURFACE)
    gs = fig.add_gridspec(4, 1, height_ratios=[1, 1, 1, 0.55], hspace=0.9,
                          left=0.09, right=0.97, top=0.845, bottom=0.03)
    fig.text(0.09, 0.955, "A mirrored beam in Gazebo's GPU LiDAR",
             fontsize=17, fontweight="bold", color=INK)
    fig.text(0.09, 0.925,
             "Empty corridor, vehicle 1.5 m left of centre. One beam at exactly "
             "-45° returns the +45° range,\nand the obstacle pipeline turns it "
             "into a phantom obstacle. Gazebo Harmonic, gpu_lidar, 16 layers.",
             fontsize=10.5, color=INK_2, va="top", linespacing=1.4)

    # 1. The scene
    ax = fig.add_subplot(gs[0])
    style_axis(ax, "1  The scene, seen from above", ylim)
    draw_walls(ax, left, right, centre)
    draw_vehicle(ax)
    d_left = left                                  # +45 beam meets the left wall here
    hit_plus = (LIDAR[0] + d_left, d_left)
    hit_true = (LIDAR[0] + (-right), right)        # where a correct -45 beam would land
    copy = (LIDAR[0] + d_left, -d_left)            # where the copied range lands
    ax.plot([LIDAR[0], hit_plus[0]], [0, hit_plus[1]], color=NORMAL, linewidth=2)
    ax.plot(*hit_plus, marker="o", markersize=8, color=NORMAL)
    ax.annotate("+45° beam hits\nthe left wall", hit_plus, xytext=(hit_plus[0] + 1.6, hit_plus[1] - 0.2),
                fontsize=9, color=INK, va="top")
    ax.plot([LIDAR[0], hit_true[0]], [0, hit_true[1]], color=MUTED, linewidth=1.4,
            linestyle=(0, (3, 3)))
    ax.plot(*hit_true, marker="o", markersize=8, markerfacecolor=SURFACE,
            markeredgecolor=MUTED, markeredgewidth=1.6)
    ax.annotate("where the -45° beam should land", hit_true,
                xytext=(hit_true[0] + 0.8, hit_true[1] + 0.25), fontsize=9, color=INK_2)
    ax.plot([LIDAR[0], copy[0]], [0, copy[1]], color=FAULT, linewidth=2)
    ax.plot(*copy, marker="o", markersize=8, color=FAULT)
    ax.annotate("reported here: the right direction,\nbut the +45° beam's range", copy,
                xytext=(copy[0] + 1.2, copy[1] - 0.15), va="top", fontsize=9, color=INK)

    # 2. What the LiDAR returned
    ax = fig.add_subplot(gs[1])
    style_axis(ax, "2  What the LiDAR returned (one scan, 0.1 to 4.5 m height band)", ylim)
    draw_walls(ax, left, right, centre, label=False)
    ok = band & ~fault
    ax.scatter(pb[ok, 0], pb[ok, 1], s=3, color=NORMAL, alpha=0.55, linewidths=0, zorder=3,
               label="returns")
    bad = band & fault
    ax.scatter(pb[bad, 0], pb[bad, 1], s=55, color=FAULT, edgecolors=SURFACE,
               linewidths=1.2, zorder=7,
               label=f"the -45° column ({int(bad.sum())} returns, stacked by height)")
    for (tx, ty, tl, tw) in tracks:
        ax.add_patch(Rectangle((tx, ty - tw / 2 - 0.15), max(tl, 0.3), tw + 0.3,
                               facecolor="none", edgecolor=FAULT, linewidth=2, zorder=6))
    if tracks:
        tx, ty = tracks[0][0], tracks[0][1]
        ax.annotate("confirmed as an obstacle\nin an empty corridor", (tx + 0.4, ty),
                    xytext=(tx + 4.5, ty + 1.6), fontsize=9, color=INK,
                    arrowprops=dict(arrowstyle="-", color=INK_2, linewidth=1))
    ax.legend(loc="upper left", bbox_to_anchor=(0.0, -0.30), ncol=2, fontsize=8.5,
              frameon=False, markerscale=1.4)

    # 3. What the planner saw
    ax = fig.add_subplot(gs[2])
    style_axis(ax, "3  What the planner saw: the cost grid", ylim)
    x0, y0, res = grid_info
    ny, nx = grid.shape
    extent = (x0, x0 + nx * res, y0, y0 + ny * res)
    # Three classes only: free (graded by lateral cost), blocked for new paths, lethal.
    cls = np.where(grid >= 100, 3, np.where(grid >= 99, 2, np.where(grid >= 50, 1, 0)))
    cmap = ListedColormap(["#f4f3f0", "#e2e1dc", "#b9b8b2", "#4a4945"])
    ax.imshow(cls, origin="lower", extent=extent, cmap=cmap,
              norm=BoundaryNorm([-0.5, 0.5, 1.5, 2.5, 3.5], 4), interpolation="nearest",
              zorder=2)
    draw_walls(ax, left, right, centre, label=False)
    if tracks:
        tx, ty, tl, tw = tracks[0]
        ax.add_patch(Rectangle((tx, ty - tw / 2 - 0.15), max(tl, 0.3), tw + 0.3,
                               facecolor="none", edgecolor=FAULT, linewidth=2, zorder=6))
        ax.text(17.0, ty, "the phantom's shadow: lethal all the way to 30 m",
                ha="center", va="center", fontsize=9.5, color=SURFACE,
                fontweight="bold", zorder=7)
    swatch = [("lethal", "#4a4945"), ("blocked for new paths", "#b9b8b2"), ("free", "#f4f3f0")]
    handles = [Rectangle((0, 0), 1, 1, facecolor=c, edgecolor=MUTED, linewidth=0.6) for _, c in swatch]
    ax.legend(handles, [n for n, _ in swatch], loc="upper left", bbox_to_anchor=(0.0, -0.30),
              ncol=3, fontsize=8.5, frameon=False)

    # Footer: before and after
    ax = fig.add_subplot(gs[3])
    ax.axis("off")
    ax.text(0.0, 1.0, "The fix", fontsize=12.5, fontweight="bold", color=INK,
            transform=ax.transAxes, va="top")
    ax.text(0.0, 0.66, "●  " + BEFORE, fontsize=10, color=INK, transform=ax.transAxes, va="top")
    ax.text(0.0, 0.42, "●  " + AFTER, fontsize=10, color=INK, transform=ax.transAxes, va="top")
    ax.text(0.0, 0.12, "Found by comparing every beam with its exact ray-cast range in an empty "
            "corridor. Simulation only.", fontsize=9, color=INK_2, transform=ax.transAxes, va="top")

    fig.savefig(out_path, facecolor=SURFACE)
    print(f"wrote {out_path}")


def main():
    if len(sys.argv) not in (2, 3):
        print(__doc__)
        sys.exit(1)
    out = sys.argv[2] if len(sys.argv) == 3 else "gazebo_lidar_mirror.png"
    draw(*load_from_bag(sys.argv[1]), out)


if __name__ == "__main__":
    main()
