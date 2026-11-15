// Path planner. SYSTEM UNDER TEST.
// Must never subscribe to Gazebo ground truth.
//
// A* over the cost grid, rebuilt from scratch on every grid. No map, no goal
// pose and no global frame: the path runs from the vehicle origin to ANY cell
// in the last column of the grid, which is simply the cheapest way forward
// through what the vehicle can currently see, under the side rule below.
//
// Moves are forward only: from column i to column i + 1, straight or one row
// to either side. The path is therefore a single y for every x. A vehicle
// cannot reverse or slide sideways, and a path with one y per x makes the
// side decision unambiguous: where the path sits as it passes an obstacle.
// The 45 degree steps this allows are not drivable as they stand; smoothing
// and the curvature check are 3d-4's job.
//
// Costs, per step into a cell:
//   step length * (1 + w * c / 98)
// where c is the destination's lateral cost (0 to 98). Cells at 99 (blocked
// for new paths) and 100 (lethal) cannot be entered. The start cell is exempt:
// the vehicle is already there, and refusing to leave it would turn a vehicle
// standing slightly inside a clearance band into "no path".
// With w = 1, staying 1 m off centre for 30 m costs about 4.1 against about
// 0.41 for the four diagonal steps that move back, so the path returns to the
// centreline whenever w > 0.1.
//
// Side rule, for every confirmed track on /obstacle_tracks with the grid's
// stamp, nearest first:
//   "right of track k" closes, in the column of its near face, every cell
//   whose centre is at or left of the track's y; "left" is the mirror image.
//   If, in that column, only one side has any open cell (below blocked_from
//   and not closed by a nearer track's side), that side is forced and no
//   search is run; most decisions are of this kind (P7 cost). Otherwise,
//   with the nearer tracks' sides already fixed, the best path is found for
//   each side of track k, giving C_R and C_L, and m = (C_L - C_R) / min.
//   Only one side passable: that side.
//   New track:        right, unless m < -e (left cheaper by more than e).
//   Committed track:  keep its side, unless the other side is cheaper by more
//                     than h (h > e), or the committed side has closed.
// The side is remembered per track id and forgotten when the id disappears.
// A grid that arrives before its own track array is held until that array
// arrives (the two topics have no ordering guarantee); planning it at once
// without the rule let a tie fall to A* queue order. Only a grid whose tracks
// never come within 0.5 s is planned without the rule, and it is counted.
// Why: a symmetric obstacle gives exact ties (the cells hold integers), and
// with plain A* the side then came from queue order, which 3 cm LiDAR noise
// reshuffles: 144 flips in 391 grids on the centre scenario. The right
// preference follows right hand traffic. The two thresholds are hysteresis:
// side margins come in two groups on this grid, under 1 % (both gaps the same
// number of cells) or over 6 % (one gap a whole cell wider), so e = 3 % sits
// in the empty range and h = 10 % allows at most one switch per obstacle.
// The path is the cheapest one under the chosen sides, not the cheapest
// overall: that is the point of the rule.
//
// Heuristic: remaining columns times the cheapest possible step (0.25 m at
// zero cost). It never overestimates, so each search is optimal under its
// constraints. The 3d-2 criterion P2 checks that independently.
//
// Output, /planned_path (nav_msgs/Path, base_link, stamp of the grid):
//   a pose at the centre of every cell on the path, or
//   NO poses when no path exists: an empty path means "blocked".
// Nothing at all is published when no grid arrives. The cost grid publishes
// nothing when the corridor geometry is missing, stale or invalid, and this
// node only ticks on a grid, so "I do not know where the walls are" stays
// silence all the way down instead of turning into a stale path. That is
// criterion P8.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <corridor_msgs/msg/obstacle_track_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class PathPlannerNode : public rclcpp::Node
{
public:
  PathPlannerNode() : Node("path_planner")
  {
    centre_weight_ = declare_parameter("centre_weight", 1.0);
    blocked_from_ = declare_parameter("blocked_from", 99);
    side_bias_ = declare_parameter("side_bias", 0.03);       // e
    side_switch_ = declare_parameter("side_switch", 0.10);   // h

    pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    // Best effort subscribers are compatible with reliable and best effort
    // publishers alike.
    track_sub_ = create_subscription<corridor_msgs::msg::ObstacleTrackArray>(
      "/obstacle_tracks", rclcpp::SensorDataQoS(),
      std::bind(&PathPlannerNode::on_tracks, this, std::placeholders::_1));
    sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/cost_grid", rclcpp::SensorDataQoS(),
      std::bind(&PathPlannerNode::on_grid, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "path_planner up: A*, forward only, centre weight %.2f, cells blocked from %ld, "
      "side rule right unless left cheaper by %.1f %%, switch at %.1f %%",
      centre_weight_, blocked_from_, 100.0 * side_bias_, 100.0 * side_switch_);
  }

private:
  struct Entry
  {
    double f;
    double g;
    int idx;
    bool operator>(const Entry & o) const { return f > o.f; }
  };

  struct Grid
  {
    int nx, ny;
    double res, ox, oy;
    const int8_t * data;
    int si, sj;
  };

  static int64_t stamp_ns(const builtin_interfaces::msg::Time & t)
  {
    return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
  }

  void on_tracks(const corridor_msgs::msg::ObstacleTrackArray::SharedPtr msg)
  {
    tracks_.push_back(msg);
    while (tracks_.size() > 20) { tracks_.pop_front(); }
    // A grid built from these tracks may have arrived first: the two topics
    // carry no ordering guarantee between them. Plan it now.
    const auto it = held_.find(stamp_ns(msg->header.stamp));
    if (it != held_.end()) {
      const auto grid = it->second;
      held_.erase(it);
      plan(grid, msg);
    }
  }

  // Best path cost from the start to any last column cell, with the cells in
  // `shut` closed in addition to those at blocked_from_ and above. Fills
  // `chain` with the cell indices of the path when one exists.
  double search(const Grid & G, const std::vector<char> & shut, std::vector<int> * chain) const
  {
    const int n = G.nx * G.ny;
    const double diag = G.res * std::sqrt(2.0);
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> g(n, inf);
    std::vector<int> parent(n, -1);
    std::vector<char> closed(n, 0);

    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    const int start = G.sj * G.nx + G.si;
    g[start] = 0.0;
    open.push({heuristic(G.si, G.nx, G.res), 0.0, start});

    int goal = -1;
    while (!open.empty()) {
      const Entry e = open.top();
      open.pop();
      if (closed[e.idx]) { continue; }
      closed[e.idx] = 1;
      const int i = e.idx % G.nx;
      const int j = e.idx / G.nx;
      if (i == G.nx - 1) { goal = e.idx; break; }      // any cell in the last column

      for (int dj = -1; dj <= 1; ++dj) {
        const int ni = i + 1, nj = j + dj;
        if (nj < 0 || nj >= G.ny) { continue; }
        const int nidx = nj * G.nx + ni;
        const int c = static_cast<int>(G.data[nidx]);
        if (c >= blocked_from_ || c < 0 || shut[nidx]) { continue; }
        const double step = (dj == 0 ? G.res : diag) * (1.0 + centre_weight_ * c / 98.0);
        const double ng = g[e.idx] + step;
        if (ng < g[nidx]) {
          g[nidx] = ng;
          parent[nidx] = e.idx;
          open.push({ng + heuristic(ni, G.nx, G.res), ng, nidx});
        }
      }
    }
    if (goal < 0) { return inf; }
    if (chain) {
      chain->clear();
      for (int k = goal; k >= 0; k = parent[k]) { chain->push_back(k); }
      std::reverse(chain->begin(), chain->end());
    }
    return g[goal];
  }

  // Close one side of a track in the column of its near face.
  static void close_side(const Grid & G, int col, double y, char side, std::vector<char> & shut)
  {
    for (int j = 0; j < G.ny; ++j) {
      const double yc = G.oy + (j + 0.5) * G.res;
      if ((side == 'R' && yc >= y) || (side == 'L' && yc <= y)) {
        shut[static_cast<size_t>(j) * G.nx + col] = 1;
      }
    }
  }

  void on_grid(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
  {
    // The tracks this grid was built from: the cost grid ticks on
    // /obstacle_tracks and keeps its stamp, so the match is exact.
    const int64_t gs = stamp_ns(grid->header.stamp);
    corridor_msgs::msg::ObstacleTrackArray::SharedPtr tr;
    for (auto it = tracks_.rbegin(); it != tracks_.rend(); ++it) {
      if (stamp_ns((*it)->header.stamp) == gs) { tr = *it; break; }
    }

    // Grids held longer than 0.5 s get no tracks any more: plan them
    // without the side rule, oldest first, and count them.
    for (auto it = held_.begin(); it != held_.end() && it->first < gs - 500000000LL;) {
      const auto old = it->second;
      it = held_.erase(it);
      ++unmatched_;
      plan(old, nullptr);
    }

    if (!tr) {
      ++held_count_;
      held_[gs] = grid;
      return;
    }
    plan(grid, tr);
  }

  void plan(const nav_msgs::msg::OccupancyGrid::SharedPtr & grid,
            const corridor_msgs::msg::ObstacleTrackArray::SharedPtr & tr)
  {
    const auto t0 = std::chrono::steady_clock::now();

    Grid G;
    G.nx = static_cast<int>(grid->info.width);
    G.ny = static_cast<int>(grid->info.height);
    G.res = grid->info.resolution;
    G.ox = grid->info.origin.position.x;
    G.oy = grid->info.origin.position.y;
    G.data = grid->data.data();
    if (G.nx < 2 || G.ny < 1 || grid->data.size() != static_cast<size_t>(G.nx) * G.ny) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "malformed grid: not planning");
      return;
    }

    // The start is the cell whose centre is the vehicle origin (0, 0).
    G.si = static_cast<int>(std::lround((0.0 - G.ox) / G.res - 0.5));
    G.sj = static_cast<int>(std::lround((0.0 - G.oy) / G.res - 0.5));
    if (G.si < 0 || G.si >= G.nx || G.sj < 0 || G.sj >= G.ny) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "vehicle origin outside the grid: not planning");
      return;
    }

    std::vector<char> shut(static_cast<size_t>(G.nx) * G.ny, 0);
    if (tr) {      // without tracks: no side rule, the memory is kept
      std::vector<const corridor_msgs::msg::ObstacleTrack *> order;
      std::unordered_set<uint32_t> present;
      for (const auto & t : tr->tracks) { order.push_back(&t); present.insert(t.id); }
      std::sort(order.begin(), order.end(),
        [](const auto * a, const auto * b) { return a->x < b->x; });

      for (const auto * t : order) {
        const int col = static_cast<int>(std::lround((t->x - G.ox) / G.res - 0.5));
        if (col <= G.si || col >= G.nx) { continue; }   // behind the start or beyond the grid

        // Which sides have any open cell in the near face column? If only
        // one does, the side is forced and costs no search; most decisions
        // are of this kind. Only a real two sided case runs C_R and C_L.
        bool open_r = false, open_l = false;
        for (int j = 0; j < G.ny; ++j) {
          const size_t k = static_cast<size_t>(j) * G.nx + col;
          const int c = static_cast<int>(G.data[k]);
          if (c >= blocked_from_ || c < 0 || shut[k]) { continue; }
          const double yc = G.oy + (j + 0.5) * G.res;
          if (yc < t->y) { open_r = true; } else if (yc > t->y) { open_l = true; }
        }
        if (!open_r && !open_l) { continue; }   // nothing passes it: the final search says so

        const auto it = committed_.find(t->id);
        char side;
        if (open_r && !open_l) {
          side = 'R';
          ++forced_;
        } else if (open_l && !open_r) {
          side = 'L';
          ++forced_;
        } else {
          std::vector<char> shut_r = shut, shut_l = shut;
          close_side(G, col, t->y, 'R', shut_r);
          close_side(G, col, t->y, 'L', shut_l);
          const double cr = search(G, shut_r, nullptr);
          const double cl = search(G, shut_l, nullptr);
          const bool fr = std::isfinite(cr), fl = std::isfinite(cl);
          if (!fr && !fl) { continue; }
          if (fr && !fl) { side = 'R'; }
          else if (fl && !fr) { side = 'L'; }
          else {
            const double m = (cl - cr) / std::min(cl, cr);
            if (it == committed_.end()) {
              side = (m < -side_bias_) ? 'L' : 'R';
            } else if (it->second == 'R') {
              side = (m < -side_switch_) ? 'L' : 'R';
            } else {
              side = (m > side_switch_) ? 'R' : 'L';
            }
          }
          ++searched_;
        }
        if (it != committed_.end() && it->second != side) { ++switches_; }
        committed_[t->id] = side;
        ++(side == 'R' ? decided_r_ : decided_l_);
        close_side(G, col, t->y, side, shut);
      }

      for (auto it = committed_.begin(); it != committed_.end();) {
        it = present.count(it->first) ? std::next(it) : committed_.erase(it);
      }
    }

    std::vector<int> chain;
    const double best = search(G, shut, &chain);

    nav_msgs::msg::Path path;
    path.header.stamp = grid->header.stamp;
    path.header.frame_id = grid->header.frame_id;
    if (std::isfinite(best)) {
      for (const int k : chain) {
        geometry_msgs::msg::PoseStamped p;
        p.header = path.header;
        p.pose.position.x = G.ox + (k % G.nx + 0.5) * G.res;
        p.pose.position.y = G.oy + (k / G.nx + 0.5) * G.res;
        p.pose.orientation.w = 1.0;
        path.poses.push_back(p);
      }
    } else {
      ++no_path_;
    }
    pub_->publish(path);

    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    plan_ms_.push_back(ms);
    if (plan_ms_.size() % 200 == 0) {
      std::vector<double> s = plan_ms_;
      std::sort(s.begin(), s.end());
      const auto at = [&s](double q) {
        return s[std::min(s.size() - 1, static_cast<size_t>(q * static_cast<double>(s.size())))];
      };
      RCLCPP_INFO(get_logger(),
        "plan time over %zu grids: median %.3f ms, p99 %.3f ms, max %.3f ms; no path on %zu",
        s.size(), at(0.5), at(0.99), s.back(), no_path_);
      RCLCPP_INFO(get_logger(),
        "side rule over %zu grids: decisions R %zu, L %zu (forced %zu, searched %zu); "
        "switches %zu; grids held for their tracks %zu; planned without tracks %zu",
        s.size(), decided_r_, decided_l_, forced_, searched_, switches_, held_count_,
        unmatched_);
    }
  }

  // Remaining columns times the cheapest possible step: never overestimates.
  static double heuristic(int i, int nx, double res) { return (nx - 1 - i) * res; }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Subscription<corridor_msgs::msg::ObstacleTrackArray>::SharedPtr track_sub_;
  double centre_weight_ = 1.0;
  int64_t blocked_from_ = 99;
  double side_bias_ = 0.03;
  double side_switch_ = 0.10;
  std::deque<corridor_msgs::msg::ObstacleTrackArray::SharedPtr> tracks_;
  std::unordered_map<uint32_t, char> committed_;
  std::map<int64_t, nav_msgs::msg::OccupancyGrid::SharedPtr> held_;   // by stamp
  size_t held_count_ = 0;
  std::vector<double> plan_ms_;
  size_t no_path_ = 0;
  size_t decided_r_ = 0, decided_l_ = 0, switches_ = 0, unmatched_ = 0;
  size_t forced_ = 0, searched_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
