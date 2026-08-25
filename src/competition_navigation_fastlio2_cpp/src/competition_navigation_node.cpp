// C++ port of competition_navigation_node.py -- a 1:1 translation of the
// navigation logic.  Only the language changed; the exploration logic is
// byte-for-byte identical to the Python package.  See the Python source for
// the (extensive) rationale comments; this file preserves the behaviour.
#include <ros/ros.h>

#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "competition_navigation_fastlio2_cpp/corridor_graph.h"
#include "competition_navigation_fastlio2_cpp/hdplanner_policy.h"

namespace nav = competition_navigation_fastlio2_cpp;

namespace {

constexpr uint8_t UNKNOWN = 127;
constexpr uint8_t FREE = 255;
constexpr uint8_t OCCUPIED = 1;

constexpr int DEFAULT_FLOOR_COUNT = 3;
constexpr int DEFAULT_ROOMS_PER_FLOOR = 4;
constexpr double DEFAULT_FLOOR_HEIGHT = 2.6;
constexpr double DEFAULT_SCAN_MIN_RANGE = 2.0;

double clampd(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

// Python round(): round-half-to-even (banker's rounding), unlike std::round.
int64_t py_round(double x) { return std::lrint(x); }
double py_round1(double x) { return std::nearbyint(x * 10.0) / 10.0; }
double py_round2(double x) { return std::nearbyint(x * 100.0) / 100.0; }

double yaw_from_quat(double qx, double qy, double qz, double qw) {
  return std::atan2(2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz));
}

// Bresenham line, inclusive of both endpoints (matches the Python generator).
std::vector<std::pair<int, int>> bresenham(int x0, int y0, int x1, int y1) {
  std::vector<std::pair<int, int>> cells;
  int dx = std::abs(x1 - x0);
  int sx = x0 < x1 ? 1 : -1;
  int dy = -std::abs(y1 - y0);
  int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;
  int x = x0, y = y0;
  while (true) {
    cells.emplace_back(x, y);
    if (x == x1 && y == y1) break;
    int twice = 2 * error;
    if (twice >= dy) {
      error += dy;
      x += sx;
    }
    if (twice <= dx) {
      error += dx;
      y += sy;
    }
  }
  return cells;
}

std::string ssprintf(const char* fmt, ...) {
  char buf[2048];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  return std::string(buf);
}

}  // namespace

// ---------------------------------------------------------------------------
// Pure grid helpers (Python @staticmethod / module-level functions).
// ---------------------------------------------------------------------------
namespace {

struct Pose {
  double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
};

struct Bounds {
  double x_min = 0.0, x_max = 0.0, y_min = 0.0, y_max = 0.0;
};

struct RoomEntryCandidate {
  std::string side;
  double x = 0.0, y = 0.0, yaw = 0.0, corridor_s = 0.0, confidence = 0.0;
};

struct RoomPlan {
  bool has_gain_cells = false;
  int gain_cells = 0;
  double path_m = -1.0, return_m = -1.0, score = -1.0;
  int candidate_count = 0, cluster_count = 0;
};

struct DistanceTree {
  std::vector<int32_t> distance;
  std::vector<int16_t> parent_x;
  std::vector<int16_t> parent_y;
};

// 8-connected frontier components.
std::vector<std::vector<std::pair<int, int>>> frontier_clusters(
    const std::vector<uint8_t>& frontier_mask, int n) {
  std::vector<std::vector<std::pair<int, int>>> clusters;
  std::vector<uint8_t> visited(n * n, 0);
  for (int seed_y = 0; seed_y < n; ++seed_y) {
    for (int seed_x = 0; seed_x < n; ++seed_x) {
      int seed = seed_y * n + seed_x;
      if (!frontier_mask[seed] || visited[seed]) continue;
      visited[seed] = 1;
      std::vector<std::pair<int, int>> cluster;
      std::deque<std::pair<int, int>> queue;
      queue.emplace_back(seed_x, seed_y);
      while (!queue.empty()) {
        auto [cx, cy] = queue.front();
        queue.pop_front();
        cluster.emplace_back(cx, cy);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            int nx = cx + dx, ny = cy + dy;
            if (nx < 0 || nx >= n || ny < 0 || ny >= n) continue;
            int ni = ny * n + nx;
            if (frontier_mask[ni] && !visited[ni]) {
              visited[ni] = 1;
              queue.emplace_back(nx, ny);
            }
          }
        }
      }
      clusters.push_back(std::move(cluster));
    }
  }
  return clusters;
}

// 4-connected BFS distance tree.  start == nullopt => all -1.
DistanceTree distance_tree(const std::vector<uint8_t>& traversable, int n,
                           std::optional<std::pair<int, int>> start) {
  DistanceTree tree;
  tree.distance.assign(n * n, -1);
  tree.parent_x.assign(n * n, -1);
  tree.parent_y.assign(n * n, -1);
  if (!start.has_value()) return tree;
  int sx = start->first, sy = start->second;
  if (!(sx >= 0 && sx < n && sy >= 0 && sy < n)) return tree;
  std::deque<std::pair<int, int>> queue;
  tree.distance[sy * n + sx] = 0;
  queue.emplace_back(sx, sy);
  while (!queue.empty()) {
    auto [cx, cy] = queue.front();
    queue.pop_front();
    int next = tree.distance[cy * n + cx] + 1;
    for (auto [dx, dy] : {std::pair{1, 0}, std::pair{-1, 0}, std::pair{0, 1},
                          std::pair{0, -1}}) {
      int nx = cx + dx, ny = cy + dy;
      if (nx < 0 || nx >= n || ny < 0 || ny >= n) continue;
      int ni = ny * n + nx;
      if (!traversable[ni] || tree.distance[ni] >= 0) continue;
      tree.distance[ni] = next;
      tree.parent_x[ni] = static_cast<int16_t>(cx);
      tree.parent_y[ni] = static_cast<int16_t>(cy);
      queue.emplace_back(nx, ny);
    }
  }
  return tree;
}

// Farthest-point sampling from one cluster (matches _sample_cluster_cells).
std::vector<std::pair<int, int>> sample_cluster_cells(
    const std::vector<std::pair<int, int>>& cluster, int count) {
  if (cluster.empty() || count <= 0) return {};
  double cx = 0.0, cy = 0.0;
  for (const auto& c : cluster) {
    cx += c.first;
    cy += c.second;
  }
  cx /= cluster.size();
  cy /= cluster.size();
  auto sq = [&](const std::pair<int, int>& c) {
    return (c.first - cx) * (c.first - cx) + (c.second - cy) * (c.second - cy);
  };
  std::pair<int, int> first = *std::min_element(
      cluster.begin(), cluster.end(),
      [&](const auto& a, const auto& b) { return sq(a) < sq(b); });
  std::vector<std::pair<int, int>> selected{first};
  std::set<std::pair<int, int>> remaining(cluster.begin(), cluster.end());
  remaining.erase(first);
  while (!remaining.empty() &&
         static_cast<int>(selected.size()) < count) {
    std::pair<int, int> next;
    double best = -1.0;
    for (const auto& c : remaining) {
      double m = std::numeric_limits<double>::infinity();
      for (const auto& chosen : selected) {
        double d = (c.first - chosen.first) * (c.first - chosen.first) +
                   (c.second - chosen.second) * (c.second - chosen.second);
        m = std::min(m, d);
      }
      if (m > best) {
        best = m;
        next = c;
      }
    }
    selected.push_back(next);
    remaining.erase(next);
  }
  return selected;
}

// Separable square dilation (matches _dilate_mask).
std::vector<uint8_t> dilate_mask(const std::vector<uint8_t>& mask, int n,
                                 int radius) {
  std::vector<uint8_t> result(n * n, 0);
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      int src_x0 = std::max(0, -dx), src_x1 = std::min(n, n - dx);
      int dst_x0 = std::max(0, dx), dst_x1 = std::min(n, n + dx);
      int src_y0 = std::max(0, -dy), src_y1 = std::min(n, n - dy);
      int dst_y0 = std::max(0, dy), dst_y1 = std::min(n, n + dy);
      for (int sy = src_y0, ty = dst_y0; sy < src_y1; ++sy, ++ty) {
        for (int sx = src_x0, tx = dst_x0; sx < src_x1; ++sx, ++tx) {
          if (mask[sy * n + sx]) result[ty * n + tx] = 1;
        }
      }
    }
  }
  return result;
}

// 4-adjacency frontier mask (matches _frontier_mask): free cell adjacent to an
// unknown cell.
std::vector<uint8_t> frontier_mask_4(const std::vector<uint8_t>& belief,
                                     int n) {
  std::vector<uint8_t> adj_unknown(n * n, 0);
  for (int y = 0; y < n; ++y) {
    for (int x = 0; x < n; ++x) {
      if (belief[y * n + x] == UNKNOWN) {
        if (y > 0) adj_unknown[(y - 1) * n + x] = 1;
        if (y < n - 1) adj_unknown[(y + 1) * n + x] = 1;
        if (x > 0) adj_unknown[y * n + x - 1] = 1;
        if (x < n - 1) adj_unknown[y * n + x + 1] = 1;
      }
    }
  }
  std::vector<uint8_t> out(n * n, 0);
  for (int i = 0; i < n * n; ++i) {
    if (belief[i] == FREE && adj_unknown[i]) out[i] = 1;
  }
  return out;
}

void append_path(std::vector<std::pair<double, double>>& result,
                 const std::vector<std::pair<double, double>>& path) {
  for (const auto& point : path) {
    if (result.empty() ||
        std::hypot(point.first - result.back().first,
                   point.second - result.back().second) > 0.10) {
      result.push_back(point);
    }
  }
}

}  // namespace

class CompetitionNavigation {
 public:
  explicit CompetitionNavigation(const ros::NodeHandle& pnh) : pnh_(pnh) {
    // ---- parameters (mirrors __init__) ----
    pnh_.param<std::string>("map_frame", map_frame_, "odom");
    pnh_.param<std::string>("scan_topic", scan_topic_, "/scan");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/Odometry_gazebo");
    pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    pnh_.param<double>("cell_size", cell_size_, 0.4);
    pnh_.param<double>("map_size_m", map_size_m_, 80.0);
    pnh_.param<double>("planning_period", planning_period_, 2.0);
    pnh_.param<double>("control_period", control_period_, 0.1);
    pnh_.param<double>("map_publish_rate", map_publish_rate_, 5.0);
    map_publish_rate_ = std::max(1.0, map_publish_rate_);
    pnh_.param<double>("frontier_revisit_radius", frontier_revisit_radius_,
                       0.8);
    frontier_revisit_radius_ = std::max(0.0, frontier_revisit_radius_);
    frontier_revisit_radius_cells_ = std::max(
        0, static_cast<int>(std::ceil(frontier_revisit_radius_ / cell_size_)));
    pnh_.param<int>("scan_stride", scan_stride_, 8);
    scan_stride_ = std::max(1, scan_stride_);
    pnh_.param<int>("max_scan_points", max_scan_points_, 1800);
    max_scan_points_ = std::max(100, max_scan_points_);
    pnh_.param<double>("scan_min_range", scan_min_range_,
                       DEFAULT_SCAN_MIN_RANGE);
    scan_min_range_ = std::max(0.0, scan_min_range_);
    pnh_.param<double>("scan_max_range", scan_max_range_, 22.0);
    scan_max_range_ = std::max(scan_min_range_ + 0.5, scan_max_range_);
    pnh_.param<bool>("scan_self_filter_enabled", scan_self_filter_enabled_,
                     true);
    pnh_.param<double>("scan_self_filter_front", scan_self_filter_front_, 0.75);
    pnh_.param<double>("scan_self_filter_rear", scan_self_filter_rear_, 0.60);
    pnh_.param<double>("scan_self_filter_left", scan_self_filter_left_, 0.55);
    pnh_.param<double>("scan_self_filter_right", scan_self_filter_right_, 0.55);
    for (double* v : {&scan_self_filter_front_, &scan_self_filter_rear_,
                      &scan_self_filter_left_, &scan_self_filter_right_}) {
      *v = std::max(0.0, *v);
    }
    pnh_.param<int>("map_progress_cell_batch", map_progress_cell_batch_, 8);
    map_progress_cell_batch_ = std::max(1, map_progress_cell_batch_);
    pnh_.param<double>("obstacle_clear_percent", obstacle_clear_percent_, 80.0);
    pnh_.param<int>("obstacle_clear_min_obs", obstacle_clear_min_obs_, 5);
    obstacle_clear_min_obs_ = std::max(1, obstacle_clear_min_obs_);
    pnh_.param<double>("max_linear", max_linear_, 0.5);
    pnh_.param<double>("linear_gain", linear_gain_, 1.0);
    pnh_.param<double>("max_lateral", max_lateral_, 0.15);
    pnh_.param<double>("max_yaw_rate", max_yaw_rate_, 1.2);
    pnh_.param<double>("robot_radius", robot_radius_, 0.20);
    robot_radius_ = std::max(0.0, robot_radius_);
    robot_radius_cells_ =
        robot_radius_ <= 0.0
            ? 0
            : std::max(1, static_cast<int>(std::ceil(robot_radius_ / cell_size_)));
    pnh_.param<bool>("allow_unknown_return", allow_unknown_return_, false);
    pnh_.param<bool>("auto_start", auto_start_, false);
    pnh_.param<double>("auto_return_after_sec", auto_return_after_sec_, 0.0);
    pnh_.param<bool>("require_hdplanner", require_hdplanner_, true);
    pnh_.param<int>("floor_count", floor_count_, DEFAULT_FLOOR_COUNT);
    floor_count_ = std::max(1, floor_count_);
    pnh_.param<double>("floor_height", floor_height_, DEFAULT_FLOOR_HEIGHT);
    pnh_.param<double>("min_floor_coverage", min_floor_coverage_, 0.45);
    double completion_min_floor = 0.70;
    pnh_.param<double>("completion_min_floor_coverage", completion_min_floor,
                       0.70);
    completion_min_floor_coverage_ =
        std::max(min_floor_coverage_, completion_min_floor);
    pnh_.param<double>("completion_min_sector_coverage",
                       completion_min_sector_coverage_, 0.30);
    pnh_.param<double>("completion_min_floor_distance",
                       completion_min_floor_distance_, 35.0);
    pnh_.param<int>("completion_cycles_required", completion_cycles_required_,
                    3);
    completion_cycles_required_ = std::max(2, completion_cycles_required_);
    pnh_.param<int>("no_frontier_cycles_required",
                    no_frontier_cycles_required_, 6);
    no_frontier_cycles_required_ = std::max(2, no_frontier_cycles_required_);
    pnh_.param<double>("stair_speed", stair_speed_, 0.80);
    stair_speed_ = std::max(0.05, stair_speed_);
    pnh_.param<double>("stair_landing_speed", stair_landing_speed_, 0.40);
    stair_landing_speed_ = std::max(0.0, stair_landing_speed_);
    pnh_.param<double>("stair_yaw_rate", stair_yaw_rate_, 1.0);
    stair_yaw_rate_ = std::max(0.05, stair_yaw_rate_);
    pnh_.param<bool>("stair_flat_use_max_linear", stair_flat_use_max_linear_,
                     true);
    pnh_.param<double>("stair_lookahead", stair_lookahead_, 0.30);
    stair_lookahead_ = std::max(0.05, stair_lookahead_);
    pnh_.param<double>("stair_brake_distance", stair_brake_distance_, 0.35);
    stair_brake_distance_ = std::max(stair_lookahead_, stair_brake_distance_);
    pnh_.param<double>("stair_lateral_limit", stair_lateral_limit_, 0.08);
    stair_lateral_limit_ = std::max(0.03, stair_lateral_limit_);
    pnh_.param<double>("stair_cruise_heading_tolerance",
                       stair_cruise_heading_tolerance_, 0.70);
    pnh_.param<double>("stair_cruise_lateral_tolerance",
                       stair_cruise_lateral_tolerance_, 0.22);
    pnh_.param<double>("stair_turn_heading_threshold",
                       stair_turn_heading_threshold_, 0.60);
    pnh_.param<double>("min_room_coverage", min_room_coverage_, 0.70);
    min_room_coverage_ = std::max(0.50, min_room_coverage_);
    pnh_.param<double>("room_task_timeout", room_task_timeout_, 120.0);
    room_task_timeout_ = std::max(30.0, room_task_timeout_);
    pnh_.param<int>("room_frontier_min_cluster_cells",
                    room_frontier_min_cluster_cells_, 3);
    room_frontier_min_cluster_cells_ =
        std::max(1, room_frontier_min_cluster_cells_);
    pnh_.param<int>("room_candidates_per_cluster", room_candidates_per_cluster_,
                    2);
    room_candidates_per_cluster_ = std::max(1, room_candidates_per_cluster_);
    pnh_.param<int>("room_candidate_limit", room_candidate_limit_, 10);
    room_candidate_limit_ = std::max(1, room_candidate_limit_);
    pnh_.param<double>("room_information_range", room_information_range_, 6.0);
    room_information_range_ = std::max(cell_size_, room_information_range_);
    double angle_step = 10.0;
    pnh_.param<double>("room_information_angle_step_deg", angle_step, 10.0);
    room_information_angle_step_deg_ = clampd(angle_step, 2.0, 45.0);
    pnh_.param<double>("room_path_cost_weight", room_path_cost_weight_, 0.45);
    room_path_cost_weight_ = std::max(0.0, room_path_cost_weight_);
    pnh_.param<double>("room_return_cost_weight", room_return_cost_weight_,
                       0.20);
    room_return_cost_weight_ = std::max(0.0, room_return_cost_weight_);
    pnh_.param<double>("room_obstacle_cost_weight", room_obstacle_cost_weight_,
                       0.80);
    room_obstacle_cost_weight_ = std::max(0.0, room_obstacle_cost_weight_);
    pnh_.param<double>("room_target_blacklist_radius",
                       room_target_blacklist_radius_, 0.80);
    room_target_blacklist_radius_ = std::max(0.0, room_target_blacklist_radius_);
    pnh_.param<int>("room_entry_confirmation_cycles",
                    room_entry_confirmation_cycles_, 2);
    room_entry_confirmation_cycles_ = std::max(1, room_entry_confirmation_cycles_);
    pnh_.param<bool>("room_entry_allow_unknown_wall",
                     room_entry_allow_unknown_wall_, true);
    pnh_.param<int>("room_entry_min_width_cells", room_entry_min_width_cells_,
                    3);
    room_entry_min_width_cells_ = std::max(2, room_entry_min_width_cells_);
    pnh_.param<int>("room_entry_wall_support_cells",
                    room_entry_wall_support_cells_, 3);
    room_entry_wall_support_cells_ = std::max(0, room_entry_wall_support_cells_);
    pnh_.param<int>("room_entry_wall_support_min_occupied",
                    room_entry_wall_support_min_occupied_, 2);
    room_entry_wall_support_min_occupied_ =
        std::max(1, room_entry_wall_support_min_occupied_);
    pnh_.param<int>("room_entry_max_width_cells", room_entry_max_width_cells_,
                    7);
    room_entry_max_width_cells_ = std::max(0, room_entry_max_width_cells_);
    pnh_.param<int>("room_entry_path_fail_limit", room_entry_path_fail_limit_,
                    5);
    room_entry_path_fail_limit_ = std::max(1, room_entry_path_fail_limit_);
    int rooms_per_floor = DEFAULT_ROOMS_PER_FLOOR;
    pnh_.param<int>("rooms_per_floor", rooms_per_floor, DEFAULT_ROOMS_PER_FLOOR);
    rooms_per_floor = std::max(1, rooms_per_floor);
    expected_room_count_.assign(floor_count_, rooms_per_floor);

    // Default building geometry (scene-fixed infrastructure).
    stair_bounds_ = {-4.85, -1.65, 0.85, 6.85};
    corridor_bounds_ = {-1.1, 1.1, 7.85, 35.91};
    footprint_bounds_ = {-10.0, 10.0, 0.0, 36.0};
    lobby_bounds_ = {-10.0, 10.0, 0.0, 7.85};
    elevator_bounds_ = {1.65, 4.05, 1.25, 3.95};

    room_door_ys_.clear();
    room_entry_observations_.assign(floor_count_, std::map<EntryKey, int>());

    pnh_.param<std::string>("model_path", model_path_,
                            "/home/uf/HDPlanner_Exp_and_Nav/model/HDPlanner_Nav/policy_traced.pt");
    pnh_.param<std::string>("library_path", library_path_,
                            "/home/uf/SimEnv/devel/lib/libhdplanner_inference.so");

    double ctrl_stall = 8.0;
    pnh_.param<double>("ctrl_stall_timeout", ctrl_stall, 8.0);
    ctrl_stall_timeout_ = std::max(4.0, ctrl_stall);

    int cells = static_cast<int>(py_round(map_size_m_ / cell_size_));
    if (cells < 80 || cells > 600) {
      throw std::runtime_error("map_size_m must produce 80..600 cells");
    }
    map_cells_ = cells;
    int n = cells * cells;
    floor_beliefs_.assign(floor_count_, std::vector<uint8_t>(n, UNKNOWN));
    obs_hits_.assign(floor_count_, std::vector<int32_t>(n, 0));
    obs_free_.assign(floor_count_, std::vector<int32_t>(n, 0));

    reset_graph_state();

    map_origin_x_ = std::nullopt;
    map_origin_y_ = std::nullopt;
    exploration_roi_ = std::nullopt;

    pose_ = std::nullopt;
    home_ = std::nullopt;
    current_floor_ = 0;
    home_floor_ = 0;
    visited_by_floor_.assign(floor_count_, std::set<std::pair<double, double>>());
    visited_cells_.assign(floor_count_, std::set<std::pair<int, int>>());
    floor_complete_.assign(floor_count_, false);
    no_frontier_cycles_.assign(floor_count_, 0);
    distributed_coverage_cycles_.assign(floor_count_, 0);
    reachable_frontiers_.assign(floor_count_, 0);
    floor_coverage_.assign(floor_count_, 0.0);
    floor_distance_.assign(floor_count_, 0.0);
    last_distance_pose_ = std::nullopt;
    transition_target_floor_ = std::nullopt;
    return_after_transition_ = false;
    full_map_complete_ = false;
    current_frontier_cell_ = std::nullopt;
    frontier_progress_cell_ = std::nullopt;
    frontier_best_distance_ = std::numeric_limits<double>::infinity();
    frontier_progress_time_ = ros::Time(0);
    blacklisted_frontiers_.assign(floor_count_, std::set<std::pair<int, int>>());
    last_progress_pose_ = std::nullopt;
    last_progress_time_ = ros::Time(0);
    ctrl_last_pose_ = std::nullopt;
    ctrl_last_time_ = ros::Time(0);
    pending_known_cells_.assign(floor_count_, 0);
    latest_scan_stamp_ = ros::Time(0);
    last_map_update_ = ros::Time(0);
    active_ = false;
    mode_ = "idle";
    path_.clear();
    path_index_ = 0;
    last_plan_ = ros::Time(0);
    last_command_ = geometry_msgs::Twist();
    started_at_ = ros::Time(0);
    status_ = "waiting_for_odom";

    policy_ = std::make_unique<nav::HdplannerPolicy>(library_path_, model_path_);
    if (require_hdplanner_ && !policy_->ready) {
      throw std::runtime_error("required HDPlanner backend failed: " +
                               policy_->reason);
    }

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
        "/competition_navigation/map", 1, true);
    for (int floor = 0; floor < floor_count_; ++floor) {
      floor_map_pubs_.push_back(nh_.advertise<nav_msgs::OccupancyGrid>(
          ssprintf("/competition_navigation/map_floor_%d", floor), 1, true));
    }
    waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/competition_navigation/waypoint", 1);
    status_pub_ = nh_.advertise<std_msgs::String>(
        "/competition_navigation/status", 2);
    odom_sub_ =
        nh_.subscribe(odom_topic_, 5, &CompetitionNavigation::odom_callback,
                      this);
    scan_sub_ =
        nh_.subscribe(scan_topic_, 1, &CompetitionNavigation::scan_callback,
                      this);
    start_srv_ = nh_.advertiseService(
        "/competition_navigation/start", &CompetitionNavigation::start_callback,
        this);
    return_srv_ = nh_.advertiseService(
        "/competition_navigation/return_home",
        &CompetitionNavigation::return_home_callback, this);
    set_home_srv_ = nh_.advertiseService(
        "/competition_navigation/set_home",
        &CompetitionNavigation::set_home_callback, this);
    stop_srv_ = nh_.advertiseService(
        "/competition_navigation/stop", &CompetitionNavigation::stop_callback,
        this);
    control_timer_ = nh_.createTimer(ros::Duration(control_period_),
                                     &CompetitionNavigation::control_timer,
                                     this);
    planning_timer_ = nh_.createTimer(ros::Duration(planning_period_),
                                      &CompetitionNavigation::planning_timer,
                                      this);
    publish_timer_ =
        nh_.createTimer(ros::Duration(1.0 / map_publish_rate_),
                        &CompetitionNavigation::publish_timer, this);

    if (auto_start_) {
      ROS_WARN("auto_start enabled: controller must already be in RL mode 6");
      active_ = true;
      mode_ = "explore";
      started_at_ = ros::Time::now();
    }

    ROS_INFO(
        "competition_navigation ready: scan=%s odom=%s cmd_vel=%s map=%dx%d "
        "backend=%s",
        scan_topic_.c_str(), odom_topic_.c_str(), cmd_vel_topic_.c_str(), cells,
        cells, policy_->reason.c_str());
  }

  void shutdown() {
    publish_zero();
    policy_->close();
  }

  using EntryKey = std::pair<std::string, int>;

  void reset_graph_state() {
    floor_graphs_.clear();
    for (int i = 0; i < floor_count_; ++i) {
      floor_graphs_.emplace_back(nav::CorridorGraph(0.5));
    }
    graph_root_nodes_.assign(floor_count_, -1);
    graph_anchor_nodes_.assign(floor_count_, std::map<std::string, int>());
    graph_end_nodes_.assign(floor_count_, -1);
    graph_phase_.assign(floor_count_, "corridor_discovery");
    graph_dfs_order_.assign(floor_count_, std::vector<int>());
    graph_dfs_index_.assign(floor_count_, 0);
    graph_dfs_attempts_.assign(floor_count_, 0);
    active_room_node_ = -1;
    active_room_phase_.clear();
    active_room_entered_ = false;
    active_room_started_ = ros::Time(0);
    active_room_no_frontier_cycles_ = 0;
    active_room_target_cell_ = std::nullopt;
    active_room_frontier_blacklist_.clear();
    room_entry_path_failures_ = 0;
    last_room_plan_ = std::nullopt;
    room_entry_progress_pose_ = std::nullopt;
    room_entry_progress_time_ = ros::Time(0);
    double stall = 10.0;
    pnh_.param<double>("room_entry_stall_timeout", stall, 10.0);
    room_entry_stall_timeout_ = std::max(3.0, stall);
    room_door_ys_by_floor_.assign(floor_count_, std::vector<double>());
    room_entry_observations_.assign(floor_count_, std::map<EntryKey, int>());
  }

  // -------------------------------------------------------------------------
  // Grid helpers.
  // -------------------------------------------------------------------------
  bool inside(int x, int y) const {
    return x >= 0 && x < map_cells_ && y >= 0 && y < map_cells_;
  }
  int idx(int x, int y) const { return y * map_cells_ + x; }

  std::optional<std::pair<int, int>> cell(double px, double py,
                                          std::optional<double> ox,
                                          std::optional<double> oy) const {
    double origin_x = ox.has_value() ? *ox : map_origin_x_.value_or(0.0);
    double origin_y = oy.has_value() ? *oy : map_origin_y_.value_or(0.0);
    int x = static_cast<int>(std::lrint((px - origin_x) / cell_size_));
    int y = static_cast<int>(std::lrint((py - origin_y) / cell_size_));
    if (inside(x, y)) return std::make_pair(x, y);
    return std::nullopt;
  }

  std::optional<std::pair<int, int>> cell(const std::pair<double, double>& p)
      const {
    return cell(p.first, p.second, std::nullopt, std::nullopt);
  }

  // -------------------------------------------------------------------------
  // TF / sensor transform helpers.
  // -------------------------------------------------------------------------
  bool lookup_sensor_transform(const std::string& source_frame,
                               const ros::Time& stamp, Eigen::Vector3d& t,
                               Eigen::Quaterniond& q) const {
    ros::Time query = stamp != ros::Time(0) ? stamp : ros::Time(0);
    geometry_msgs::TransformStamped ts;
    try {
      ts = tf_buffer_.lookupTransform(map_frame_, source_frame, query);
    } catch (const tf2::ExtrapolationException&) {
      ts = tf_buffer_.lookupTransform(map_frame_, source_frame, ros::Time(0));
    }
    t = Eigen::Vector3d(ts.transform.translation.x, ts.transform.translation.y,
                        ts.transform.translation.z);
    q = Eigen::Quaterniond(ts.transform.rotation.w, ts.transform.rotation.x,
                           ts.transform.rotation.y, ts.transform.rotation.z);
    return true;
  }

  std::optional<std::vector<std::array<double, 3>>> transform_points(
      const std::string& source_frame, const ros::Time& stamp,
      const std::vector<geometry_msgs::Point32>& points, const Pose& pose)
      const {
    if (source_frame.empty() || source_frame == map_frame_ ||
        source_frame == "odom" || source_frame == "/odom") {
      std::vector<std::array<double, 3>> out;
      out.reserve(points.size());
      for (const auto& p : points) out.push_back({p.x, p.y, p.z});
      return out;
    }
    try {
      Eigen::Vector3d t;
      Eigen::Quaterniond q;
      lookup_sensor_transform(source_frame, stamp, t, q);
      Eigen::Matrix3d R = q.normalized().toRotationMatrix();
      std::vector<std::array<double, 3>> out;
      out.reserve(points.size());
      for (const auto& p : points) {
        Eigen::Vector3d v(p.x, p.y, p.z);
        Eigen::Vector3d r = R * v + t;
        out.push_back({r.x(), r.y(), r.z()});
      }
      return out;
    } catch (const tf2::TransformException&) {
      ROS_WARN_THROTTLE(5.0, "TF unavailable: %s -> %s", source_frame.c_str(),
                        map_frame_.c_str());
      return std::nullopt;
    }
  }

  std::optional<std::array<double, 3>> sensor_origin(
      const std::string& source_frame, const ros::Time& stamp,
      const Pose& pose) const {
    if (source_frame.empty() || source_frame == map_frame_ ||
        source_frame == "odom" || source_frame == "/odom") {
      return std::array<double, 3>{pose.x, pose.y, pose.z};
    }
    try {
      Eigen::Vector3d t;
      Eigen::Quaterniond q;
      lookup_sensor_transform(source_frame, stamp, t, q);
      return std::array<double, 3>{t.x(), t.y(), t.z()};
    } catch (const tf2::TransformException&) {
      std::string stripped = source_frame;
      if (!stripped.empty() && stripped[0] == '/') stripped = stripped.substr(1);
      if (stripped == "laser_livox") {
        return std::array<double, 3>{pose.x + 0.2 * std::cos(pose.yaw),
                                     pose.y + 0.2 * std::sin(pose.yaw),
                                     pose.z + 0.08};
      }
      return std::nullopt;
    }
  }

  // -------------------------------------------------------------------------
  // Callbacks.
  // -------------------------------------------------------------------------
  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    const auto& position = msg->pose.pose.position;
    const auto& q = msg->pose.pose.orientation;
    if (!(std::isfinite(position.x) && std::isfinite(position.y) &&
          std::isfinite(position.z) && std::isfinite(q.x) && std::isfinite(q.y) &&
          std::isfinite(q.z) && std::isfinite(q.w))) {
      ROS_ERROR_THROTTLE(5.0, "Odometry contains NaN/Inf");
      return;
    }
    Pose new_pose;
    new_pose.x = position.x;
    new_pose.y = position.y;
    new_pose.z = position.z;
    new_pose.yaw = yaw_from_quat(q.x, q.y, q.z, q.w);
    int previous_floor = current_floor_;
    pose_ = new_pose;
    if (!home_.has_value()) {
      home_ = new_pose;
      map_origin_x_ = new_pose.x - map_size_m_ / 2.0;
      map_origin_y_ = new_pose.y - map_size_m_ / 2.0;
      build_exploration_roi();
      last_progress_pose_ = new_pose;
      last_progress_time_ = ros::Time::now();
      ROS_INFO("home pose recorded at (%.3f, %.3f)", new_pose.x, new_pose.y);
    }
    current_floor_ = floor_from_height(position.z);
    if (current_floor_ != previous_floor) {
      ROS_INFO("floor transition observed: %d -> %d at z=%.3f", previous_floor,
               current_floor_, position.z);
    }
    if (last_distance_pose_.has_value()) {
      auto [last_floor, last_x, last_y] = last_distance_pose_.value();
      double step = std::hypot(new_pose.x - last_x, new_pose.y - last_y);
      if (last_floor == current_floor_ && step < 1.0) {
        floor_distance_[current_floor_] += step;
      }
    }
    last_distance_pose_ = std::make_tuple(current_floor_, new_pose.x, new_pose.y);
  }

  bool free_past(const std::vector<uint8_t>& belief,
                 const std::vector<std::pair<int, int>>& line, int idx) const {
    for (size_t j = idx + 1; j < line.size(); ++j) {
      int cx = line[j].first, cy = line[j].second;
      if (!inside(cx, cy)) return false;
      uint8_t v = belief[this->idx(cx, cy)];
      if (v == FREE) return true;
      if (v != OCCUPIED) return false;
    }
    return false;
  }

  bool is_self_return(const std::pair<double, double>& point,
                      const Pose& pose) const {
    double dx = point.first - pose.x;
    double dy = point.second - pose.y;
    double cos_yaw = std::cos(pose.yaw);
    double sin_yaw = std::sin(pose.yaw);
    double forward = cos_yaw * dx + sin_yaw * dy;
    double lateral = -sin_yaw * dx + cos_yaw * dy;
    return (-scan_self_filter_rear_ <= forward &&
            forward <= scan_self_filter_front_ &&
            -scan_self_filter_right_ <= lateral &&
            lateral <= scan_self_filter_left_);
  }

  void scan_callback(const sensor_msgs::PointCloud::ConstPtr& msg) {
    if (!pose_.has_value() || !map_origin_x_.has_value()) return;
    Pose pose = pose_.value();
    int floor_index = current_floor_;
    double origin_x = *map_origin_x_;
    double origin_y = *map_origin_y_;
    if (mode_ == "stairs_up" || mode_ == "stairs_down") {
      latest_scan_stamp_ = msg->header.stamp;
      last_map_update_ = ros::Time::now();
      return;
    }
    if (msg->points.empty()) return;

    int stride = std::max(
        scan_stride_,
        static_cast<int>(
            std::ceil(static_cast<double>(msg->points.size()) /
                      static_cast<double>(max_scan_points_))));
    std::vector<geometry_msgs::Point32> sampled;
    for (size_t i = 0; i < msg->points.size(); i += stride) {
      sampled.push_back(msg->points[i]);
    }
    std::optional<std::vector<std::array<double, 3>>> transformed =
        transform_points(msg->header.frame_id, msg->header.stamp, sampled, pose);
    if (!transformed.has_value()) return;

    auto robot_cell_opt = cell(std::make_pair(pose.x, pose.y));
    auto sensor_origin_opt =
        sensor_origin(msg->header.frame_id, msg->header.stamp, pose);
    if (!robot_cell_opt.has_value() || !sensor_origin_opt.has_value()) return;
    auto sensor_cell_opt = cell(std::make_pair((*sensor_origin_opt)[0],
                                               (*sensor_origin_opt)[1]));
    if (!sensor_cell_opt.has_value()) sensor_cell_opt = robot_cell_opt;
    auto robot_cell = *robot_cell_opt;
    auto sensor_cell = *sensor_cell_opt;

    std::vector<uint8_t>& belief = floor_beliefs_[floor_index];
    std::vector<int32_t>& obs_hits = obs_hits_[floor_index];
    std::vector<int32_t>& obs_free = obs_free_[floor_index];
    const std::vector<uint8_t>* roi =
        exploration_roi_.has_value() ? &(*exploration_roi_) : nullptr;
    int new_known_cells = 0;

    for (const auto& point : *transformed) {
      double px = point[0], py = point[1], pz = point[2];
      if (!(std::isfinite(px) && std::isfinite(py) && std::isfinite(pz)))
        continue;
      double distance = std::hypot(px - pose.x, py - pose.y);
      if (distance > scan_max_range_ || distance < scan_min_range_) continue;
      if (scan_self_filter_enabled_ &&
          is_self_return(std::make_pair(px, py), pose))
        continue;
      if (pz < pose.z - 0.05 || pz > pose.z + 1.0) continue;
      auto endpoint_opt = cell(px, py, origin_x, origin_y);
      if (!endpoint_opt.has_value()) continue;
      auto endpoint = *endpoint_opt;
      std::vector<std::pair<int, int>> line = bresenham(
          sensor_cell.first, sensor_cell.second, endpoint.first, endpoint.second);
      for (size_t i = 0; i + 1 < line.size(); ++i) {
        int cell_x = line[i].first, cell_y = line[i].second;
        if (!inside(cell_x, cell_y)) continue;
        int cidx = idx(cell_x, cell_y);
        if (belief[cidx] == OCCUPIED) {
          if (free_past(belief, line, static_cast<int>(i))) {
            int free = obs_free[cidx] + 1;
            obs_free[cidx] = free;
            int total = free + obs_hits[cidx];
            if (total >= obstacle_clear_min_obs_ &&
                free * 100.0 / total > obstacle_clear_percent_) {
              belief[cidx] = FREE;
              obs_free[cidx] = 0;
              obs_hits[cidx] = 0;
            }
          }
        } else {
          if (belief[cidx] == UNKNOWN &&
              (roi == nullptr || (*roi)[cidx])) {
            new_known_cells += 1;
          }
          belief[cidx] = FREE;
          obs_free[cidx] = 0;
          obs_hits[cidx] = 0;
        }
      }
      int ex = endpoint.first, ey = endpoint.second;
      if (inside(ex, ey)) {
        int eidx = idx(ex, ey);
        if (belief[eidx] == UNKNOWN && (roi == nullptr || (*roi)[eidx])) {
          new_known_cells += 1;
        }
        if (belief[eidx] == OCCUPIED) {
          obs_hits[eidx] += 1;
        } else {
          obs_hits[eidx] = 1;
          obs_free[eidx] = 0;
        }
        belief[eidx] = OCCUPIED;
      }
    }
    if (inside(robot_cell.first, robot_cell.second)) {
      int rcidx = idx(robot_cell.first, robot_cell.second);
      if (belief[rcidx] == UNKNOWN && (roi == nullptr || (*roi)[rcidx])) {
        new_known_cells += 1;
      }
      belief[rcidx] = FREE;
      obs_free[rcidx] = 0;
      obs_hits[rcidx] = 0;
    }
    pending_known_cells_[floor_index] += new_known_cells;
    if (pending_known_cells_[floor_index] >= map_progress_cell_batch_) {
      last_progress_pose_ = pose;
      last_progress_time_ = ros::Time::now();
      pending_known_cells_[floor_index] = 0;
    }
    latest_scan_stamp_ = msg->header.stamp;
    last_map_update_ = ros::Time::now();
  }

  // -------------------------------------------------------------------------
  // Services.
  // -------------------------------------------------------------------------
  bool start_callback(std_srvs::Trigger::Request&,
                      std_srvs::Trigger::Response& res) {
    if (!home_.has_value()) {
      res.success = false;
      res.message = "waiting for /Odometry_gazebo";
      return true;
    }
    active_ = true;
    mode_ = "explore";
    path_.clear();
    path_index_ = 0;
    current_frontier_cell_ = std::nullopt;
    frontier_progress_cell_ = std::nullopt;
    frontier_best_distance_ = std::numeric_limits<double>::infinity();
    frontier_progress_time_ = ros::Time::now();
    started_at_ = ros::Time::now();
    full_map_complete_ = false;
    reset_graph_state();
    return_after_transition_ = false;
    std::fill(no_frontier_cycles_.begin(), no_frontier_cycles_.end(), 0);
    std::fill(distributed_coverage_cycles_.begin(),
              distributed_coverage_cycles_.end(), 0);
    status_ = "explore_requested";
    ROS_INFO("exploration started");
    res.success = true;
    res.message = "exploration started";
    return true;
  }

  bool return_home_callback(std_srvs::Trigger::Request&,
                            std_srvs::Trigger::Response& res) {
    if (!home_.has_value()) {
      res.success = false;
      res.message = "home pose is not recorded";
      return true;
    }
    active_ = true;
    return_after_transition_ = true;
    mode_ = current_floor_ > home_floor_ ? "go_stairs_down" : "return";
    transition_target_floor_ =
        current_floor_ > home_floor_ ? std::optional<int>(current_floor_ - 1)
                                     : std::nullopt;
    path_.clear();
    path_index_ = 0;
    status_ = "return_requested";
    ROS_INFO("return-home requested");
    res.success = true;
    res.message = "return-home requested";
    return true;
  }

  bool set_home_callback(std_srvs::Trigger::Request&,
                         std_srvs::Trigger::Response& res) {
    if (!pose_.has_value()) {
      res.success = false;
      res.message = "waiting for odometry";
      return true;
    }
    home_ = pose_;
    status_ = "home_updated";
    ROS_INFO("home pose updated to (%.3f, %.3f)", home_->x, home_->y);
    res.success = true;
    res.message = "home pose updated";
    return true;
  }

  bool stop_callback(std_srvs::Trigger::Request&,
                     std_srvs::Trigger::Response& res) {
    active_ = false;
    mode_ = "idle";
    path_.clear();
    path_index_ = 0;
    status_ = "stopped";
    publish_zero();
    res.success = true;
    res.message = "navigation stopped";
    return true;
  }

  // -------------------------------------------------------------------------
  // Coverage.
  // -------------------------------------------------------------------------
  double coverage_ratio(const std::vector<uint8_t>& belief) const {
    const std::vector<uint8_t>* roi =
        exploration_roi_.has_value() ? &(*exploration_roi_) : nullptr;
    int denominator = 0;
    for (int i = 0; i < static_cast<int>(belief.size()); ++i) {
      if (roi == nullptr || (*roi)[i]) denominator++;
    }
    if (denominator == 0) return 0.0;
    int num = 0;
    for (int i = 0; i < static_cast<int>(belief.size()); ++i) {
      if (belief[i] != UNKNOWN && (roi == nullptr || (*roi)[i])) num++;
    }
    return static_cast<double>(num) / denominator;
  }

  double coverage_in_world_bounds(const std::vector<uint8_t>& belief,
                                  const Bounds& bounds) const {
    if (!map_origin_x_.has_value() || !map_origin_y_.has_value()) return 0.0;
    int x0 = std::max(
        0, static_cast<int>(
               std::floor((bounds.x_min - *map_origin_x_) / cell_size_)));
    int x1 = std::min(
        map_cells_,
        static_cast<int>(
            std::ceil((bounds.x_max - *map_origin_x_) / cell_size_)) +
            1);
    int y0 = std::max(
        0, static_cast<int>(
               std::floor((bounds.y_min - *map_origin_y_) / cell_size_)));
    int y1 = std::min(
        map_cells_,
        static_cast<int>(
            std::ceil((bounds.y_max - *map_origin_y_) / cell_size_)) +
            1);
    if (x1 <= x0 || y1 <= y0) return 0.0;
    int num = 0;
    int total = 0;
    for (int y = y0; y < y1; ++y) {
      for (int x = x0; x < x1; ++x) {
        total++;
        if (belief[idx(x, y)] != UNKNOWN) num++;
      }
    }
    if (total == 0) return 0.0;
    return static_cast<double>(num) / total;
  }

  // -------------------------------------------------------------------------
  // Graph / room helpers.
  // -------------------------------------------------------------------------
  int graph_root(int floor_index) {
    nav::CorridorGraph& graph = floor_graphs_[floor_index];
    int root_id = graph_root_nodes_[floor_index];
    if (root_id >= 0) return root_id;
    double y = corridor_bounds_.y_min;
    double x = 0.0;
    auto [id, created] = graph.add_node("entrance", floor_index, x, y, 0.0, 0.0,
                                        1.0, "corridor_entrance");
    (void)created;
    graph_root_nodes_[floor_index] = id;
    return id;
  }

  std::vector<RoomEntryCandidate> room_entry_candidates(
      const std::vector<uint8_t>& belief) const {
    if (!map_origin_x_.has_value()) return {};
    double x_min = corridor_bounds_.x_min, x_max = corridor_bounds_.x_max;
    double y_min = corridor_bounds_.y_min + 0.8, y_max = corridor_bounds_.y_max - 0.8;
    std::vector<RoomEntryCandidate> candidates;
    struct SideSpec {
      std::string side;
      double wall_x;
      double dir;
    };
    for (const auto& spec :
         {SideSpec{"left", x_min, -1.0}, SideSpec{"right", x_max, 1.0}}) {
      std::vector<std::pair<double, bool>> samples;
      for (double y = y_min; y <= y_max; y += cell_size_) {
        auto wall_cell = cell(spec.wall_x, y, *map_origin_x_, *map_origin_y_);
        auto room_cell = cell(spec.wall_x + spec.dir * 0.45, y, *map_origin_x_,
                              *map_origin_y_);
        auto corridor_cell = cell(spec.wall_x - spec.dir * 0.45, y,
                                  *map_origin_x_, *map_origin_y_);
        bool opening = false;
        if (wall_cell.has_value() && room_cell.has_value() &&
            corridor_cell.has_value()) {
          uint8_t wall_value = belief[idx(wall_cell->first, wall_cell->second)];
          bool wall_open =
              (wall_value == FREE ||
               (room_entry_allow_unknown_wall_ && wall_value == UNKNOWN));
          opening = wall_open &&
                    belief[idx(room_cell->first, room_cell->second)] == FREE &&
                    belief[idx(corridor_cell->first, corridor_cell->second)] ==
                        FREE;
        }
        samples.emplace_back(y, opening);
      }
      int start = -1;
      for (size_t index = 0; index <= samples.size(); ++index) {
        bool opening = index < samples.size() ? samples[index].second : false;
        if (opening && start < 0) start = static_cast<int>(index);
        if (!opening && start >= 0) {
          int len = static_cast<int>(index) - start;
          if (len >= room_entry_min_width_cells_) {
            bool width_ok = true;
            if (room_entry_max_width_cells_ > 0 &&
                len > room_entry_max_width_cells_)
              width_ok = false;
            bool support_ok = true;
            if (width_ok && room_entry_wall_support_cells_ > 0) {
              support_ok = false;
              int window = room_entry_wall_support_cells_;
              for (int step : {-1, 1}) {
                int base = step < 0 ? start : static_cast<int>(index) - 1;
                int occupied_count = 0;
                int checked = 0;
                for (int k = 1; k <= window; ++k) {
                  int pos = base + step * k;
                  if (pos < 0 || pos >= static_cast<int>(samples.size())) break;
                  auto wall_cell =
                      cell(spec.wall_x, samples[pos].first, *map_origin_x_,
                           *map_origin_y_);
                  checked++;
                  if (wall_cell.has_value() &&
                      belief[idx(wall_cell->first, wall_cell->second)] ==
                          OCCUPIED)
                    occupied_count++;
                }
                if (checked >= 1 &&
                    occupied_count >= room_entry_wall_support_min_occupied_) {
                  support_ok = true;
                  break;
                }
              }
            }
            if (width_ok && support_ok) {
              double center_y = 0.0;
              for (int i = start; i < static_cast<int>(index); ++i)
                center_y += samples[i].first;
              center_y /= len;
              RoomEntryCandidate c;
              c.side = spec.side;
              c.x = spec.wall_x + spec.dir * 0.60;
              c.y = center_y;
              c.yaw = (spec.side == "left") ? M_PI : 0.0;
              c.corridor_s = center_y - corridor_bounds_.y_min;
              c.confidence = std::min(1.0, len / 4.0);
              candidates.push_back(c);
            }
          }
          start = -1;
        }
      }
    }
    return candidates;
  }

  std::vector<RoomEntryCandidate> confirmed_room_entry_candidates(
      const std::vector<RoomEntryCandidate>& candidates, int floor_index) {
    if (floor_index >= static_cast<int>(room_entry_observations_.size()))
      return {};
    std::map<EntryKey, int>& previous = room_entry_observations_[floor_index];
    std::map<EntryKey, int> current;
    std::vector<RoomEntryCandidate> confirmed;
    for (const auto& candidate : candidates) {
      EntryKey key(candidate.side,
                   static_cast<int>(
                       std::lrint(candidate.y / std::max(cell_size_, 0.1))));
      int count = std::min(room_entry_confirmation_cycles_,
                           previous.count(key) ? previous[key] + 1 : 1);
      current[key] = count;
      if (count >= room_entry_confirmation_cycles_) {
        RoomEntryCandidate c = candidate;
        c.confidence = std::min(1.0, std::max(candidate.confidence,
                                              static_cast<double>(count) / 4.0));
        confirmed.push_back(c);
      }
    }
    room_entry_observations_[floor_index] = current;
    return confirmed;
  }

  bool room_progress_stalled(const Pose& pose) {
    ros::Time now = ros::Time::now();
    if (room_entry_progress_time_ == ros::Time(0) ||
        !room_entry_progress_pose_.has_value()) {
      room_entry_progress_pose_ = pose;
      room_entry_progress_time_ = now;
      return false;
    }
    Pose last = room_entry_progress_pose_.value();
    double moved = std::hypot(pose.x - last.x, pose.y - last.y);
    double turned = std::abs(
        std::remainder(pose.yaw - last.yaw, 2.0 * M_PI));
    if (moved >= 0.25 || turned >= 0.25) {
      room_entry_progress_pose_ = pose;
      room_entry_progress_time_ = now;
      return false;
    }
    return now - room_entry_progress_time_ > ros::Duration(room_entry_stall_timeout_);
  }

  void update_floor_graph(const std::vector<uint8_t>& belief, int floor_index) {
    if (floor_index >= static_cast<int>(floor_graphs_.size())) return;
    nav::CorridorGraph& graph = floor_graphs_[floor_index];
    graph_root(floor_index);
    std::vector<RoomEntryCandidate> candidates = confirmed_room_entry_candidates(
        room_entry_candidates(belief), floor_index);
    for (const auto& candidate : candidates) {
      std::string key = ssprintf("%.2f", candidate.y);
      auto it = graph_anchor_nodes_[floor_index].find(key);
      int anchor_id = (it != graph_anchor_nodes_[floor_index].end()) ? it->second
                                                                     : -1;
      if (anchor_id < 0) {
        for (const auto& kv : graph_anchor_nodes_[floor_index]) {
          const nav::GraphNode& existing = graph.nodes.at(kv.second);
          if (std::abs(existing.y - candidate.y) <= 0.75) {
            anchor_id = kv.second;
            break;
          }
        }
      }
      if (anchor_id < 0) {
        auto [id, created] = graph.add_node(
            "junction", floor_index, 0.0, candidate.y, 0.0,
            candidate.corridor_s, candidate.confidence,
            "corridor_anchor_" + key);
        (void)created;
        anchor_id = id;
        graph_anchor_nodes_[floor_index][key] = anchor_id;
      } else {
        graph_anchor_nodes_[floor_index][key] = anchor_id;
      }
      auto [entry_id, created] = graph.add_room_entry(
          floor_index, candidate.x, candidate.y, candidate.yaw,
          candidate.corridor_s, candidate.confidence, "room_entry_" + key);
      (void)created;
      graph.add_edge(anchor_id, entry_id);
    }

    std::vector<int> corridor_nodes = {graph_root(floor_index)};
    std::vector<std::pair<double, int>> junctions;
    for (const auto& kv : graph.nodes) {
      const nav::GraphNode& node = kv.second;
      if (node.node_type == "junction" && node.floor == floor_index) {
        double s = node.corridor_s.has_value()
                       ? *node.corridor_s
                       : std::numeric_limits<double>::infinity();
        junctions.emplace_back(s, node.node_id);
      }
    }
    std::sort(junctions.begin(), junctions.end());
    for (const auto& j : junctions) corridor_nodes.push_back(j.second);
    if (graph_end_nodes_[floor_index] >= 0)
      corridor_nodes.push_back(graph_end_nodes_[floor_index]);
    for (size_t i = 0; i + 1 < corridor_nodes.size(); ++i) {
      graph.add_edge(corridor_nodes[i], corridor_nodes[i + 1]);
    }
    refresh_room_door_ys(floor_index);
  }

  std::vector<double> refresh_room_door_ys(int floor_index) {
    if (floor_index >= static_cast<int>(floor_graphs_.size())) return {};
    nav::CorridorGraph& graph = floor_graphs_[floor_index];
    std::set<double> observed_ys;
    for (const auto& kv : graph.nodes) {
      const nav::GraphNode& node = kv.second;
      if (node.node_type == "room_entry" && node.floor == floor_index &&
          std::isfinite(node.y)) {
        observed_ys.insert(node.y);
      }
    }
    std::vector<double> door_ys;
    for (double y : observed_ys) {
      if (door_ys.empty() || std::abs(y - door_ys.back()) > 0.75)
        door_ys.push_back(y);
    }
    room_door_ys_by_floor_[floor_index] = door_ys;
    if (floor_index == current_floor_) room_door_ys_ = door_ys;
    return door_ys;
  }

  bool prepare_graph_dfs(int floor_index) {
    if (floor_index >= static_cast<int>(floor_graphs_.size())) return false;
    int end_id = graph_end_nodes_[floor_index];
    if (end_id < 0) return false;
    nav::CorridorGraph& graph = floor_graphs_[floor_index];
    std::vector<int> order = graph.dfs_order(end_id, floor_index);
    if (order.empty()) return false;
    graph_dfs_order_[floor_index] = order;
    graph_dfs_index_[floor_index] = 0;
    graph_dfs_attempts_[floor_index] = 0;
    graph_phase_[floor_index] = "dfs";
    status_ = ssprintf("dfs_started floor=%d nodes=%zu", floor_index,
                       order.size());
    return true;
  }

  bool plan_graph_corridor_survey(const std::vector<uint8_t>& belief,
                                  const Pose& pose, int floor_index) {
    if (floor_index >= static_cast<int>(graph_phase_.size())) return false;
    if (graph_phase_[floor_index] != "corridor_discovery") return false;
    double corridor_min_y = corridor_bounds_.y_min;
    if (pose.y < corridor_min_y - 0.5) {
      auto path = astar_path(belief, std::make_pair(pose.x, pose.y),
                             std::make_pair(0.0, corridor_min_y + 0.8), true,
                             false);
      if (!path.empty()) {
        set_path(path, "graph_main_entrance");
        return true;
      }
    }
    double end_y = corridor_bounds_.y_max - 0.8;
    if (pose.y >= end_y - 0.5) {
      nav::CorridorGraph& graph = floor_graphs_[floor_index];
      auto [end_id, created] = graph.add_node(
          "corridor_end", floor_index, 0.0, end_y, 0.0,
          end_y - corridor_min_y, 1.0, "corridor_end");
      (void)created;
      graph_end_nodes_[floor_index] = end_id;
      update_floor_graph(belief, floor_index);
      int room_count = 0;
      for (const auto& kv : graph.nodes)
        if (kv.second.node_type == "room_entry") room_count++;
      prepare_graph_dfs(floor_index);
      status_ = ssprintf("corridor_end_discovered rooms=%d dfs_nodes=%zu",
                         room_count, graph_dfs_order_[floor_index].size());
      path_.clear();
      path_index_ = 0;
      return true;
    }
    auto path = astar_path(belief, std::make_pair(pose.x, pose.y),
                           std::make_pair(0.0, end_y), false, true);
    if (path.empty()) {
      path = astar_path(belief, std::make_pair(pose.x, pose.y),
                        std::make_pair(0.0, end_y), true, true);
    }
    if (path.empty()) return false;
    set_path(path, "graph_corridor_survey");
    return true;
  }

  std::string room_side(const nav::GraphNode& node) const {
    double center_x = 0.5 * (corridor_bounds_.x_min + corridor_bounds_.x_max);
    return node.x < center_x ? "left" : "right";
  }

  std::optional<Bounds> room_bounds(const nav::GraphNode& node) const {
    double y_min = corridor_bounds_.y_min, y_max = corridor_bounds_.y_max;
    const nav::CorridorGraph& graph = floor_graphs_[node.floor];
    std::string side = room_side(node);
    std::set<double> y_values;
    for (const auto& kv : graph.nodes) {
      const nav::GraphNode& other = kv.second;
      if (other.node_type == "room_entry" && other.floor == node.floor &&
          room_side(other) == side) {
        y_values.insert(py_round2(other.y));
      }
    }
    double lower = y_min, upper = y_max;
    for (double value : y_values) {
      if (value < node.y - 0.4)
        lower = std::max(lower, value);
      else if (value > node.y + 0.4) {
        upper = std::min(upper, value);
        break;
      }
    }
    Bounds b;
    if (side == "left") {
      b.x_min = footprint_bounds_.x_min;
      b.x_max = corridor_bounds_.x_min;
    } else {
      b.x_min = corridor_bounds_.x_max;
      b.x_max = footprint_bounds_.x_max;
    }
    b.y_min = lower;
    b.y_max = upper;
    if (b.x_max <= b.x_min || b.y_max <= b.y_min) return std::nullopt;
    return b;
  }

  std::vector<uint8_t> room_mask(const Bounds& bounds) const {
    std::vector<uint8_t> mask(map_cells_ * map_cells_, 0);
    for (int y = 0; y < map_cells_; ++y) {
      double wy = *map_origin_y_ + y * cell_size_;
      if (wy < bounds.y_min || wy > bounds.y_max) continue;
      for (int x = 0; x < map_cells_; ++x) {
        double wx = *map_origin_x_ + x * cell_size_;
        if (wx >= bounds.x_min && wx <= bounds.x_max) mask[idx(x, y)] = 1;
      }
    }
    return mask;
  }

  std::optional<std::pair<double, double>> room_target(
      const nav::GraphNode& node) const {
    std::optional<Bounds> b = room_bounds(node);
    if (!b.has_value()) return std::nullopt;
    return std::make_pair(0.5 * (b->x_min + b->x_max),
                          0.5 * (b->y_min + b->y_max));
  }

  double room_coverage(const std::vector<uint8_t>& belief,
                       const nav::GraphNode& node) const {
    std::optional<Bounds> b = room_bounds(node);
    if (!b.has_value()) return 0.0;
    return coverage_in_world_bounds(belief, *b);
  }

  // ---- room frontier planning ----
  int room_information_gain(const std::vector<uint8_t>& belief,
                            const std::vector<uint8_t>& room_mask,
                            const std::pair<int, int>& candidate) const {
    int max_cells = std::max(
        1, static_cast<int>(std::ceil(room_information_range_ / cell_size_)));
    int ray_count = std::max(
        8, static_cast<int>(std::ceil(360.0 / room_information_angle_step_deg_)));
    std::set<std::pair<int, int>> visible_unknown;
    int cell_x = candidate.first, cell_y = candidate.second;
    for (int ray = 0; ray < ray_count; ++ray) {
      double angle = 2.0 * M_PI * ray / ray_count;
      int end_x = cell_x + static_cast<int>(std::lrint(max_cells * std::cos(angle)));
      int end_y = cell_y + static_cast<int>(std::lrint(max_cells * std::sin(angle)));
      for (const auto& [rx, ry] : bresenham(cell_x, cell_y, end_x, end_y)) {
        if (!inside(rx, ry) || !room_mask[idx(rx, ry)]) break;
        uint8_t v = belief[idx(rx, ry)];
        if (v == OCCUPIED) break;
        if (v == UNKNOWN) visible_unknown.insert({rx, ry});
      }
    }
    return static_cast<int>(visible_unknown.size());
  }

  void blacklist_reached_room_target(const Pose& pose) {
    if (!active_room_target_cell_.has_value()) return;
    auto target = *active_room_target_cell_;
    double tx = *map_origin_x_ + target.first * cell_size_;
    double ty = *map_origin_y_ + target.second * cell_size_;
    if (std::hypot(pose.x - tx, pose.y - ty) > 0.90) return;
    int radius = static_cast<int>(
        std::ceil(room_target_blacklist_radius_ / cell_size_));
    for (int oy = -radius; oy <= radius; ++oy) {
      for (int ox = -radius; ox <= radius; ++ox) {
        if (ox * ox + oy * oy > radius * radius) continue;
        active_room_frontier_blacklist_.insert(
            {target.first + ox, target.second + oy});
      }
    }
    active_room_target_cell_ = std::nullopt;
  }

  std::vector<std::pair<double, double>> room_frontier_path(
      const std::vector<uint8_t>& belief, const Pose& pose,
      const nav::GraphNode& node) {
    std::optional<Bounds> b = room_bounds(node);
    if (!b.has_value()) return {};
    std::vector<uint8_t> rmask = room_mask(*b);
    std::vector<uint8_t> frontier = frontier_mask_4(belief, map_cells_);
    for (int i = 0; i < map_cells_ * map_cells_; ++i)
      frontier[i] = frontier[i] && rmask[i];
    for (const auto& [cell_x, cell_y] : active_room_frontier_blacklist_) {
      if (inside(cell_x, cell_y)) frontier[idx(cell_x, cell_y)] = 0;
    }
    std::vector<uint8_t> known_free = known_free_mask(belief);
    for (int i = 0; i < map_cells_ * map_cells_; ++i)
      known_free[i] = known_free[i] && rmask[i];

    auto start_opt = cell(std::make_pair(pose.x, pose.y));
    if (!start_opt.has_value()) return {};
    auto start = *start_opt;
    known_free[idx(start.first, start.second)] = 1;
    DistanceTree tree = distance_tree(known_free, map_cells_, start);

    auto door_cell_opt = cell(node.position());
    if (door_cell_opt.has_value())
      known_free[idx(door_cell_opt->first, door_cell_opt->second)] = 1;
    DistanceTree door_tree =
        distance_tree(known_free, map_cells_, door_cell_opt);

    std::vector<std::vector<std::pair<int, int>>> clusters;
    for (auto& c : frontier_clusters(frontier, map_cells_)) {
      if (static_cast<int>(c.size()) >= room_frontier_min_cluster_cells_)
        clusters.push_back(std::move(c));
    }
    std::sort(clusters.begin(), clusters.end(),
              [](const auto& a, const auto& b) { return a.size() > b.size(); });
    std::vector<std::pair<int, int>> candidates;
    for (const auto& cluster : clusters) {
      std::vector<std::pair<int, int>> reachable;
      for (const auto& c : cluster)
        if (tree.distance[idx(c.first, c.second)] >= 2) reachable.push_back(c);
      for (auto& c : sample_cluster_cells(reachable, room_candidates_per_cluster_))
        candidates.push_back(c);
      if (static_cast<int>(candidates.size()) >= room_candidate_limit_) break;
    }
    if (static_cast<int>(candidates.size()) > room_candidate_limit_)
      candidates.resize(room_candidate_limit_);
    if (candidates.empty()) {
      RoomPlan plan;
      plan.candidate_count = 0;
      plan.cluster_count = static_cast<int>(clusters.size());
      last_room_plan_ = plan;
      return {};
    }

    std::optional<std::pair<int, int>> best_cell;
    double best_score = -std::numeric_limits<double>::infinity();
    RoomPlan best_record;
    for (const auto& [cell_x, cell_y] : candidates) {
      int path_cells = tree.distance[idx(cell_x, cell_y)];
      if (path_cells < 2) continue;
      int gain_cells = room_information_gain(belief, rmask, {cell_x, cell_y});
      double gain_area = gain_cells * cell_size_ * cell_size_;
      double path_m = path_cells * cell_size_;
      int return_cells = door_tree.distance[idx(cell_x, cell_y)];
      double return_m;
      if (return_cells >= 0)
        return_m = return_cells * cell_size_;
      else if (door_cell_opt.has_value())
        return_m = std::hypot(cell_x - door_cell_opt->first,
                              cell_y - door_cell_opt->second) *
                   cell_size_;
      else
        return_m = 0.0;
      double obstacle_penalty =
          near_obstacle(belief, cell_x, cell_y, 2) ? room_obstacle_cost_weight_
                                                   : 0.0;
      double score = gain_area - room_path_cost_weight_ * path_m -
                     room_return_cost_weight_ * return_m - obstacle_penalty;
      if (score > best_score) {
        best_score = score;
        best_cell = std::make_pair(cell_x, cell_y);
        best_record.has_gain_cells = true;
        best_record.gain_cells = gain_cells;
        best_record.path_m = path_m;
        best_record.return_m = return_m;
        best_record.score = score;
        best_record.candidate_count = static_cast<int>(candidates.size());
        best_record.cluster_count = static_cast<int>(clusters.size());
      }
    }
    if (!best_cell.has_value()) return {};

    std::vector<std::pair<int, int>> cells;
    std::optional<std::pair<int, int>> current = best_cell;
    while (current.has_value()) {
      cells.push_back(*current);
      if (*current == start) break;
      int px = tree.parent_x[idx(current->first, current->second)];
      int py = tree.parent_y[idx(current->first, current->second)];
      current = (px < 0 || py < 0) ? std::nullopt
                                   : std::optional<std::pair<int, int>>(
                                         std::make_pair(px, py));
    }
    if (cells.empty() || cells.back() != start) return {};
    std::reverse(cells.begin(), cells.end());
    active_room_target_cell_ = best_cell;
    last_room_plan_ = best_record;
    return cells_to_waypoints(cells);
  }

  std::vector<std::pair<double, double>> room_path(
      const std::vector<uint8_t>& belief, const Pose& pose,
      const nav::GraphNode& node, bool entering) const {
    std::vector<std::pair<double, double>> path;
    std::pair<double, double> corridor_pose = std::make_pair(0.0, node.y);
    std::pair<double, double> entry_pose = node.position();
    if (entering) {
      auto approach = astar_path(belief, std::make_pair(pose.x, pose.y),
                                 corridor_pose, false, true);
      if (approach.empty() &&
          std::hypot(pose.x - corridor_pose.first,
                     pose.y - corridor_pose.second) > 0.55)
        return {};
      append_path(path, approach);
      auto doorway = astar_path(belief, corridor_pose, entry_pose, true, true);
      if (doorway.empty()) return {};
      append_path(path, doorway);
      auto target = room_target(node);
      if (!target.has_value()) return {};
      auto interior =
          astar_path(belief, entry_pose, *target, true, true);
      if (interior.empty()) return {};
      append_path(path, interior);
      return path;
    }
    auto interior_exit =
        astar_path(belief, std::make_pair(pose.x, pose.y), entry_pose, false,
                   true);
    if (interior_exit.empty())
      interior_exit = astar_path(belief, std::make_pair(pose.x, pose.y),
                                 entry_pose, true, true);
    if (interior_exit.empty() &&
        std::hypot(pose.x - entry_pose.first, pose.y - entry_pose.second) <=
            0.90) {
      interior_exit = {entry_pose};
    }
    if (interior_exit.empty()) return {};
    append_path(path, interior_exit);
    auto corridor_exit =
        astar_path(belief, entry_pose, corridor_pose, false, true);
    if (corridor_exit.empty())
      corridor_exit = astar_path(belief, entry_pose, corridor_pose, true, true);
    if (corridor_exit.empty() &&
        std::hypot(pose.x - entry_pose.first, pose.y - entry_pose.second) <=
            0.90) {
      corridor_exit = {entry_pose, corridor_pose};
    }
    if (corridor_exit.empty()) return {};
    append_path(path, corridor_exit);
    return path;
  }

  bool pose_inside_room(const Pose& pose, const nav::GraphNode& node) const {
    std::optional<Bounds> b = room_bounds(node);
    if (!b.has_value()) return false;
    double margin = 0.45;
    return (b->x_min + margin <= pose.x && pose.x <= b->x_max - margin &&
            b->y_min + margin <= pose.y && pose.y <= b->y_max - margin);
  }

  bool plan_graph_room_task(const std::vector<uint8_t>& belief, const Pose& pose,
                            int floor_index) {
    if (floor_index >= static_cast<int>(graph_phase_.size())) return false;
    if (graph_phase_[floor_index] != "dfs" &&
        graph_phase_[floor_index] != "room_reverse")
      return false;
    nav::CorridorGraph& graph = floor_graphs_[floor_index];

    if (graph_phase_[floor_index] == "room_reverse") {
      if (!prepare_graph_dfs(floor_index)) {
        graph_phase_[floor_index] = "frontier_fallback";
        status_ = "room_graph_unavailable";
        return false;
      }
    }

    if (active_room_node_ < 0) {
      std::vector<int>& order = graph_dfs_order_[floor_index];
      std::set<int> known_nodes(order.begin(), order.end());
      bool has_late = false;
      for (const auto& kv : graph.nodes) {
        if (kv.second.node_type == "room_entry" &&
            !known_nodes.count(kv.first)) {
          has_late = true;
          break;
        }
      }
      if (has_late) {
        int resume_node = -1;
        int idxv = graph_dfs_index_[floor_index];
        if (idxv >= 0 && idxv < static_cast<int>(order.size()))
          resume_node = order[idxv];
        if (!prepare_graph_dfs(floor_index)) {
          graph_phase_[floor_index] = "frontier_fallback";
          status_ = "room_graph_unavailable";
          return false;
        }
        order = graph_dfs_order_[floor_index];
        if (resume_node >= 0) {
          auto it = std::find(order.begin(), order.end(), resume_node);
          graph_dfs_index_[floor_index] =
              it != order.end() ? static_cast<int>(it - order.begin()) : 0;
        }
      }

      while (graph_dfs_index_[floor_index] < static_cast<int>(order.size())) {
        int node_id = order[graph_dfs_index_[floor_index]];
        nav::GraphNode& node = graph.nodes.at(node_id);
        if (node.node_type == "room_entry") {
          if (node.completed || node.blocked) {
            graph_dfs_index_[floor_index] += 1;
            continue;
          }
          active_room_node_ = node_id;
          active_room_phase_ = "entering";
          active_room_entered_ = false;
          active_room_started_ = ros::Time::now();
          active_room_no_frontier_cycles_ = 0;
          active_room_target_cell_ = std::nullopt;
          active_room_frontier_blacklist_.clear();
          room_entry_path_failures_ = 0;
          last_room_plan_ = std::nullopt;
          room_entry_progress_pose_ = pose;
          room_entry_progress_time_ = ros::Time::now();
          graph_dfs_attempts_[floor_index] = 0;
          graph.mark_node(node_id, true, std::nullopt, std::nullopt);
          path_.clear();
          path_index_ = 0;
          status_ = ssprintf("dfs_room_selected id=%d y=%.2f", node.node_id,
                             node.y);
          break;
        }
        if (node.node_type != "entrance" && node.node_type != "junction" &&
            node.node_type != "corridor_end") {
          graph_dfs_index_[floor_index] += 1;
          continue;
        }
        double distance = std::hypot(node.x - pose.x, node.y - pose.y);
        if (distance <= 0.65) {
          graph.mark_node(node_id, true, std::nullopt, std::nullopt);
          graph_dfs_index_[floor_index] += 1;
          graph_dfs_attempts_[floor_index] = 0;
          path_.clear();
          path_index_ = 0;
          continue;
        }
        if (path_index_ < static_cast<int>(path_.size())) return true;
        auto path = astar_path(belief, std::make_pair(pose.x, pose.y),
                               node.position(), false, true);
        if (!path.empty()) {
          set_path(path, ssprintf("dfs_corridor_segment id=%d", node.node_id));
          return true;
        }
        graph_dfs_attempts_[floor_index] += 1;
        int attempts = graph_dfs_attempts_[floor_index];
        if (attempts < 3) {
          status_ = ssprintf("dfs_waiting_for_segment id=%d attempt=%d",
                             node.node_id, attempts);
          path_.clear();
          path_index_ = 0;
          return true;
        }
        graph.mark_node(node_id, std::nullopt, std::nullopt, true);
        graph_dfs_index_[floor_index] += 1;
        graph_dfs_attempts_[floor_index] = 0;
        status_ = ssprintf("dfs_segment_blocked id=%d", node.node_id);
      }
      if (active_room_node_ < 0) {
        graph_phase_[floor_index] = "frontier_fallback";
        status_ = "room_graph_complete";
        path_.clear();
        path_index_ = 0;
        return false;
      }
    }

    nav::GraphNode& node = graph.nodes.at(active_room_node_);
    double elapsed = (ros::Time::now() - active_room_started_).toSec();
    if (active_room_phase_ == "entering") {
      auto target = room_target(node);
      if (pose_inside_room(pose, node) ||
          (target.has_value() &&
           std::hypot(pose.x - target->first, pose.y - target->second) <=
               0.65)) {
        active_room_entered_ = true;
        node.entered = true;
        active_room_phase_ = "surveying";
        path_.clear();
        path_index_ = 0;
        status_ = ssprintf("room_entered id=%d", node.node_id);
        return true;
      }
      if (path_index_ < static_cast<int>(path_.size())) {
        if (room_progress_stalled(pose)) {
          graph.mark_node(active_room_node_, std::nullopt, std::nullopt, true);
          status_ = ssprintf("room_blocked_stalled id=%d", node.node_id);
          active_room_node_ = -1;
          active_room_phase_.clear();
          room_entry_progress_pose_ = std::nullopt;
          room_entry_progress_time_ = ros::Time(0);
          graph_dfs_index_[floor_index] += 1;
          return true;
        }
        return true;
      }
      auto path = room_path(belief, pose, node, true);
      if (!path.empty()) {
        room_entry_path_failures_ = 0;
        set_path(path, ssprintf("room_enter_path id=%d", node.node_id));
        return true;
      }
      room_entry_path_failures_ += 1;
      if (elapsed >= room_task_timeout_ ||
          room_entry_path_failures_ >= room_entry_path_fail_limit_) {
        graph.mark_node(active_room_node_, std::nullopt, std::nullopt, true);
        status_ = ssprintf("room_blocked id=%d entry_failures=%d", node.node_id,
                           room_entry_path_failures_);
        active_room_node_ = -1;
        active_room_phase_.clear();
        graph_dfs_index_[floor_index] += 1;
        return true;
      }
      status_ = ssprintf("room_waiting_for_entry_path id=%d failures=%d",
                         node.node_id, room_entry_path_failures_);
      path_.clear();
      path_index_ = 0;
      return true;
    }

    if (active_room_phase_ == "surveying") {
      double coverage = room_coverage(belief, node);
      node.coverage = std::max(node.coverage, coverage);
      if (active_room_entered_ && coverage > 0.05) node.interior_observation = true;
      if (active_room_entered_ && coverage >= min_room_coverage_) {
        active_room_phase_ = "returning";
        status_ = ssprintf("room_coverage_reached id=%d %.3f", node.node_id,
                           coverage);
      } else {
        if (path_index_ < static_cast<int>(path_.size())) {
          if (room_progress_stalled(pose)) {
            graph.mark_node(active_room_node_, std::nullopt, std::nullopt, true);
            status_ =
                ssprintf("room_blocked_frontier_stalled id=%d", node.node_id);
            active_room_node_ = -1;
            active_room_phase_.clear();
            room_entry_progress_pose_ = std::nullopt;
            room_entry_progress_time_ = ros::Time(0);
            graph_dfs_index_[floor_index] += 1;
            return true;
          }
          return true;
        }
        blacklist_reached_room_target(pose);
        auto path = room_frontier_path(belief, pose, node);
        if (!path.empty()) {
          active_room_no_frontier_cycles_ = 0;
          const RoomPlan& plan = last_room_plan_.value_or(RoomPlan{});
          std::string gain = plan.has_gain_cells
                                 ? ssprintf("%d", plan.gain_cells)
                                 : std::string("?");
          set_path(path, ssprintf(
                             "room_frontier id=%d coverage=%.3f gain=%s "
                             "path=%.2f return=%.2f score=%.2f",
                             node.node_id, coverage, gain.c_str(), plan.path_m,
                             plan.return_m, plan.score));
          return true;
        }
        active_room_no_frontier_cycles_ += 1;
        status_ = ssprintf("room_waiting_for_frontier id=%d %.3f", node.node_id,
                           coverage);
        path_.clear();
        path_index_ = 0;
        if (elapsed < room_task_timeout_) return true;
        graph.mark_node(active_room_node_, std::nullopt, std::nullopt, true);
        status_ = ssprintf("room_blocked_no_frontier id=%d", node.node_id);
        active_room_node_ = -1;
        active_room_phase_.clear();
        graph_dfs_index_[floor_index] += 1;
        return true;
      }
    }

    if (active_room_phase_ == "returning") {
      bool in_corridor =
          (corridor_bounds_.x_min - 0.35 <= pose.x &&
           pose.x <= corridor_bounds_.x_max + 0.35 &&
           corridor_bounds_.y_min - 0.35 <= pose.y &&
           pose.y <= corridor_bounds_.y_max + 0.35);
      if (in_corridor) {
        graph.mark_node(active_room_node_, std::nullopt, true, std::nullopt);
        node.coverage = std::max(node.coverage, room_coverage(belief, node));
        status_ = ssprintf("room_complete id=%d", node.node_id);
        active_room_node_ = -1;
        active_room_phase_.clear();
        graph_dfs_index_[floor_index] += 1;
        graph_dfs_attempts_[floor_index] = 0;
        path_.clear();
        path_index_ = 0;
        return true;
      }
      if (path_index_ < static_cast<int>(path_.size())) {
        if (room_progress_stalled(pose)) {
          graph.mark_node(active_room_node_, std::nullopt, std::nullopt, true);
          status_ = ssprintf("room_return_blocked_stalled id=%d", node.node_id);
          active_room_node_ = -1;
          active_room_phase_.clear();
          room_entry_progress_pose_ = std::nullopt;
          room_entry_progress_time_ = ros::Time(0);
          graph_dfs_index_[floor_index] += 1;
          return true;
        }
        return true;
      }
      auto path = room_path(belief, pose, node, false);
      if (!path.empty()) {
        set_path(path, ssprintf("room_return_path id=%d", node.node_id));
        return true;
      }
      if (elapsed >= room_task_timeout_) {
        graph.mark_node(active_room_node_, std::nullopt, std::nullopt, true);
        status_ = ssprintf("room_return_blocked id=%d", node.node_id);
        active_room_node_ = -1;
        active_room_phase_.clear();
        graph_dfs_index_[floor_index] += 1;
        return true;
      }
      status_ = ssprintf("room_waiting_for_return_path id=%d", node.node_id);
      path_.clear();
      path_index_ = 0;
      return true;
    }
    return false;
  }

  // ---- stairs ----
  std::optional<std::tuple<double, double, double, double, double>>
  stair_geometry() const {
    double width = stair_bounds_.x_max - stair_bounds_.x_min;
    double x_left = stair_bounds_.x_min + 0.26 * width;
    double x_right = stair_bounds_.x_max - 0.26 * width;
    double x_exit = stair_bounds_.x_max + 0.5 * width;
    double y_entry = stair_bounds_.y_min + 0.65;
    double y_turn = stair_bounds_.y_max - 1.65;
    return std::make_tuple(x_left, x_right, y_entry, y_turn, x_exit);
  }

  std::optional<std::pair<double, double>> stair_entry(
      const std::string& direction) const {
    auto g = stair_geometry();
    if (!g.has_value()) return std::nullopt;
    return std::make_pair(std::get<4>(*g), std::get<2>(*g));
  }

  std::vector<std::pair<double, double>> stair_route(
      const std::string& direction) const {
    auto g = stair_geometry();
    if (!g.has_value()) return {};
    double x_left = std::get<0>(*g), x_right = std::get<1>(*g);
    double y_entry = std::get<2>(*g), y_turn = std::get<3>(*g);
    double x_exit = std::get<4>(*g);
    if (direction == "up") {
      return {{x_left, y_entry},
              {x_left, y_turn},
              {x_right, y_turn},
              {x_right, y_entry},
              {x_exit, y_entry}};
    }
    return {{x_right, y_entry},
            {x_right, y_turn},
            {x_left, y_turn},
            {x_left, y_entry},
            {x_exit, y_entry}};
  }

  bool begin_stair_transition(const std::string& direction) {
    auto route = stair_route(direction);
    if (route.empty()) {
      status_ = "stair_geometry_unavailable";
      return false;
    }
    mode_ = "stairs_" + direction;
    path_ = route;
    path_index_ = 0;
    status_ = ssprintf("stairs_%s_started target_floor=%d", direction.c_str(),
                       transition_target_floor_.value_or(-1));
    current_frontier_cell_ = std::nullopt;
    ROS_INFO("starting stair transition %s: floor %d -> %d", direction.c_str(),
             current_floor_, transition_target_floor_.value_or(-1));
    return true;
  }

  bool finish_stair_transition() {
    if (!transition_target_floor_.has_value() ||
        current_floor_ != transition_target_floor_.value())
      return false;
    if (!path_.empty() && pose_.has_value()) {
      double finish_tol = std::max(waypoint_tolerance(mode_), 0.40);
      if (std::hypot(pose_->x - path_.back().first,
                     pose_->y - path_.back().second) > finish_tol)
        return false;
    }
    ROS_INFO("stair transition reached floor %d", current_floor_);
    path_.clear();
    path_index_ = 0;
    no_frontier_cycles_[current_floor_] = 0;
    distributed_coverage_cycles_[current_floor_] = 0;
    if (return_after_transition_) {
      if (current_floor_ > home_floor_) {
        transition_target_floor_ = current_floor_ - 1;
        mode_ = "go_stairs_down";
        status_ = "continue_return_via_stairs";
      } else {
        transition_target_floor_ = std::nullopt;
        mode_ = "return";
        status_ = "return_on_home_floor";
      }
    } else {
      transition_target_floor_ = std::nullopt;
      mode_ = "explore";
      status_ = ssprintf("explore_floor_%d", current_floor_);
    }
    return true;
  }

  void mark_floor_complete(int floor_index) {
    floor_complete_[floor_index] = true;
    ROS_INFO("floor %d exploration complete: coverage=%.3f distance=%.2f",
             floor_index, floor_coverage_[floor_index],
             floor_distance_[floor_index]);
    if (floor_index < floor_count_ - 1) {
      transition_target_floor_ = floor_index + 1;
      return_after_transition_ = false;
      mode_ = "go_stairs_up";
      path_.clear();
      path_index_ = 0;
      status_ = ssprintf("floor_%d_complete_go_stairs", floor_index);
      return;
    }
    full_map_complete_ = true;
    for (bool f : floor_complete_)
      if (!f) full_map_complete_ = false;
    return_after_transition_ = true;
    if (current_floor_ > home_floor_) {
      transition_target_floor_ = current_floor_ - 1;
      mode_ = "go_stairs_down";
      status_ = "full_map_complete_return_via_stairs";
    } else {
      mode_ = "return";
      status_ = "full_map_complete_return";
    }
    path_.clear();
    path_index_ = 0;
  }

  std::vector<std::pair<double, double>> safe_stair_entry_path(
      const Pose& pose) {
    auto entry_opt = stair_entry("up");
    if (!entry_opt.has_value()) return {};
    double x_min = corridor_bounds_.x_min, x_max = corridor_bounds_.x_max;
    double y_min = corridor_bounds_.y_min, y_max = corridor_bounds_.y_max;
    double corridor_x = clampd(0.0, x_min + 0.45, x_max - 0.45);
    auto entry = *entry_opt;
    if (pose.y <= y_min) return {entry};
    std::vector<double> door_ys = refresh_room_door_ys(current_floor_);
    if (door_ys.empty()) return {};
    double door_y = *std::min_element(
        door_ys.begin(), door_ys.end(),
        [&](double a, double b) { return std::abs(a - pose.y) < std::abs(b - pose.y); });
    std::vector<std::pair<double, double>> path;
    double wp_tol = waypoint_tolerance("go_stairs_up");
    if (pose.x > x_max + 0.25) {
      double route_x = pose.x;
      double room_approach_x =
          std::abs(pose.y - door_y) >= wp_tol ? x_max + 0.80 : pose.x;
      if (std::abs(pose.x - room_approach_x) > 0.45) {
        path.emplace_back(room_approach_x, pose.y);
        route_x = room_approach_x;
      }
      if (std::abs(pose.y - door_y) < wp_tol) {
        double offset = pose.y <= door_y ? 0.45 : -0.45;
        path.emplace_back(route_x, door_y + offset);
      }
      path.emplace_back(route_x, door_y);
      path.emplace_back(x_max - 0.45, door_y);
    } else if (pose.x < x_min - 0.25) {
      double route_x = pose.x;
      double room_approach_x =
          std::abs(pose.y - door_y) >= wp_tol ? x_min - 0.80 : pose.x;
      if (std::abs(pose.x - room_approach_x) > 0.45) {
        path.emplace_back(room_approach_x, pose.y);
        route_x = room_approach_x;
      }
      if (std::abs(pose.y - door_y) < wp_tol) {
        double offset = pose.y <= door_y ? 0.45 : -0.45;
        path.emplace_back(route_x, door_y + offset);
      }
      path.emplace_back(route_x, door_y);
      path.emplace_back(x_min + 0.45, door_y);
    } else {
      path.emplace_back(clampd(pose.x, x_min + 0.45, x_max - 0.45),
                        clampd(pose.y, y_min + 0.45, y_max - 0.45));
    }
    path.emplace_back(corridor_x, y_min - 0.45);
    path.emplace_back(entry);
    return path;
  }

  void plan_to_stairs(const std::vector<uint8_t>& belief, const Pose& pose,
                      const std::string& direction) {
    auto entry_opt = stair_entry(direction);
    if (!entry_opt.has_value()) {
      status_ = "stair_geometry_unavailable";
      return;
    }
    auto entry = *entry_opt;
    if (std::hypot(pose.x - entry.first, pose.y - entry.second) <= 0.55) {
      begin_stair_transition(direction);
      return;
    }
    auto safe_path = safe_stair_entry_path(pose);
    if (!safe_path.empty()) {
      set_path(safe_path, "go_stairs_" + direction + "_door_route");
      return;
    }
    auto path = astar_path(belief, std::make_pair(pose.x, pose.y), entry, false,
                           true);
    if (path.empty()) {
      status_ = "stair_entry_path_unavailable_" + direction;
      ROS_WARN_THROTTLE(5.0, "known-free path to stair entry is unavailable");
      return;
    }
    set_path(path, "go_stairs_" + direction + "_planned");
  }

  void build_exploration_roi() {
    int n = map_cells_ * map_cells_;
    std::vector<uint8_t> roi(n, 1);
    double margin = 0.35;
    double x_min = footprint_bounds_.x_min - margin;
    double x_max = footprint_bounds_.x_max + margin;
    double y_min = footprint_bounds_.y_min - margin;
    double y_max = footprint_bounds_.y_max + margin;
    double entrance_x = 0.5 * (footprint_bounds_.x_min + footprint_bounds_.x_max);
    double corridor_x_min = std::min(entrance_x, home_->x) - 1.25;
    double corridor_x_max = std::max(entrance_x, home_->x) + 1.25;
    double corridor_y_min = std::min(home_->y, footprint_bounds_.y_min) - 0.5;
    double corridor_y_max = std::max(home_->y, footprint_bounds_.y_min) + 0.5;
    for (int y = 0; y < map_cells_; ++y) {
      double wy = *map_origin_y_ + y * cell_size_;
      for (int x = 0; x < map_cells_; ++x) {
        double wx = *map_origin_x_ + x * cell_size_;
        bool inside_building =
            (x_min <= wx && wx <= x_max && y_min <= wy && wy <= y_max);
        bool inside_entrance =
            (corridor_x_min <= wx && wx <= corridor_x_max &&
             corridor_y_min <= wy && wy <= corridor_y_max);
        if (inside_building || inside_entrance) roi[idx(x, y)] = 1;
        else roi[idx(x, y)] = 0;
      }
    }
    // Clear core bounds (stairs / elevator) from the ROI.  Python uses
    // ``y_bounds = lobby_bounds or bounds`` for the vertical range, so the
    // cleared region spans the full lobby height for each core's x-range.
    std::vector<Bounds> cores = {stair_bounds_, elevator_bounds_};
    double core_margin = 0.20;
    double cy0 = lobby_bounds_.y_min - core_margin;
    double cy1 = lobby_bounds_.y_max + core_margin;
    for (const auto& cb : cores) {
      double cx0 = cb.x_min - core_margin, cx1 = cb.x_max + core_margin;
      for (int y = 0; y < map_cells_; ++y) {
        double wy = *map_origin_y_ + y * cell_size_;
        if (wy < cy0 || wy > cy1) continue;
        for (int x = 0; x < map_cells_; ++x) {
          double wx = *map_origin_x_ + x * cell_size_;
          if (wx >= cx0 && wx <= cx1) roi[idx(x, y)] = 0;
        }
      }
    }
    exploration_roi_ = roi;
  }

  int floor_from_height(double z_value) const {
    if (!home_.has_value() || floor_height_ <= 0.0) return 0;
    double rel = (z_value - home_->z) / floor_height_;
    return static_cast<int>(
        clampd(static_cast<double>(std::lrint(rel)), 0.0, floor_count_ - 1.0));
  }

  // ---- masks / planning ----
  std::vector<uint8_t> known_free_mask(const std::vector<uint8_t>& belief) const {
    int n = map_cells_;
    std::vector<uint8_t> free(n * n, 0), occupied(n * n, 0);
    for (int i = 0; i < n * n; ++i) {
      free[i] = belief[i] == FREE;
      occupied[i] = belief[i] == OCCUPIED;
    }
    int free_radius = std::max(2, static_cast<int>(std::ceil(0.8 / cell_size_)));
    int obstacle_radius =
        std::max(1, static_cast<int>(std::ceil(0.4 / cell_size_)));
    std::vector<uint8_t> observed_free = dilate_mask(free, n, free_radius);
    std::vector<uint8_t> blocked = dilate_mask(occupied, n, obstacle_radius);
    std::vector<uint8_t> traversable(n * n, 0);
    for (int i = 0; i < n * n; ++i) traversable[i] = observed_free[i] && !blocked[i];

    int off1 = std::max(1, static_cast<int>(py_round(0.4 / cell_size_)));
    int off2 = std::max(1, static_cast<int>(py_round(0.8 / cell_size_)));
    std::vector<int> offsets;
    if (off1 == off2) offsets = {off1};
    else offsets = {std::min(off1, off2), std::max(off1, off2)};
    std::vector<uint8_t> up(n * n, 0), down(n * n, 0), left(n * n, 0),
        right(n * n, 0);
    for (int offset : offsets) {
      for (int y = offset; y < n; ++y)
        for (int x = 0; x < n; ++x)
          up[y * n + x] = up[y * n + x] || occupied[(y - offset) * n + x];
      for (int y = 0; y < n - offset; ++y)
        for (int x = 0; x < n; ++x)
          down[y * n + x] = down[y * n + x] || occupied[(y + offset) * n + x];
      for (int y = 0; y < n; ++y)
        for (int x = offset; x < n; ++x)
          left[y * n + x] = left[y * n + x] || occupied[y * n + x - offset];
      for (int y = 0; y < n; ++y)
        for (int x = 0; x < n - offset; ++x)
          right[y * n + x] = right[y * n + x] || occupied[y * n + x + offset];
    }
    std::vector<uint8_t> doorway_seed(n * n, 0);
    for (int i = 0; i < n * n; ++i) {
      doorway_seed[i] = free[i] && ((up[i] && down[i]) || (left[i] && right[i]));
    }
    int doorway_radius = std::max(2, static_cast<int>(std::ceil(0.8 / cell_size_)));
    std::vector<uint8_t> doorway_clearance =
        dilate_mask(doorway_seed, n, doorway_radius);
    for (int i = 0; i < n * n; ++i)
      traversable[i] = traversable[i] || (doorway_clearance[i] && free[i]);

    // Main entrance: restore positively-observed free cells.
    double entrance_x = 0.5 * (footprint_bounds_.x_min + footprint_bounds_.x_max);
    double entrance_y = footprint_bounds_.y_min;
    auto entrance_cell_opt = cell(std::make_pair(entrance_x, entrance_y));
    if (entrance_cell_opt.has_value()) {
      int radius_x = std::max(1, static_cast<int>(std::ceil(0.45 / cell_size_)));
      int radius_y = std::max(1, static_cast<int>(std::ceil(0.80 / cell_size_)));
      int cxx = entrance_cell_opt->first, cyy = entrance_cell_opt->second;
      int x0 = std::max(0, cxx - radius_x), x1 = std::min(n, cxx + radius_x + 1);
      int y0 = std::max(0, cyy - radius_y), y1 = std::min(n, cyy + radius_y + 1);
      for (int y = y0; y < y1; ++y)
        for (int x = x0; x < x1; ++x)
          if (belief[idx(x, y)] == FREE) traversable[idx(x, y)] = 1;
    }
    return traversable;
  }

  std::vector<uint8_t> frontier_mask_of(const std::vector<uint8_t>& belief) const {
    return frontier_mask_4(belief, map_cells_);
  }

  std::vector<uint8_t> exploration_frontier_mask(
      const std::vector<uint8_t>& belief) const {
    if (!exploration_roi_.has_value()) return frontier_mask_4(belief, map_cells_);
    std::vector<uint8_t> bounded = belief;
    for (int i = 0; i < map_cells_ * map_cells_; ++i)
      if (!(*exploration_roi_)[i]) bounded[i] = OCCUPIED;
    return frontier_mask_4(bounded, map_cells_);
  }

  std::vector<std::pair<double, double>> astar_path(
      const std::vector<uint8_t>& belief,
      const std::pair<double, double>& start,
      const std::pair<double, double>& goal, bool allow_unknown,
      bool restrict_roi) const {
    auto start_cell_opt = cell(start);
    auto goal_cell_opt = cell(goal);
    if (!start_cell_opt.has_value() || !goal_cell_opt.has_value()) return {};
    auto start_cell = *start_cell_opt, goal_cell = *goal_cell_opt;
    int n = map_cells_;
    std::vector<uint8_t> traversable(n * n, 0);
    if (allow_unknown) {
      std::vector<uint8_t> occupied(n * n, 0);
      for (int i = 0; i < n * n; ++i) occupied[i] = belief[i] == OCCUPIED;
      std::vector<uint8_t> blocked = dilate_mask(occupied, n, robot_radius_cells_);
      for (int i = 0; i < n * n; ++i) traversable[i] = !blocked[i];
    } else {
      traversable = known_free_mask(belief);
    }
    if (restrict_roi && exploration_roi_.has_value()) {
      for (int i = 0; i < n * n; ++i)
        traversable[i] = traversable[i] && (*exploration_roi_)[i];
    }
    traversable[idx(start_cell.first, start_cell.second)] = 1;
    traversable[idx(goal_cell.first, goal_cell.second)] = 1;

    using Node = std::pair<int, int>;  // (x, y)
    std::vector<double> g(n * n, std::numeric_limits<double>::infinity());
    std::vector<int32_t> parent(n * n, -1);
    std::priority_queue<std::tuple<double, int, int>,
                        std::vector<std::tuple<double, int, int>>,
                        std::greater<std::tuple<double, int, int>>>
        heap;
    g[idx(start_cell.first, start_cell.second)] = 0.0;
    heap.emplace(0.0, start_cell.first, start_cell.second);
    int visited_count = 0;
    while (!heap.empty() && visited_count < 30000) {
      auto [_, cx, cy] = heap.top();
      heap.pop();
      visited_count++;
      if (cx == goal_cell.first && cy == goal_cell.second) {
        std::vector<std::pair<int, int>> cells;
        int cur = idx(cx, cy);
        while (cur >= 0) {
          cells.emplace_back(cur % n, cur / n);
          cur = parent[cur];
        }
        std::reverse(cells.begin(), cells.end());
        return cells_to_waypoints(cells);
      }
      for (auto [dx, dy] : {std::pair{1, 0}, std::pair{-1, 0}, std::pair{0, 1},
                            std::pair{0, -1}}) {
        int nx = cx + dx, ny = cy + dy;
        if (!inside(nx, ny) || !traversable[idx(nx, ny)]) continue;
        double new_g = g[idx(cx, cy)] + 1.0;
        if (new_g < g[idx(nx, ny)]) {
          g[idx(nx, ny)] = new_g;
          double h = std::hypot(nx - goal_cell.first, ny - goal_cell.second);
          heap.emplace(new_g + h, nx, ny);
          parent[idx(nx, ny)] = idx(cx, cy);
        }
      }
    }
    return {};
  }

  bool near_obstacle(const std::vector<uint8_t>& belief, int x, int y,
                     int radius) const {
    int x0 = std::max(0, x - radius), x1 = std::min(map_cells_, x + radius + 1);
    int y0 = std::max(0, y - radius), y1 = std::min(map_cells_, y + radius + 1);
    for (int yy = y0; yy < y1; ++yy)
      for (int xx = x0; xx < x1; ++xx)
        if (belief[idx(xx, yy)] == OCCUPIED) return true;
    return false;
  }

  std::vector<std::pair<double, double>> cells_to_waypoints(
      const std::vector<std::pair<int, int>>& cells) const {
    if (cells.empty()) return {};
    std::vector<std::pair<double, double>> waypoints;
    std::optional<std::pair<double, double>> last;
    for (size_t index = 0; index < cells.size(); ++index) {
      if (index == 0) continue;
      bool turn = false;
      if (index > 0 && index < cells.size() - 1) {
        auto prev_dx = cells[index].first - cells[index - 1].first;
        auto prev_dy = cells[index].second - cells[index - 1].second;
        auto next_dx = cells[index + 1].first - cells[index].first;
        auto next_dy = cells[index + 1].second - cells[index].second;
        turn = std::make_pair(prev_dx, prev_dy) != std::make_pair(next_dx, next_dy);
      }
      if (index != 1 && index != cells.size() - 1 && index % 3 != 0 && !turn)
        continue;
      std::pair<double, double> point = {
          *map_origin_x_ + cells[index].first * cell_size_,
          *map_origin_y_ + cells[index].second * cell_size_};
      double dist_last =
          last.has_value()
              ? std::hypot(point.first - last->first, point.second - last->second)
              : std::numeric_limits<double>::infinity();
      if (dist_last >= 0.5 || index == 1 || index == cells.size() - 1 || turn) {
        if (last.has_value() && dist_last < 1e-6) continue;
        waypoints.push_back(point);
        last = point;
      }
    }
    return waypoints;
  }

  bool set_path(const std::vector<std::pair<double, double>>& path,
                const std::string& status) {
    if (!active_) return false;
    path_ = path;
    path_index_ = 0;
    last_plan_ = ros::Time::now();
    status_ = status;
    if (pose_.has_value()) {
      visited_by_floor_[current_floor_].insert(
          {py_round1(pose_->x), py_round1(pose_->y)});
      if (map_origin_x_.has_value()) {
        auto c = cell(std::make_pair(pose_->x, pose_->y));
        if (c.has_value()) visited_cells_[current_floor_].insert(*c);
      }
    }
    return true;
  }

  std::vector<uint8_t> visited_neighborhood_mask(int floor_index) const {
    std::vector<uint8_t> mask(map_cells_ * map_cells_, 0);
    int radius = frontier_revisit_radius_cells_;
    if (radius <= 0) return mask;
    for (const auto& [cx, cy] : visited_cells_[floor_index]) {
      int x0 = std::max(0, cx - radius), x1 = std::min(map_cells_, cx + radius + 1);
      int y0 = std::max(0, cy - radius), y1 = std::min(map_cells_, cy + radius + 1);
      for (int y = y0; y < y1; ++y)
        for (int x = x0; x < x1; ++x) mask[idx(x, y)] = 1;
    }
    return mask;
  }

  struct FrontierPlan {
    std::vector<std::pair<double, double>> waypoints;
    std::optional<std::pair<double, double>> target;
    int reachable_count = 0;
  };

  FrontierPlan reachable_frontier_plan(const std::vector<uint8_t>& belief,
                                       const Pose& pose, int floor_index) {
    FrontierPlan result;
    int n = map_cells_;
    std::vector<uint8_t> frontier = exploration_frontier_mask(belief);
    std::vector<uint8_t> traversable = known_free_mask(belief);
    if (exploration_roi_.has_value())
      for (int i = 0; i < n * n; ++i) traversable[i] = traversable[i] && (*exploration_roi_)[i];
    for (const auto& [cx, cy] : blacklisted_frontiers_[floor_index])
      if (inside(cx, cy)) frontier[idx(cx, cy)] = 0;

    auto start_opt = cell(std::make_pair(pose.x, pose.y));
    if (!start_opt.has_value()) return result;
    auto start = *start_opt;
    traversable[idx(start.first, start.second)] = 1;
    std::vector<int32_t> distance(n * n, -1);
    std::vector<int16_t> parent_x(n * n, -1), parent_y(n * n, -1);
    std::deque<std::pair<int, int>> queue;
    distance[idx(start.first, start.second)] = 0;
    queue.emplace_back(start.first, start.second);
    while (!queue.empty()) {
      auto [cx, cy] = queue.front();
      queue.pop_front();
      int nd = distance[idx(cx, cy)] + 1;
      for (auto [dx, dy] : {std::pair{1, 0}, std::pair{-1, 0}, std::pair{0, 1},
                            std::pair{0, -1}}) {
        int nx = cx + dx, ny = cy + dy;
        if (!inside(nx, ny) || !traversable[idx(nx, ny)] ||
            distance[idx(nx, ny)] >= 0)
          continue;
        distance[idx(nx, ny)] = nd;
        parent_x[idx(nx, ny)] = static_cast<int16_t>(cx);
        parent_y[idx(nx, ny)] = static_cast<int16_t>(cy);
        queue.emplace_back(nx, ny);
      }
    }
    std::vector<std::pair<int, int>> candidates;
    for (int y = 0; y < n; ++y)
      for (int x = 0; x < n; ++x)
        if (frontier[idx(x, y)] && distance[idx(x, y)] >= 2)
          candidates.emplace_back(x, y);
    if (candidates.empty()) {
      result.reachable_count = 0;
      return result;
    }
    std::vector<uint8_t> unknown(n * n, 0);
    for (int i = 0; i < n * n; ++i) unknown[i] = belief[i] == UNKNOWN;

    std::optional<std::pair<int, int>> best_cell;
    if (current_frontier_cell_.has_value()) {
      int cx = current_frontier_cell_->first, cy = current_frontier_cell_->second;
      if (inside(cx, cy) && frontier[idx(cx, cy)] && distance[idx(cx, cy)] >= 2)
        best_cell = std::make_pair(cx, cy);
    }
    if (!best_cell.has_value()) {
      double best_score = -std::numeric_limits<double>::infinity();
      std::vector<uint8_t> visited_near = visited_neighborhood_mask(floor_index);
      for (const auto& [cx, cy] : candidates) {
        if (visited_near[idx(cx, cy)]) continue;
        int radius = 5;
        int y0 = std::max(0, cy - radius), y1 = std::min(n, cy + radius + 1);
        int x0 = std::max(0, cx - radius), x1 = std::min(n, cx + radius + 1);
        int info_gain = 0;
        for (int yy = y0; yy < y1; ++yy)
          for (int xx = x0; xx < x1; ++xx)
            if (unknown[idx(xx, yy)]) info_gain++;
        double score = info_gain + 0.4 * distance[idx(cx, cy)];
        if (score > best_score) {
          best_score = score;
          best_cell = std::make_pair(cx, cy);
        }
      }
    }
    std::vector<std::pair<int, int>> cells;
    std::optional<std::pair<int, int>> current = best_cell;
    while (current.has_value()) {
      cells.push_back(*current);
      if (*current == start) break;
      int px = parent_x[idx(current->first, current->second)];
      int py = parent_y[idx(current->first, current->second)];
      current = (px < 0 || py < 0) ? std::nullopt
                                   : std::optional<std::pair<int, int>>(
                                         std::make_pair(px, py));
    }
    if (cells.empty() || cells.back() != start) {
      result.reachable_count = static_cast<int>(candidates.size());
      return result;
    }
    std::reverse(cells.begin(), cells.end());
    result.waypoints = cells_to_waypoints(cells);
    result.target = std::make_pair(*map_origin_x_ + best_cell->first * cell_size_,
                                   *map_origin_y_ + best_cell->second * cell_size_);
    result.reachable_count = static_cast<int>(candidates.size());
    return result;
  }

  // ---- HDPlanner graph build ----
  bool line_is_free(const std::vector<uint8_t>& traversable,
                    const std::pair<double, double>& start,
                    const std::pair<double, double>& end) const {
    auto sc = cell(start), ec = cell(end);
    if (!sc.has_value() || !ec.has_value()) return false;
    for (const auto& [x, y] : bresenham(sc->first, sc->second, ec->first, ec->second)) {
      if (!inside(x, y) || !traversable[idx(x, y)]) return false;
    }
    return true;
  }

  int node_utility(const std::vector<uint8_t>& traversable,
                   const std::pair<double, double>& node,
                   const std::vector<std::pair<double, double>>& frontier_points)
      const {
    if (frontier_points.empty()) return 0;
    int visible = 0;
    for (const auto& point : frontier_points) {
      double d = std::hypot(point.first - node.first, point.second - node.second);
      if (d >= 4.0) continue;
      if (line_is_free(traversable, node, point)) {
        visible++;
        if (visible > 2) return 1;
      }
    }
    return 0;
  }

  std::vector<int64_t> sparsify_centers(
      const std::vector<uint8_t>& traversable,
      const std::vector<std::pair<double, double>>& nodes,
      const std::vector<int>& candidates, int target_index) const {
    if (candidates.empty()) return {};
    auto target = nodes[target_index];
    std::vector<int> ordered = candidates;
    std::sort(ordered.begin(), ordered.end(), [&](int a, int b) {
      double da = std::hypot(nodes[a].first - target.first,
                             nodes[a].second - target.second);
      double db = std::hypot(nodes[b].first - target.first,
                             nodes[b].second - target.second);
      return da < db;
    });
    std::vector<int64_t> selected;
    for (int index : ordered) {
      bool covered = false;
      for (int center : selected) {
        double d = std::hypot(nodes[index].first - nodes[center].first,
                              nodes[index].second - nodes[center].second);
        if (d <= 20.0 && line_is_free(traversable, nodes[index], nodes[center])) {
          covered = true;
          break;
        }
      }
      if (!covered) selected.push_back(index);
      if (static_cast<int64_t>(selected.size()) >= nav::HdplannerPolicy::K_SIZE)
        break;
    }
    return selected;
  }

  struct GraphData {
    std::vector<std::pair<double, double>> nodes;
    std::vector<int> utility;
    std::vector<int> visited;
    int current_index = 0;
    int target_index = 0;
    std::vector<int64_t> neighbors;
    std::vector<int64_t> centers;
    std::vector<std::vector<int64_t>> adjacency;
  };

  GraphData build_graph(const std::vector<uint8_t>& belief, const Pose& pose,
                        const std::pair<double, double>& target,
                        int floor_index,
                        const std::vector<std::pair<double, double>>& frontier_path) {
    GraphData data;
    double spacing = 4.0;
    double origin_x = *map_origin_x_, origin_y = *map_origin_y_;
    double center_x = origin_x + py_round((pose.x - origin_x) / spacing) * spacing;
    double center_y = origin_y + py_round((pose.y - origin_y) / spacing) * spacing;
    std::pair<double, double> current = {pose.x, pose.y};
    data.nodes.push_back({pose.x, pose.y});
    std::vector<uint8_t> traversable = known_free_mask(belief);
    if (exploration_roi_.has_value())
      for (int i = 0; i < map_cells_ * map_cells_; ++i)
        traversable[i] = traversable[i] && (*exploration_roi_)[i];
    for (int ix = -7; ix <= 7; ++ix) {
      for (int iy = -7; iy <= 7; ++iy) {
        std::pair<double, double> node = {center_x + ix * spacing,
                                          center_y + iy * spacing};
        auto c = cell(node);
        if (!c.has_value() || !traversable[idx(c->first, c->second)]) continue;
        if (std::hypot(node.first - current.first, node.second - current.second) <
            0.8)
          continue;
        bool dup = false;
        for (const auto& existing : data.nodes)
          if (std::hypot(node.first - existing.first,
                         node.second - existing.second) <= 0.2)
            dup = true;
        if (!dup) data.nodes.push_back({node.first, node.second});
      }
    }
    int current_index = 0;
    std::optional<std::pair<double, double>> guide_node;
    for (size_t i = 1; i < frontier_path.size(); ++i) {
      auto candidate = frontier_path[i];
      double d = std::hypot(candidate.first - current.first,
                            candidate.second - current.second);
      if (d > 3.5) break;
      if (d >= 0.8 && line_is_free(traversable, current, candidate)) {
        guide_node = candidate;
      }
    }
    if (guide_node.has_value()) {
      bool dup = false;
      for (const auto& existing : data.nodes)
        if (std::hypot(guide_node->first - existing.first,
                       guide_node->second - existing.second) <= 0.2)
          dup = true;
      if (!dup) data.nodes.push_back({guide_node->first, guide_node->second});
    }
    std::pair<double, double> target_node = target;
    int target_index = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < data.nodes.size(); ++i) {
      double d = std::hypot(data.nodes[i].first - target_node.first,
                            data.nodes[i].second - target_node.second);
      if (d < best_d) {
        best_d = d;
        target_index = static_cast<int>(i);
      }
    }
    if (std::hypot(data.nodes[target_index].first - target_node.first,
                   data.nodes[target_index].second - target_node.second) >
        0.2) {
      data.nodes.push_back({target_node.first, target_node.second});
      target_index = static_cast<int>(data.nodes.size()) - 1;
    }
    std::vector<uint8_t> frontier = exploration_frontier_mask(belief);
    std::vector<std::pair<double, double>> frontier_points;
    for (int y = 0; y < map_cells_; ++y)
      for (int x = 0; x < map_cells_; ++x)
        if (frontier[idx(x, y)])
          frontier_points.emplace_back(origin_x + x * cell_size_,
                                       origin_y + y * cell_size_);

    for (const auto& node : data.nodes) {
      double kx = py_round1(node.first), ky = py_round1(node.second);
      bool was_visited = false;
      for (const auto& old : visited_by_floor_[floor_index])
        if (std::hypot(kx - old.first, ky - old.second) <= 2.0) was_visited = true;
      data.visited.push_back(was_visited ? 1 : 0);
      data.utility.push_back(was_visited ? 0
                                         : node_utility(traversable, node,
                                                        frontier_points));
    }
    int num_nodes = static_cast<int>(data.nodes.size());
    data.adjacency.assign(num_nodes, std::vector<int64_t>());
    for (int i = 0; i < num_nodes; ++i) {
      std::vector<std::pair<double, int>> adj;
      for (int j = 0; j < num_nodes; ++j) {
        if (j == i) continue;
        double dx = std::abs(data.nodes[i].first - data.nodes[j].first);
        double dy = std::abs(data.nodes[i].second - data.nodes[j].second);
        if (dx <= 2.01 * spacing && dy <= 2.01 * spacing &&
            line_is_free(traversable, data.nodes[i], data.nodes[j])) {
          adj.emplace_back(std::hypot(data.nodes[j].first - data.nodes[i].first,
                                      data.nodes[j].second - data.nodes[i].second),
                           j);
        }
      }
      std::sort(adj.begin(), adj.end());
      for (const auto& [d, j] : adj) {
        data.adjacency[i].push_back(j);
        if (static_cast<int64_t>(data.adjacency[i].size()) >=
            nav::HdplannerPolicy::K_SIZE - 1)
          break;
      }
    }
    data.neighbors = data.adjacency[current_index];
    std::vector<int> cand;
    for (int i = 0; i < num_nodes; ++i)
      if (data.utility[i] > 0 && i != current_index) cand.push_back(i);
    data.centers = sparsify_centers(traversable, data.nodes, cand, target_index);
    if (target_index != current_index &&
        std::find(data.centers.begin(), data.centers.end(), target_index) ==
            data.centers.end())
      data.centers.push_back(target_index);
    if (static_cast<int64_t>(data.centers.size()) > nav::HdplannerPolicy::K_SIZE)
      data.centers.resize(nav::HdplannerPolicy::K_SIZE);
    for (int64_t center : data.centers) {
      if (center != target_index) {
        if (std::find(data.adjacency[target_index].begin(),
                      data.adjacency[target_index].end(),
                      center) == data.adjacency[target_index].end())
          data.adjacency[target_index].push_back(center);
        if (std::find(data.adjacency[center].begin(), data.adjacency[center].end(),
                      static_cast<int64_t>(target_index)) ==
            data.adjacency[center].end())
          data.adjacency[center].push_back(target_index);
      }
    }
    data.current_index = current_index;
    data.target_index = target_index;
    return data;
  }

  // -------------------------------------------------------------------------
  // Timers.
  // -------------------------------------------------------------------------
  void planning_timer(const ros::TimerEvent&) {
    if (!pose_.has_value() || !active_) return;
    if (auto_return_after_sec_ > 0.0 && started_at_ != ros::Time(0)) {
      double elapsed = (ros::Time::now() - started_at_).toSec();
      if (elapsed >= auto_return_after_sec_ && mode_ == "explore") {
        return_after_transition_ = true;
        if (current_floor_ > home_floor_) {
          transition_target_floor_ = current_floor_ - 1;
          mode_ = "go_stairs_down";
        } else {
          mode_ = "return";
        }
        path_.clear();
        path_index_ = 0;
        status_ = "automatic_return_timeout";
      }
    }
    Pose pose = *pose_;
    std::string mode = mode_;
    int floor_index = current_floor_;
    std::vector<uint8_t> belief = floor_beliefs_[floor_index];
    bool map_ready = last_map_update_ != ros::Time(0);

    try {
      double coverage = coverage_ratio(belief);
      floor_coverage_[floor_index] = coverage;
      if (mode != "return" && !map_ready) {
        status_ = "waiting_for_first_scan";
        ROS_WARN_THROTTLE(5.0, "waiting for the first sensor map before planning");
        return;
      }
      update_floor_graph(belief, floor_index);
      if (mode == "stairs_up" || mode == "stairs_down") {
        finish_stair_transition();
        return;
      }
      if (mode == "go_stairs_up" || mode == "go_stairs_down") {
        std::string direction = mode.back() == 'p' ? "up" : "down";
        plan_to_stairs(belief, pose, direction);
        return;
      }
      if (mode == "return") {
        if (!home_.has_value()) return;
        if (floor_index > home_floor_) {
          return_after_transition_ = true;
          transition_target_floor_ = floor_index - 1;
          mode_ = "go_stairs_down";
          path_.clear();
          path_index_ = 0;
          status_ = "return_requires_floor_transition";
          return;
        }
        auto goal = std::make_pair(home_->x, home_->y);
        auto new_path = astar_path(belief, std::make_pair(pose.x, pose.y), goal,
                                   allow_unknown_return_, false);
        if (new_path.empty()) {
          status_ = "return_path_unavailable";
          ROS_WARN_THROTTLE(5.0, "return path unavailable; holding zero velocity");
          return;
        }
        set_path(new_path, "return_planned");
        return;
      }
      if (mode != "explore") return;

      if (plan_graph_corridor_survey(belief, pose, floor_index)) return;
      if (plan_graph_room_task(belief, pose, floor_index)) return;

      ros::Time now = ros::Time::now();
      if (distributed_coverage_complete(belief, coverage, floor_index)) {
        distributed_coverage_cycles_[floor_index] += 1;
        int cycles = distributed_coverage_cycles_[floor_index];
        status_ = ssprintf("survey_distributed_coverage cycles=%d", cycles);
        if (cycles >= completion_cycles_required_) mark_floor_complete(floor_index);
        return;
      }
      distributed_coverage_cycles_[floor_index] = 0;

      if (current_frontier_cell_.has_value() && last_progress_time_ != ros::Time(0) &&
          now - last_progress_time_ > ros::Duration(30.0) &&
          frontier_progress_time_ != ros::Time(0) &&
          now - frontier_progress_time_ > ros::Duration(10.0)) {
        int fx = current_frontier_cell_->first, fy = current_frontier_cell_->second;
        for (int dx = -2; dx <= 2; ++dx)
          for (int dy = -2; dy <= 2; ++dy)
            blacklisted_frontiers_[floor_index].insert({fx + dx, fy + dy});
        current_frontier_cell_ = std::nullopt;
        frontier_progress_cell_ = std::nullopt;
        frontier_best_distance_ = std::numeric_limits<double>::infinity();
        frontier_progress_time_ = now;
        path_.clear();
        path_index_ = 0;
        pending_known_cells_[floor_index] = 0;
        last_progress_pose_ = pose;
        last_progress_time_ = now;
        status_ = "stalled_frontier_blacklisted";
        ROS_WARN("stalled exploration frontier blacklisted on floor %d", floor_index);
      }

      FrontierPlan fp = reachable_frontier_plan(belief, pose, floor_index);
      reachable_frontiers_[floor_index] = fp.reachable_count;
      if (fp.waypoints.empty() || !fp.target.has_value()) {
        current_frontier_cell_ = std::nullopt;
        frontier_progress_cell_ = std::nullopt;
        frontier_best_distance_ = std::numeric_limits<double>::infinity();
        if (graph_floor_ready(floor_index) && coverage >= min_floor_coverage_) {
          no_frontier_cycles_[floor_index] += 1;
          int cycles = no_frontier_cycles_[floor_index];
          status_ = ssprintf("survey_no_frontier cycles=%d", cycles);
          if (cycles >= no_frontier_cycles_required_) mark_floor_complete(floor_index);
        } else {
          no_frontier_cycles_[floor_index] = 0;
          status_ = ssprintf("survey_coverage_incomplete %.3f<%.3f", coverage,
                             min_floor_coverage_);
        }
        return;
      }

      auto target_cell_opt = cell(*fp.target);
      double target_distance =
          std::hypot(fp.target->first - pose.x, fp.target->second - pose.y);
      no_frontier_cycles_[floor_index] = 0;
      current_frontier_cell_ = target_cell_opt;
      if (target_cell_opt != frontier_progress_cell_) {
        frontier_progress_cell_ = target_cell_opt;
        frontier_best_distance_ = target_distance;
        frontier_progress_time_ = now;
      } else if (target_distance <= frontier_best_distance_ - 0.40) {
        frontier_best_distance_ = target_distance;
        frontier_progress_time_ = now;
      }

      if (!policy_->ready) {
        if (!fp.waypoints.empty()) {
          set_path(fp.waypoints, "frontier_astar_fallback");
          return;
        }
      }

      GraphData gd = build_graph(belief, pose, *fp.target, floor_index,
                                 fp.waypoints);
      if (static_cast<int>(gd.nodes.size()) < 2 || gd.neighbors.empty() ||
          gd.centers.empty() || gd.target_index == gd.current_index) {
        set_path(fp.waypoints, "hdplanner_graph_recovery");
        return;
      }
      int64_t selected_index =
          policy_->select(gd.nodes, gd.utility, gd.visited, gd.current_index,
                          gd.target_index, gd.neighbors, gd.centers,
                          gd.adjacency);
      if (selected_index < 0 || selected_index == gd.current_index) {
        set_path(fp.waypoints, "hdplanner_action_recovery");
        return;
      }
      std::string plan_status = "hdplanner_policy_selected";
      auto goal = gd.nodes[selected_index];
      double cur_to_target =
          std::hypot(gd.nodes[gd.current_index].first - fp.target->first,
                     gd.nodes[gd.current_index].second - fp.target->second);
      double sel_to_target = std::hypot(goal.first - fp.target->first,
                                        goal.second - fp.target->second);
      if (sel_to_target > cur_to_target - 0.20) {
        set_path(fp.waypoints,
                 ssprintf("hdplanner_nonprogress_recovery inference=%d",
                          policy_->inference_count));
        return;
      }
      auto new_path = astar_path(belief, std::make_pair(pose.x, pose.y), goal,
                                 false, true);
      if (new_path.empty()) {
        set_path(fp.waypoints, "hdplanner_selected_path_recovery");
        return;
      }
      set_path(new_path,
               ssprintf("%s inference=%d center=%ld logp=%.5f",
                        plan_status.c_str(), policy_->inference_count,
                        static_cast<long>(policy_->last_selected_center),
                        policy_->last_action_logp));
    } catch (const std::exception& exc) {
      ROS_ERROR_THROTTLE(5.0, "planning exception: %s", exc.what());
      status_ = "planning_exception";
      path_.clear();
      path_index_ = 0;
    }
  }

  bool distributed_coverage_complete(const std::vector<uint8_t>& belief,
                                     double coverage, int floor_index) const {
    if (!graph_floor_ready(floor_index)) return false;
    if (coverage < completion_min_floor_coverage_ ||
        floor_distance_[floor_index] < completion_min_floor_distance_)
      return false;
    double x_mid = 0.5 * (footprint_bounds_.x_min + footprint_bounds_.x_max);
    double y_min = footprint_bounds_.y_min;
    double y_step = (footprint_bounds_.y_max - y_min) / 3.0;
    for (int row = 0; row < 3; ++row) {
      double sy0 = y_min + row * y_step, sy1 = y_min + (row + 1) * y_step;
      Bounds left{footprint_bounds_.x_min, x_mid, sy0, sy1};
      Bounds right{x_mid, footprint_bounds_.x_max, sy0, sy1};
      if (coverage_in_world_bounds(belief, left) < completion_min_sector_coverage_ ||
          coverage_in_world_bounds(belief, right) < completion_min_sector_coverage_)
        return false;
    }
    return true;
  }

  bool graph_floor_ready(int floor_index) const {
    if (floor_index >= static_cast<int>(floor_graphs_.size())) return false;
    if (active_room_node_ >= 0) return false;
    const nav::CorridorGraph& graph = floor_graphs_[floor_index];
    int rooms = 0;
    for (const auto& kv : graph.nodes)
      if (kv.second.node_type == "room_entry") rooms++;
    int expected = floor_index < static_cast<int>(expected_room_count_.size())
                       ? expected_room_count_[floor_index]
                       : 0;
    if (expected && rooms < expected) return false;
    if (rooms == 0) return false;
    for (const auto& kv : graph.nodes)
      if (kv.second.node_type == "room_entry" &&
          !(kv.second.completed || kv.second.blocked))
        return false;
    return true;
  }

  std::string room_task_status() const {
    std::string records;
    for (int floor_index = 0; floor_index < static_cast<int>(floor_graphs_.size());
         ++floor_index) {
      const nav::CorridorGraph& graph = floor_graphs_[floor_index];
      std::vector<int> ids;
      for (const auto& kv : graph.nodes)
        if (kv.second.node_type == "room_entry") ids.push_back(kv.first);
      std::sort(ids.begin(), ids.end());
      for (int node_id : ids) {
        const nav::GraphNode& node = graph.nodes.at(node_id);
        std::string side = node.x < 0.0 ? "L" : "R";
        if (!records.empty()) records += ",";
        records += ssprintf("%d:%d:%s:%.2f:%d:%d:%d:%d:%.3f", floor_index,
                            node.node_id, side.c_str(), node.y,
                            node.entered ? 1 : 0, node.interior_observation ? 1 : 0,
                            node.completed ? 1 : 0, node.blocked ? 1 : 0,
                            node.coverage);
      }
    }
    return records.empty() ? "none" : records;
  }

  static double waypoint_tolerance(const std::string& mode) {
    if (mode == "stairs_up" || mode == "stairs_down") return 0.25;
    return 0.35;
  }

  static double stair_speed_cap(const std::string& mode, int path_index,
                                double stair_speed, double landing_speed,
                                double max_linear, bool flat_use_max_linear) {
    if (mode != "stairs_up" && mode != "stairs_down") return max_linear;
    if (path_index == 1 || path_index == 3) return stair_speed;
    if (path_index == 2) return landing_speed;
    if ((path_index == 0 || path_index == 4) && !flat_use_max_linear)
      return stair_speed;
    return max_linear;
  }

  static std::pair<double, double> tracking_waypoint(
      const std::string& mode, int path_index, const Pose& pose,
      const std::pair<double, double>& waypoint, double lookahead) {
    if (mode != "stairs_up" && mode != "stairs_down") return waypoint;
    if (path_index != 1 && path_index != 3) return waypoint;
    double delta_y = waypoint.second - pose.y;
    if (std::abs(delta_y) <= lookahead) return waypoint;
    return {waypoint.first, pose.y + std::copysign(lookahead, delta_y)};
  }

  static std::optional<double> stair_cruise_command(
      const std::string& mode, int path_index, double body_x, double body_y,
      double heading_error, double waypoint_distance, double stair_speed,
      double brake_distance, double heading_tolerance,
      double lateral_tolerance) {
    if (mode != "stairs_up" && mode != "stairs_down") return std::nullopt;
    if (path_index != 1 && path_index != 3) return std::nullopt;
    if (waypoint_distance <= brake_distance) return std::nullopt;
    if (body_x <= 0.0) return std::nullopt;
    if (std::abs(body_y) > lateral_tolerance) return std::nullopt;
    if (std::abs(heading_error) > heading_tolerance) return std::nullopt;
    return std::copysign(std::abs(stair_speed), body_x);
  }

  void control_timer(const ros::TimerEvent&) {
    geometry_msgs::Twist command;
    double home_tolerance = 0.35;
    Pose pose;
    bool active;
    std::vector<std::pair<double, double>> path;
    int index;
    std::string mode;
    std::string status;
    if (pose_.has_value()) pose = *pose_;
    else { cmd_pub_.publish(command); return; }
    active = active_;
    path = path_;
    index = path_index_;
    mode = mode_;
    status = status_;
    double wp_tol = waypoint_tolerance(mode);
    if (!active) {
      cmd_pub_.publish(command);
      return;
    }
    if (index < static_cast<int>(path.size()) && mode != "stairs_up" &&
        mode != "stairs_down") {
      double moved = 0.0;
      if (ctrl_last_pose_.has_value())
        moved = std::hypot(pose.x - ctrl_last_pose_->x, pose.y - ctrl_last_pose_->y);
      ros::Time now = ros::Time::now();
      if (ctrl_last_time_ == ros::Time(0) || moved >= 0.15) {
        ctrl_last_pose_ = pose;
        ctrl_last_time_ = now;
      } else if (now - ctrl_last_time_ > ros::Duration(ctrl_stall_timeout_)) {
        path_.clear();
        path_index_ = 0;
        status_ = "path_stalled_replan";
        ctrl_last_pose_ = pose;
        ctrl_last_time_ = now;
        ROS_WARN_THROTTLE(5.0, "path progress stalled; dropping path for re-plan");
      }
    } else {
      ctrl_last_pose_ = pose;
      ctrl_last_time_ = ros::Time(0);
    }
    if (ros::Time::now() - last_map_update_ > ros::Duration(5.0) && mode != "return") {
      cmd_pub_.publish(command);
      ROS_WARN_THROTTLE(5.0, "scan map is stale; holding zero velocity");
      return;
    }
    if (mode == "return") {
      if (home_.has_value() &&
          std::hypot(pose.x - home_->x, pose.y - home_->y) <= home_tolerance) {
        active_ = false;
        mode_ = "idle";
        status_ = ssprintf("returned_home full_map=%s",
                           full_map_complete_ ? "True" : "False");
        cmd_pub_.publish(command);
        ROS_INFO("return-home completed");
        return;
      }
    }
    while (index < static_cast<int>(path.size()) &&
           std::hypot(path[index].first - pose.x, path[index].second - pose.y) <
               wp_tol)
      index++;
    if (index >= static_cast<int>(path.size())) {
      if (mode == "stairs_up" || mode == "stairs_down") {
        path_index_ = static_cast<int>(path.size());
        cmd_pub_.publish(command);
        return;
      }
      if (mode == "explore" && status.rfind("survey_", 0) == 0) {
        command.angular.z = std::min(0.8, max_yaw_rate_);
        last_command_ = command;
        cmd_pub_.publish(command);
        return;
      }
      if (mode == "return") {
        double home_distance = std::numeric_limits<double>::infinity();
        if (home_.has_value())
          home_distance = std::hypot(pose.x - home_->x, pose.y - home_->y);
        if (home_distance > home_tolerance) {
          status_ = "return_waiting_for_path";
          cmd_pub_.publish(command);
          return;
        }
        active_ = false;
        mode_ = "idle";
        status_ = ssprintf("returned_home full_map=%s",
                           full_map_complete_ ? "True" : "False");
        cmd_pub_.publish(command);
        ROS_INFO("return-home completed");
        return;
      }
      path_index_ = 0;
      cmd_pub_.publish(command);
      return;
    }

    std::pair<double, double> waypoint = path[index];
    std::pair<double, double> twp = tracking_waypoint(
        mode, index, pose, waypoint, stair_lookahead_);
    double dx = twp.first - pose.x;
    double dy = twp.second - pose.y;
    double body_x = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy;
    double body_y = -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
    double target_yaw = std::atan2(dy, dx);
    double heading_error = target_yaw - pose.yaw;
    heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));
    command.linear.x = clampd(linear_gain_ * body_x, -max_linear_, max_linear_);
    command.linear.y = clampd(0.55 * body_y, -max_lateral_, max_lateral_);
    if (std::abs(heading_error) > 1.0) {
      command.linear.x *= 0.2;
      command.linear.y *= 0.2;
    }
    command.angular.z = clampd(0.9 * heading_error, -max_yaw_rate_, max_yaw_rate_);
    if (mode == "stairs_up" || mode == "stairs_down") {
      double waypoint_distance = std::hypot(waypoint.first - pose.x,
                                            waypoint.second - pose.y);
      std::optional<double> cruise = stair_cruise_command(
          mode, index, body_x, body_y, heading_error, waypoint_distance,
          stair_speed_, stair_brake_distance_, stair_cruise_heading_tolerance_,
          stair_cruise_lateral_tolerance_);
      if (cruise.has_value()) command.linear.x = *cruise;
      if (index != 1 && index != 3 &&
          std::abs(heading_error) > stair_turn_heading_threshold_) {
        command.linear.x = 0.0;
        command.linear.y = 0.0;
      }
      double speed_cap = stair_speed_cap(mode, index, stair_speed_,
                                         stair_landing_speed_, max_linear_,
                                         stair_flat_use_max_linear_);
      command.linear.x = clampd(command.linear.x, -speed_cap, speed_cap);
      command.linear.y = clampd(command.linear.y, -stair_lateral_limit_,
                                stair_lateral_limit_);
      double yaw_limit = std::min(max_yaw_rate_, stair_yaw_rate_);
      command.angular.z = clampd(command.angular.z, -yaw_limit, yaw_limit);
    }
    if (!(std::isfinite(command.linear.x) && std::isfinite(command.linear.y) &&
          std::isfinite(command.angular.z))) {
      ROS_ERROR("computed cmd_vel contains NaN/Inf; sending zero");
      command = geometry_msgs::Twist();
    }
    path_index_ = index;
    last_command_ = command;
    cmd_pub_.publish(command);
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position.x = waypoint.first;
    pose_msg.pose.position.y = waypoint.second;
    pose_msg.pose.orientation.w = 1.0;
    waypoint_pub_.publish(pose_msg);
  }

  void publish_zero() { cmd_pub_.publish(geometry_msgs::Twist()); }

  nav_msgs::OccupancyGrid occupancy_message(const std::vector<uint8_t>& belief,
                                            int floor_index) const {
    nav_msgs::OccupancyGrid message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = map_frame_;
    message.info.resolution = cell_size_;
    message.info.width = map_cells_;
    message.info.height = map_cells_;
    message.info.origin.position.x = *map_origin_x_;
    message.info.origin.position.y = *map_origin_y_;
    if (home_.has_value())
      message.info.origin.position.z = home_->z + floor_index * floor_height_;
    message.info.origin.orientation.w = 1.0;
    message.data.assign(map_cells_ * map_cells_, -1);
    for (int i = 0; i < map_cells_ * map_cells_; ++i) {
      if (belief[i] == FREE) message.data[i] = 0;
      else if (belief[i] == OCCUPIED) message.data[i] = 100;
    }
    return message;
  }

  void publish_timer(const ros::TimerEvent&) {
    std::vector<double> coverage;
    for (const auto& belief : floor_beliefs_) coverage.push_back(coverage_ratio(belief));
    floor_coverage_ = coverage;
    std::string graph_rooms;
    for (const auto& graph : floor_graphs_) {
      int count = 0;
      for (const auto& kv : graph.nodes)
        if (kv.second.node_type == "room_entry") count++;
      if (!graph_rooms.empty()) graph_rooms += ",";
      graph_rooms += std::to_string(count);
    }
    std::string room_tasks = room_task_status();
    std::string complete;
    for (bool f : floor_complete_) complete += (complete.empty() ? "" : ",") + std::string(f ? "1" : "0");
    std::string cov;
    for (double c : coverage) {
      if (!cov.empty()) cov += ",";
      cov += ssprintf("%.3f", c);
    }
    std::string frontiers;
    for (int r : reachable_frontiers_) {
      if (!frontiers.empty()) frontiers += ",";
      frontiers += std::to_string(r);
    }
    std::string dist;
    for (double d : floor_distance_) {
      if (!dist.empty()) dist += ",";
      dist += ssprintf("%.1f", d);
    }
    std::string pose_str = pose_.has_value()
                               ? ssprintf("(%g, %g, %g, %g)", pose_->x, pose_->y,
                                          pose_->z, pose_->yaw)
                               : std::string("None");
    std::string home_str = home_.has_value()
                               ? ssprintf("(%g, %g, %g, %g)", home_->x, home_->y,
                                          home_->z, home_->yaw)
                               : std::string("None");
    std::string status =
        ssprintf("%s mode=%s active=%s policy=%s inferences=%d floor=%d/%d "
                 "full_map=%s complete=%s coverage=%s frontiers=%s "
                 "graph_rooms=%s distance=%s pose=%s home=%s path=%d/%d "
                 "room_tasks=%s",
                 status_.c_str(), mode_.c_str(), active_ ? "True" : "False",
                 policy_->reason.c_str(), policy_->inference_count,
                 current_floor_, floor_count_,
                 full_map_complete_ ? "True" : "False", complete.c_str(),
                 cov.c_str(), frontiers.c_str(), graph_rooms.c_str(),
                 dist.c_str(), pose_str.c_str(), home_str.c_str(), path_index_,
                 static_cast<int>(path_.size()), room_tasks.c_str());

    std_msgs::String msg;
    msg.data = status;
    status_pub_.publish(msg);
    if (!map_origin_x_.has_value()) return;
    for (int floor_index = 0; floor_index < floor_count_; ++floor_index) {
      nav_msgs::OccupancyGrid message =
          occupancy_message(floor_beliefs_[floor_index], floor_index);
      floor_map_pubs_[floor_index].publish(message);
      if (floor_index == current_floor_) map_pub_.publish(message);
    }
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Parameters.
  std::string map_frame_, scan_topic_, odom_topic_, cmd_vel_topic_;
  double cell_size_ = 0.4, map_size_m_ = 80.0, planning_period_ = 2.0,
         control_period_ = 0.1, map_publish_rate_ = 5.0;
  double frontier_revisit_radius_ = 0.8;
  int frontier_revisit_radius_cells_ = 0;
  int scan_stride_ = 8, max_scan_points_ = 1800;
  double scan_min_range_ = 2.0, scan_max_range_ = 22.0;
  bool scan_self_filter_enabled_ = true;
  double scan_self_filter_front_ = 0.75, scan_self_filter_rear_ = 0.60,
         scan_self_filter_left_ = 0.55, scan_self_filter_right_ = 0.55;
  int map_progress_cell_batch_ = 8;
  double obstacle_clear_percent_ = 80.0;
  int obstacle_clear_min_obs_ = 5;
  double max_linear_ = 0.5, linear_gain_ = 1.0, max_lateral_ = 0.15,
         max_yaw_rate_ = 1.2;
  double robot_radius_ = 0.20;
  int robot_radius_cells_ = 0;
  bool allow_unknown_return_ = false, auto_start_ = false;
  double auto_return_after_sec_ = 0.0;
  bool require_hdplanner_ = true;
  int floor_count_ = 3;
  double floor_height_ = 2.6;
  double min_floor_coverage_ = 0.45, completion_min_floor_coverage_ = 0.70,
         completion_min_sector_coverage_ = 0.30,
         completion_min_floor_distance_ = 35.0;
  int completion_cycles_required_ = 3, no_frontier_cycles_required_ = 6;
  double stair_speed_ = 0.80, stair_landing_speed_ = 0.40, stair_yaw_rate_ = 1.0;
  bool stair_flat_use_max_linear_ = true;
  double stair_lookahead_ = 0.30, stair_brake_distance_ = 0.35,
         stair_lateral_limit_ = 0.08, stair_cruise_heading_tolerance_ = 0.70,
         stair_cruise_lateral_tolerance_ = 0.22,
         stair_turn_heading_threshold_ = 0.60;
  double min_room_coverage_ = 0.70, room_task_timeout_ = 120.0;
  int room_frontier_min_cluster_cells_ = 3, room_candidates_per_cluster_ = 2,
      room_candidate_limit_ = 10;
  double room_information_range_ = 6.0, room_information_angle_step_deg_ = 10.0;
  double room_path_cost_weight_ = 0.45, room_return_cost_weight_ = 0.20,
         room_obstacle_cost_weight_ = 0.80, room_target_blacklist_radius_ = 0.80;
  int room_entry_confirmation_cycles_ = 2;
  bool room_entry_allow_unknown_wall_ = true;
  int room_entry_min_width_cells_ = 3, room_entry_wall_support_cells_ = 3,
      room_entry_wall_support_min_occupied_ = 2, room_entry_max_width_cells_ = 7,
      room_entry_path_fail_limit_ = 5;
  int rooms_per_floor_ = 4;
  Bounds stair_bounds_, corridor_bounds_, footprint_bounds_, lobby_bounds_,
      elevator_bounds_;
  std::vector<double> room_door_ys_;
  std::vector<std::map<EntryKey, int>> room_entry_observations_;
  std::vector<int> expected_room_count_;
  std::string model_path_, library_path_;
  double ctrl_stall_timeout_ = 8.0;

  // State.
  int map_cells_ = 0;
  std::vector<std::vector<uint8_t>> floor_beliefs_;
  std::vector<std::vector<int32_t>> obs_hits_;
  std::vector<std::vector<int32_t>> obs_free_;
  std::optional<double> map_origin_x_, map_origin_y_;
  std::optional<std::vector<uint8_t>> exploration_roi_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  std::optional<Pose> pose_, home_;
  int current_floor_ = 0, home_floor_ = 0;
  std::vector<std::set<std::pair<double, double>>> visited_by_floor_;
  std::vector<std::set<std::pair<int, int>>> visited_cells_;
  std::vector<bool> floor_complete_;
  std::vector<int> no_frontier_cycles_, distributed_coverage_cycles_,
      reachable_frontiers_;
  std::vector<double> floor_coverage_, floor_distance_;
  std::optional<std::tuple<int, double, double>> last_distance_pose_;
  std::optional<int> transition_target_floor_;
  bool return_after_transition_ = false, full_map_complete_ = false;
  std::optional<std::pair<int, int>> current_frontier_cell_,
      frontier_progress_cell_;
  double frontier_best_distance_ = std::numeric_limits<double>::infinity();
  ros::Time frontier_progress_time_{0};
  std::vector<std::set<std::pair<int, int>>> blacklisted_frontiers_;
  std::optional<Pose> last_progress_pose_;
  ros::Time last_progress_time_{0};
  std::optional<Pose> ctrl_last_pose_;
  ros::Time ctrl_last_time_{0};
  std::vector<int> pending_known_cells_;
  ros::Time latest_scan_stamp_{0}, last_map_update_{0};
  bool active_ = false;
  std::string mode_ = "idle", status_ = "waiting_for_odom";
  std::vector<std::pair<double, double>> path_;
  int path_index_ = 0;
  ros::Time last_plan_{0};
  geometry_msgs::Twist last_command_;
  ros::Time started_at_{0};

  // Graph state.
  std::vector<nav::CorridorGraph> floor_graphs_;
  std::vector<int> graph_root_nodes_, graph_end_nodes_, graph_dfs_index_,
      graph_dfs_attempts_;
  std::vector<std::map<std::string, int>> graph_anchor_nodes_;
  std::vector<std::string> graph_phase_;
  std::vector<std::vector<int>> graph_dfs_order_;
  int active_room_node_ = -1;
  std::string active_room_phase_;
  bool active_room_entered_ = false;
  ros::Time active_room_started_{0};
  int active_room_no_frontier_cycles_ = 0;
  std::optional<std::pair<int, int>> active_room_target_cell_;
  std::set<std::pair<int, int>> active_room_frontier_blacklist_;
  int room_entry_path_failures_ = 0;
  std::optional<RoomPlan> last_room_plan_;
  std::optional<Pose> room_entry_progress_pose_;
  ros::Time room_entry_progress_time_{0};
  double room_entry_stall_timeout_ = 10.0;
  std::vector<std::vector<double>> room_door_ys_by_floor_;

  // HDPlanner.
  std::unique_ptr<nav::HdplannerPolicy> policy_;

  // Publishers / subscribers / services / timers.
  ros::Publisher cmd_pub_, map_pub_, waypoint_pub_, status_pub_;
  std::vector<ros::Publisher> floor_map_pubs_;
  ros::Subscriber odom_sub_, scan_sub_;
  ros::ServiceServer start_srv_, return_srv_, set_home_srv_, stop_srv_;
  ros::Timer control_timer_, planning_timer_, publish_timer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "competition_navigation");
  ros::NodeHandle pnh("~");
  try {
    CompetitionNavigation node(pnh);
    ros::spin();
  } catch (const std::exception& exc) {
    ROS_FATAL("competition_navigation failed to start: %s", exc.what());
    return 1;
  }
  return 0;
}
