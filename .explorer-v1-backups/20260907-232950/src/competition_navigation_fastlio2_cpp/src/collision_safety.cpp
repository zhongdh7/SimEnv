// C++ port of collision_safety.py -- an independent safety layer that sits
// between the navigation node and the robot servo.  See the Python source for
// the full rationale; the behaviour is unchanged.
#include <ros/ros.h>

#include <cctype>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

namespace {

constexpr int OCCUPIED = 100;
constexpr int FREE = 0;
// How far past an occupied cell to look for FREE space when deciding whether
// it is a solid wall (occluded behind) or a floating phantom obstacle.
constexpr double PHANTOM_LOOKAHEAD = 1.5;

double yaw_from_quat(double qx, double qy, double qz, double qw) {
  return std::atan2(2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz));
}

bool parse_in_stairs(const std::string& data) {
  size_t pos = data.find("mode=");
  if (pos == std::string::npos) return false;
  pos += 5;
  std::string mode;
  while (pos < data.size() &&
         !std::isspace(static_cast<unsigned char>(data[pos]))) {
    mode += data[pos++];
  }
  return mode == "stairs_up" || mode == "stairs_down";
}

}  // namespace

class CollisionSafety {
 public:
  CollisionSafety(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    pnh_.param<std::string>("cmd_in", cmd_in_, "/cmd_vel_raw");
    pnh_.param<std::string>("cmd_out", cmd_out_, "/cmd_vel");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/Odometry_fastlio");
    pnh_.param<std::string>("map_topic", map_topic_,
                            "/competition_navigation/map");
    pnh_.param<std::string>("status_topic", status_topic_,
                            "/competition_navigation/status");
    double rate = 20.0;
    pnh_.param<double>("rate", rate, 20.0);
    rate_ = rate;
    double stale = 0.6;
    pnh_.param<double>("stale_timeout", stale, 0.6);
    stale_timeout_ = ros::Duration(stale);

    pnh_.param<double>("safety_dist", safety_dist_, 0.6);
    pnh_.param<double>("safety_half_width", safety_half_width_, 0.35);
    pnh_.param<double>("min_check_dist", min_check_dist_, 0.25);
    pnh_.param<bool>("room_entry_bypass", room_entry_bypass_, true);
    pnh_.param<bool>("corridor_route_bypass", corridor_route_bypass_, true);
    pnh_.param<double>("corridor_x_min", corridor_x_min_, -1.1);
    pnh_.param<double>("corridor_x_max", corridor_x_max_, 1.1);
    pnh_.param<double>("room_return_crossing_margin",
                       room_return_crossing_margin_, 1.0);
    pnh_.param<double>("corridor_route_bypass_margin",
                       corridor_route_bypass_margin_, 1.0);

    pnh_.param<bool>("recover_enable", recover_enable_, true);
    double reverse_speed = 0.30;
    pnh_.param<double>("reverse_speed", reverse_speed, 0.30);
    reverse_speed_ = -std::abs(reverse_speed);
    pnh_.param<double>("reverse_dist", reverse_dist_, 0.45);
    pnh_.param<double>("reverse_max_time", reverse_max_time_, 4.0);
    pnh_.param<double>("recover_hold_time", recover_hold_time_, 1.5);
    pnh_.param<double>("recover_confirm_time", recover_confirm_time_, 0.8);
    pnh_.param<double>("recover_cooldown", recover_cooldown_, 15.0);

    pnh_.param<bool>("turn_recovery", turn_recovery_, true);
    double turn_yaw = 0.5;
    pnh_.param<double>("turn_yaw_rate", turn_yaw, 0.5);
    turn_yaw_rate_ = std::abs(turn_yaw);
    pnh_.param<double>("turn_duration", turn_duration_, 2.5);

    pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_out_, 5);
    cmd_sub_ = nh_.subscribe(cmd_in_, 5, &CollisionSafety::cmdCb, this);
    odom_sub_ =
        nh_.subscribe(odom_topic_, 5, &CollisionSafety::odomCb, this);
    map_sub_ =
        nh_.subscribe(map_topic_, 5, &CollisionSafety::mapCb, this);
    status_sub_ =
        nh_.subscribe(status_topic_, 5, &CollisionSafety::statusCb, this);

    ROS_INFO(
        "collision_safety: %s -> %s (dist=%.2fm half-width=%.2fm rate=%.0fHz)",
        cmd_in_.c_str(), cmd_out_.c_str(), safety_dist_, safety_half_width_,
        rate_);
  }

  void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) { map_ = msg; }

  void statusCb(const std_msgs::String::ConstPtr& msg) {
    nav_status_ = msg->data;
    bool in_stairs = parse_in_stairs(msg->data);
    if (in_stairs != legacy_in_stairs_) {
      legacy_in_stairs_ = in_stairs;
      ROS_INFO("collision_safety: stair flight %s -> safety box %s",
               in_stairs ? "ACTIVE" : "inactive",
               in_stairs ? "DISABLED" : "enabled");
    }
  }

  void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    const auto& p = msg->pose.pose.position;
    const auto& q = msg->pose.pose.orientation;
    double yaw = yaw_from_quat(q.x, q.y, q.z, q.w);
    pose_ = {p.x, p.y, p.z, yaw};
    has_pose_ = true;
  }

  void cmdCb(const geometry_msgs::Twist::ConstPtr& msg) {
    last_cmd_ = *msg;
    last_cmd_stamp_ = ros::Time::now();
    has_cmd_stamp_ = true;
  }

  // Occupancy at world (x, y), or nullopt if outside the map.
  std::optional<int> occ(double x, double y) const {
    if (!map_) return std::nullopt;
    double res = map_->info.resolution;
    int cx = static_cast<int>(
        std::floor((x - map_->info.origin.position.x) / res));
    int cy = static_cast<int>(
        std::floor((y - map_->info.origin.position.y) / res));
    if (cx >= 0 && cx < map_->info.width && cy >= 0 && cy < map_->info.height) {
      return static_cast<int>(map_->data[cy * map_->info.width + cx]);
    }
    return std::nullopt;
  }

  bool free_behind(double x, double y, double sign, double cos_yaw,
                   double sin_yaw, double start_d, double res) const {
    double d = start_d + res;
    double limit = start_d + PHANTOM_LOOKAHEAD;
    while (d <= limit + 1e-6) {
      std::optional<int> v = occ(x + sign * d * cos_yaw, y + sign * d * sin_yaw);
      if (v.has_value() && v.value() == FREE) return true;
      if (!v.has_value() || v.value() != OCCUPIED) return false;
      d += res;
    }
    return false;
  }

  bool obstacle_is_isolated(double x, double y, double cos_yaw, double sin_yaw,
                            double d) const {
    double res = map_ ? map_->info.resolution : 0.2;
    double ox = x + d * cos_yaw;
    double oy = y + d * sin_yaw;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;
        std::optional<int> v = occ(ox + dx * res, oy + dy * res);
        if (v.has_value() && v.value() == OCCUPIED) return false;
      }
    }
    return true;
  }

  bool blocked(double sign) const {
    if (!has_pose_ || !map_) return false;
    double x = pose_.x, y = pose_.y, yaw = pose_.yaw;
    double cos_yaw = std::cos(yaw), sin_yaw = std::sin(yaw);
    double res = map_->info.resolution;
    double d = min_check_dist_;
    while (d <= safety_dist_ + 1e-6) {
      bool occupied[3];
      double lats[3] = {-safety_half_width_, 0.0, safety_half_width_};
      for (int i = 0; i < 3; ++i) {
        double px = x + sign * d * cos_yaw + lats[i] * -sin_yaw;
        double py = y + sign * d * sin_yaw + lats[i] * cos_yaw;
        std::optional<int> v = occ(px, py);
        occupied[i] = v.has_value() && v.value() == OCCUPIED;
      }
      bool centre = occupied[1];
      bool both_edges = occupied[0] && occupied[2];
      if (centre && !both_edges) {
        if (free_behind(x, y, sign, cos_yaw, sin_yaw, d, res) &&
            obstacle_is_isolated(x, y, cos_yaw, sin_yaw, d)) {
          centre = false;
        }
      }
      if (centre || both_edges) return true;
      d += 0.2;
    }
    return false;
  }

  bool blocked_forward(const geometry_msgs::Twist& cmd) const {
    return has_pose_ && map_ && cmd.linear.x > 0.01 && blocked(+1);
  }

  static bool starts_with(const std::string& value,
                          const std::string& prefix) {
    return value.rfind(prefix, 0) == 0;
  }

  static bool parse_path(const std::string& status, int* index,
                         int* length) {
    const size_t pos = status.find("path=");
    if (pos == std::string::npos) return false;
    int parsed_index = -1;
    int parsed_length = -1;
    if (std::sscanf(status.c_str() + pos, "path=%d/%d", &parsed_index,
                    &parsed_length) != 2) {
      return false;
    }
    *index = parsed_index;
    *length = parsed_length;
    return true;
  }

  // Allow only the bounded forward crossing used by the sensor-built route.
  // FAST-LIO2 and the navigation map share the same pose, so a thin stale
  // door-frame cell can overlap the safety box even after navigation has
  // selected a valid room/corridor path.  The exception is limited to the
  // measured corridor band; all other poses keep the normal safety check.
  bool allow_mapped_crossing(const geometry_msgs::Twist& cmd) const {
    if (!has_pose_ || cmd.linear.x <= 0.01) return false;
    const double x = pose_.x;
    if (starts_with(nav_status_, "room_enter_path")) {
      if (!room_entry_bypass_) return false;
      const double margin = 0.25;
      return x >= corridor_x_min_ - margin &&
             x <= corridor_x_max_ + margin;
    }
    if (starts_with(nav_status_, "room_return_path") ||
        starts_with(nav_status_, "room_return_to_corridor")) {
      if (!room_entry_bypass_) return false;
      return x >= corridor_x_min_ - room_return_crossing_margin_ &&
             x <= corridor_x_max_ + room_return_crossing_margin_;
    }
    if (starts_with(nav_status_, "hdplanner_policy_selected")) {
      if (!corridor_route_bypass_ || pose_.y > 7.85) return false;
      return x >= corridor_x_min_ - corridor_route_bypass_margin_ &&
             x <= corridor_x_max_ + corridor_route_bypass_margin_;
    }
    const bool mapped_route_status =
        starts_with(nav_status_, "dfs_started") ||
        starts_with(nav_status_, "dfs_corridor_segment") ||
        starts_with(nav_status_, "graph_corridor_survey") ||
        starts_with(nav_status_, "frontier_astar_fallback") ||
        starts_with(nav_status_, "hdplanner_graph_recovery") ||
        starts_with(nav_status_, "hdplanner_action_recovery") ||
        starts_with(nav_status_, "hdplanner_selected_path_recovery") ||
        starts_with(nav_status_, "hdplanner_nonprogress_recovery") ||
        starts_with(nav_status_, "go_stairs_up_door_route") ||
        starts_with(nav_status_, "go_stairs_down_door_route") ||
        starts_with(nav_status_, "full_map_complete_return_via_stairs") ||
        starts_with(nav_status_, "full_map_complete_return") ||
        starts_with(nav_status_, "return_planned") ||
        starts_with(nav_status_, "return_waiting_for_path");
    if (!mapped_route_status || !corridor_route_bypass_) return false;
    // The mapped-route exception is only for a straight crossing of a stale
    // single-cell frame.  A large lateral or yaw correction means the body is
    // not following the route's forward corridor anymore; keep the normal
    // safety box active so recovery can turn/replan.
    if (std::abs(cmd.linear.y) > 0.08 || std::abs(cmd.angular.z) > 0.35) {
      return false;
    }
    return x >= corridor_x_min_ - corridor_route_bypass_margin_ &&
           x <= corridor_x_max_ + corridor_route_bypass_margin_;
  }

  // Formal stair flight statuses carry a five-waypoint path.  The flattened
  // 2-D map contains the risers as occupied cells, so only these validated
  // forward flight segments may bypass the safety box.  Approach/planning
  // states and malformed paths remain protected by the normal check.
  bool allow_stair_transition_crossing(
      const geometry_msgs::Twist& cmd) const {
    if (cmd.linear.x <= 0.01) return false;
    if (!(starts_with(nav_status_, "stairs_up_started") ||
          starts_with(nav_status_, "stairs_down_started"))) {
      return false;
    }
    int path_index = -1;
    int path_length = -1;
    if (!parse_path(nav_status_, &path_index, &path_length)) return false;
    return path_length == 5 && path_index >= 0 && path_index < path_length;
  }

  void start_recover(const ros::Time& now) {
    recover_phase_ = "reverse";
    recover_start_pose_ = {pose_.x, pose_.y};
    recover_start_time_ = now;
    blocked_since_ = std::nullopt;
    ROS_WARN("collision_safety: recovery start (reverse), backing off %.2f m",
             reverse_dist_);
  }

  geometry_msgs::Twist recover_step(const ros::Time& now) {
    geometry_msgs::Twist out;
    if (recover_phase_ == "reverse") {
      double traveled = 0.0;
      if (recover_start_pose_.has_value()) {
        traveled = std::hypot(pose_.x - recover_start_pose_->first,
                              pose_.y - recover_start_pose_->second);
      }
      double elapsed = (now - recover_start_time_).toSec();
      if (traveled >= reverse_dist_ || elapsed >= reverse_max_time_ ||
          blocked(-1)) {
        if (turn_recovery_) {
          recover_phase_ = "turn";
          // Alternate the escape direction after each wedged recovery.  A
          // fixed turn direction can keep the robot circling into the same
          // wall when both the forward and reverse safety boxes are blocked.
          turn_sign_ = -turn_sign_;
          recover_start_time_ = now;
          ROS_WARN(
              "collision_safety: reverse blocked/done (%.2f m), turning %s",
              traveled, turn_sign_ > 0.0 ? "left" : "right");
        } else {
          recover_phase_ = "hold";
          recover_start_time_ = now;
          ROS_WARN("collision_safety: reverse done (%.2f m), holding",
                   traveled);
        }
      } else {
        out.linear.x = reverse_speed_;
        out.linear.y = 0.0;
        out.angular.z = 0.0;
        return out;
      }
    }
    if (recover_phase_ == "turn") {
      if ((now - recover_start_time_).toSec() >= turn_duration_) {
        recover_phase_ = "hold";
        recover_start_time_ = now;
        ROS_WARN("collision_safety: turn escape done, holding");
      } else {
        out.linear.x = 0.0;
        out.linear.y = 0.0;
        out.angular.z = turn_sign_ * turn_yaw_rate_;
        return out;
      }
    }
    if (recover_phase_ == "hold") {
      if ((now - recover_start_time_).toSec() >= recover_hold_time_) {
        recover_phase_.clear();
        last_recover_end_ = ros::Time::now();
        ROS_WARN("collision_safety: recovery finished");
      } else {
        out.linear.x = 0.0;
        out.linear.y = 0.0;
        out.angular.z = 0.0;
        return out;
      }
    }
    return out;
  }

  void spin() {
    // Unlike rospy (whose subscriber callbacks run on their own threads),
    // roscpp only dispatches callbacks when the node spins.  This manual loop
    // must call ros::spinOnce() each iteration so cmd/odom/map/status callbacks
    // actually fire, and uses WallRate so the loop stays at real-time (the
    // rospy.Rate original sleeps in wall time; ros::Rate would stall on sim
    // time).
    ros::WallRate r(rate_);
    while (ros::ok()) {
      ros::spinOnce();
      geometry_msgs::Twist cmd = last_cmd_;
      ros::Time now = ros::Time::now();
      if (has_cmd_stamp_ && now - last_cmd_stamp_ > stale_timeout_) {
        cmd = geometry_msgs::Twist();
        recover_phase_.clear();
        blocked_since_ = std::nullopt;
        ROS_WARN_THROTTLE(2.0,
                          "collision_safety: cmd_vel stale, holding zero");
      } else if (!recover_phase_.empty()) {
        cmd = recover_step(now);
      } else if (blocked_forward(cmd)) {
        const bool mapped_bypass = allow_mapped_crossing(cmd);
        const bool stair_bypass = allow_stair_transition_crossing(cmd);
        if (mapped_bypass || stair_bypass) {
          blocked_since_ = std::nullopt;
          ROS_INFO_THROTTLE(
              2.0,
              "collision_safety: allowing bounded planned forward crossing "
              "status=%s",
              nav_status_.c_str());
        } else if (!recover_enable_) {
          cmd.linear.x = 0.0;
          cmd.linear.y = 0.0;
          ROS_WARN_THROTTLE(1.0,
                            "collision_safety: forward blocked, zeroing "
                            "velocity");
        } else {
          if (!blocked_since_.has_value()) {
            blocked_since_ = now;
          }
          if ((now - blocked_since_.value()) <
              ros::Duration(recover_confirm_time_)) {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
          } else if ((now - last_recover_end_) >=
                     ros::Duration(recover_cooldown_)) {
            start_recover(now);
            cmd = recover_step(now);
          } else {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            ROS_WARN_THROTTLE(
                2.0,
                "collision_safety: forward blocked (recovery cooldown), "
                "holding");
          }
        }
      } else if (cmd.linear.x < -0.01 && has_pose_ && map_ &&
                 blocked(-1)) {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        ROS_WARN_THROTTLE(1.0,
                          "collision_safety: reverse blocked, zeroing velocity");
      } else {
        blocked_since_ = std::nullopt;
      }
      pub_.publish(cmd);
      r.sleep();
    }
  }

 private:
  struct Pose {
    double x, y, z, yaw;
  };

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string cmd_in_, cmd_out_, odom_topic_, map_topic_, status_topic_;
  double rate_ = 20.0;
  ros::Duration stale_timeout_{0.6};

  double safety_dist_ = 0.6;
  double safety_half_width_ = 0.35;
  double min_check_dist_ = 0.25;

  bool recover_enable_ = true;
  double reverse_speed_ = -0.30;
  double reverse_dist_ = 0.45;
  double reverse_max_time_ = 4.0;
  double recover_hold_time_ = 1.5;
  double recover_confirm_time_ = 0.8;
  double recover_cooldown_ = 15.0;

  bool turn_recovery_ = true;
  double turn_yaw_rate_ = 0.5;
  double turn_duration_ = 2.5;

  ros::Publisher pub_;
  ros::Subscriber cmd_sub_, odom_sub_, map_sub_, status_sub_;

  nav_msgs::OccupancyGrid::ConstPtr map_;
  Pose pose_{0, 0, 0, 0};
  bool has_pose_ = false;
  bool legacy_in_stairs_ = false;
  std::string nav_status_;
  bool room_entry_bypass_ = true;
  bool corridor_route_bypass_ = true;
  double corridor_x_min_ = -1.1;
  double corridor_x_max_ = 1.1;
  double room_return_crossing_margin_ = 1.0;
  double corridor_route_bypass_margin_ = 1.0;
  geometry_msgs::Twist last_cmd_;
  ros::Time last_cmd_stamp_;
  bool has_cmd_stamp_ = false;
  std::string recover_phase_;
  std::optional<std::pair<double, double>> recover_start_pose_;
  ros::Time recover_start_time_;
  std::optional<ros::Time> blocked_since_;
  ros::Time last_recover_end_{0};
  // First recovery turns left; subsequent wedged recoveries alternate sides.
  double turn_sign_ = -1.0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "collision_safety");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  CollisionSafety safety(nh, pnh);
  safety.spin();
  return 0;
}
