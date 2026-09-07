// C++ port of scan_tf_relay.py.  Bridges FAST-LIO2 odometry into the
// navigation node's TF expectations by re-anchoring camera_init -> odom with a
// one-time ground-truth anchor.  See the Python source for the full rationale;
// the math and behaviour are unchanged.
#include <ros/ros.h>

#include <cmath>
#include <deque>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/transform_broadcaster.h>
#include <XmlRpcValue.h>

namespace {

bool finite(double v) { return std::isfinite(v); }

bool finite_all(const std::vector<double>& v) {
  for (double x : v) {
    if (!std::isfinite(x)) return false;
  }
  return true;
}

Eigen::Matrix3d rot_matrix(const Eigen::Quaterniond& q) {
  return q.normalized().toRotationMatrix();
}

double yaw_from_quat(const Eigen::Quaterniond& q) {
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

bool get_double_vec(const ros::NodeHandle& nh, const std::string& name,
                    std::vector<double>& out) {
  if (!nh.hasParam(name)) return false;
  XmlRpc::XmlRpcValue val;
  if (!nh.getParam(name, val)) return false;
  if (val.getType() != XmlRpc::XmlRpcValue::TypeArray) return false;
  out.clear();
  for (int i = 0; i < val.size(); ++i) {
    out.push_back(static_cast<double>(val[i]));
  }
  return true;
}

struct GtEntry {
  double t;
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
};

}  // namespace

class ScanTfRelay {
 public:
  ScanTfRelay(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    pnh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    pnh_.param<std::string>("gt_odom_topic", gt_odom_topic_,
                            "/Odometry_gazebo");
    pnh_.param<std::string>("out_odom_topic", out_odom_topic_,
                            "/Odometry_fastlio");
    pnh_.param<std::string>("scan_topic", scan_topic_, "/scan");
    pnh_.param<std::string>("out_scan_topic", out_scan_topic_, "/scan_fastlio");
    pnh_.param<std::string>("map_frame", map_frame_, "odom");
    pnh_.param<std::string>("laser_frame", laser_frame_, "laser_livox_fastlio");

    std::vector<double> t_bl_default = {0.2, 0.0, 0.08};
    std::vector<double> rpy_bl_default = {0.0, M_PI / 4.0, 0.0};
    std::vector<double> t_bl = t_bl_default;
    std::vector<double> rpy_bl = rpy_bl_default;
    get_double_vec(pnh_, "extrinsic_T", t_bl);
    get_double_vec(pnh_, "extrinsic_rpy", rpy_bl);
    if (t_bl.size() >= 3) {
      t_bl_ = Eigen::Vector3d(t_bl[0], t_bl[1], t_bl[2]);
    }
    double r = rpy_bl.size() >= 3 ? rpy_bl[0] : 0.0;
    double p = rpy_bl.size() >= 3 ? rpy_bl[1] : M_PI / 4.0;
    double y = rpy_bl.size() >= 3 ? rpy_bl[2] : 0.0;
    q_bl_ = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());

    pnh_.param<double>("anchor_delay", anchor_delay_, 2.0);

    odom_sub_ = nh_.subscribe(odom_topic_, 5, &ScanTfRelay::odomCb, this);
    gt_sub_ = nh_.subscribe(gt_odom_topic_, 5, &ScanTfRelay::gtCb, this);
    scan_sub_ = nh_.subscribe(scan_topic_, 1, &ScanTfRelay::scanCb, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(out_odom_topic_, 5);
    scan_pub_ = nh_.advertise<sensor_msgs::PointCloud>(out_scan_topic_, 1);

    ROS_INFO("scan_tf_relay: odom=%s -> %s (map=%s, laser=%s)",
             odom_topic_.c_str(), out_odom_topic_.c_str(), map_frame_.c_str(),
             laser_frame_.c_str());
    ROS_INFO(
        "scan_tf_relay: initial anchor from %s (used once at start-up only)",
        gt_odom_topic_.c_str());
  }

  void gtCb(const nav_msgs::Odometry::ConstPtr& msg) {
    const auto& p = msg->pose.pose.position;
    const auto& q = msg->pose.pose.orientation;
    std::vector<double> vals = {p.x, p.y, p.z, q.x, q.y, q.z, q.w};
    if (!finite_all(vals)) return;
    GtEntry entry;
    entry.t = msg->header.stamp.toSec();
    entry.p = Eigen::Vector3d(p.x, p.y, p.z);
    entry.q = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
    gt_buf_.push_back(entry);
    while (gt_buf_.size() > 400) gt_buf_.pop_front();
  }

  // GT pose closest in time to ``t``.
  std::optional<GtEntry> gtAt(double t) const {
    if (gt_buf_.empty()) return std::nullopt;
    const GtEntry* best = nullptr;
    double best_dt = 0.0;
    for (const GtEntry& entry : gt_buf_) {
      double dt = std::abs(entry.t - t);
      if (best == nullptr || dt < best_dt) {
        best = &entry;
        best_dt = dt;
      }
    }
    return *best;
  }

  void tryLockAlignment(const Eigen::Quaterniond& q_cb0,
                        const Eigen::Vector3d& t_cb0, double stamp) {
    if (aligned_) return;
    if (!first_fl_time_.has_value()) {
      first_fl_time_ = stamp;
      ROS_INFO(
          "scan_tf_relay: waiting %.1fs for FAST-LIO2 to converge before "
          "anchoring",
          anchor_delay_);
      return;
    }
    if (stamp - first_fl_time_.value() < anchor_delay_) return;
    if (gt_buf_.empty()) {
      ROS_WARN_THROTTLE(5.0,
                        "scan_tf_relay: waiting for %s to anchor the initial "
                        "pose",
                        gt_odom_topic_.c_str());
      return;
    }
    std::optional<GtEntry> gt = gtAt(stamp);
    if (!gt.has_value()) return;
    const Eigen::Vector3d& t_ob0 = gt->p;
    const Eigen::Quaterniond& q_ob0 = gt->q;

    // R_oc = R_ob(0) * R_cb(0)^T,  t_oc = t_ob(0) - R_oc * t_cb(0).
    Eigen::Quaterniond q_oc = q_ob0 * q_cb0.inverse();
    Eigen::Matrix3d R_oc = rot_matrix(q_oc);
    Eigen::Vector3d t_oc = t_ob0 - R_oc * t_cb0;

    q_oc_ = q_oc;
    t_oc_ = t_oc;
    aligned_ = true;

    double yaw = yaw_from_quat(q_oc);
    ROS_INFO(
        "scan_tf_relay: alignment locked t_oc=(%.3f, %.3f, %.3f) yaw=%.2f deg",
        t_oc.x(), t_oc.y(), t_oc.z(), yaw * 180.0 / M_PI);
  }

  void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    const auto& p = msg->pose.pose.position;
    const auto& q = msg->pose.pose.orientation;
    std::vector<double> vals = {p.x, p.y, p.z, q.x, q.y, q.z, q.w};
    if (!finite_all(vals)) return;

    Eigen::Quaterniond q_cb(q.w, q.x, q.y, q.z);
    Eigen::Vector3d t_cb(p.x, p.y, p.z);

    tryLockAlignment(q_cb, t_cb, msg->header.stamp.toSec());
    if (!aligned_) return;
    Eigen::Quaterniond q_oc = q_oc_;
    Eigen::Vector3d t_oc = t_oc_;

    // Body pose in odom: R_ob = R_oc * R_cb, t_ob = t_oc + R_oc * t_cb.
    Eigen::Quaterniond q_ob = q_oc * q_cb;
    Eigen::Matrix3d R_oc = rot_matrix(q_oc);
    Eigen::Matrix3d R_ob = rot_matrix(q_ob);
    Eigen::Vector3d t_ob = t_oc + R_oc * t_cb;

    // Lidar pose in odom: q_ol = q_ob * q_bl, t_ol = t_ob + R_ob * t_bl.
    Eigen::Quaterniond q_ol = q_ob * q_bl_;
    Eigen::Vector3d t_ol = t_ob + R_ob * t_bl_;

    geometry_msgs::TransformStamped ts;
    ts.header.stamp = msg->header.stamp;
    ts.header.frame_id = map_frame_;
    ts.child_frame_id = laser_frame_;
    ts.transform.translation.x = t_ol.x();
    ts.transform.translation.y = t_ol.y();
    ts.transform.translation.z = t_ol.z();
    ts.transform.rotation.x = q_ol.x();
    ts.transform.rotation.y = q_ol.y();
    ts.transform.rotation.z = q_ol.z();
    ts.transform.rotation.w = q_ol.w();
    broadcaster_.sendTransform(ts);

    nav_msgs::Odometry out;
    out.header = msg->header;
    out.header.frame_id = map_frame_;
    out.child_frame_id = "body";
    out.pose.pose.position.x = t_ob.x();
    out.pose.pose.position.y = t_ob.y();
    out.pose.pose.position.z = t_ob.z();
    out.pose.pose.orientation.x = q_ob.x();
    out.pose.pose.orientation.y = q_ob.y();
    out.pose.pose.orientation.z = q_ob.z();
    out.pose.pose.orientation.w = q_ob.w();
    out.pose.covariance = msg->pose.covariance;
    out.twist = msg->twist;
    odom_pub_.publish(out);
  }

  void scanCb(const sensor_msgs::PointCloud::ConstPtr& msg) {
    sensor_msgs::PointCloud relay;
    relay.header = msg->header;
    relay.header.frame_id = laser_frame_;
    relay.points = msg->points;
    relay.channels = msg->channels;
    scan_pub_.publish(relay);
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string odom_topic_, gt_odom_topic_, out_odom_topic_, scan_topic_,
      out_scan_topic_, map_frame_, laser_frame_;
  Eigen::Vector3d t_bl_ = Eigen::Vector3d(0.2, 0.0, 0.08);
  Eigen::Quaterniond q_bl_;
  double anchor_delay_ = 2.0;

  ros::Subscriber odom_sub_, gt_sub_, scan_sub_;
  ros::Publisher odom_pub_, scan_pub_;
  tf2_ros::TransformBroadcaster broadcaster_;

  bool aligned_ = false;
  Eigen::Quaterniond q_oc_;
  Eigen::Vector3d t_oc_;
  std::optional<double> first_fl_time_;
  std::deque<GtEntry> gt_buf_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_tf_relay");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ScanTfRelay relay(nh, pnh);
  ros::spin();
  return 0;
}
