// FAST-LIO2 -> building-aligned odometry relay.
//
// FAST-LIO2's camera_init frame is estimator-local and is not necessarily
// aligned with the building axes used by the exploration graph.  This relay
// applies one configured deployment transform, optionally using the initial
// IMU yaw, and then forwards only live FAST-LIO2 pose updates.  It has no
// Gazebo truth subscription.  The Python relay in competition_navigation_fastlio2
// intentionally implements the same state and parameter semantics.

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <XmlRpcValue.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_broadcaster.h>

namespace {

bool finite_all(const std::vector<double>& values) {
  for (double value : values) {
    if (!std::isfinite(value)) return false;
  }
  return true;
}

bool get_double_vec(const ros::NodeHandle& nh, const std::string& name,
                    std::vector<double>& out) {
  if (!nh.hasParam(name)) return false;
  XmlRpc::XmlRpcValue value;
  if (!nh.getParam(name, value) ||
      value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return false;
  }
  out.clear();
  for (int i = 0; i < value.size(); ++i) {
    if (value[i].getType() != XmlRpc::XmlRpcValue::TypeInt &&
        value[i].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
      return false;
    }
    out.push_back(static_cast<double>(value[i]));
  }
  return true;
}

Eigen::Quaterniond pose_quaternion(const std::vector<double>& pose) {
  return Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX());
}

struct SensorPose {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  double roll;
  double pitch;
  double yaw;
};

bool read_sensor_pose(const nav_msgs::Odometry::ConstPtr& msg,
                      SensorPose& pose) {
  const auto& p = msg->pose.pose.position;
  const auto& q = msg->pose.pose.orientation;
  const std::vector<double> values = {p.x, p.y, p.z, q.x, q.y, q.z, q.w};
  if (!finite_all(values)) return false;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm <= 1e-9) return false;
  pose.position = Eigen::Vector3d(p.x, p.y, p.z);
  pose.orientation = Eigen::Quaterniond(q.w / norm, q.x / norm,
                                        q.y / norm, q.z / norm);
  const auto& n = pose.orientation;
  pose.roll = std::atan2(2.0 * (n.w() * n.x() + n.y() * n.z()),
                         1.0 - 2.0 * (n.x() * n.x() + n.y() * n.y()));
  const double pitch_arg = std::clamp(
      2.0 * (n.w() * n.y() - n.z() * n.x()), -1.0, 1.0);
  pose.pitch = std::asin(pitch_arg);
  pose.yaw = std::atan2(2.0 * (n.w() * n.z() + n.x() * n.y()),
                        1.0 - 2.0 * (n.y() * n.y() + n.z() * n.z()));
  return true;
}

bool read_pose_param(const ros::NodeHandle& nh, const std::string& name,
                     const std::vector<double>& fallback,
                     std::vector<double>& out) {
  out = fallback;
  if (nh.hasParam(name) && !get_double_vec(nh, name, out)) return false;
  return out.size() == 6 && finite_all(out);
}

}  // namespace

class ScanTfRelay {
 public:
  ScanTfRelay(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    pnh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    pnh_.param<std::string>("out_odom_topic", out_odom_topic_,
                            "/Odometry_fastlio");
    pnh_.param<std::string>("scan_topic", scan_topic_, "/scan");
    pnh_.param<std::string>("out_scan_topic", out_scan_topic_,
                            "/scan_fastlio");
    pnh_.param<std::string>("map_frame", map_frame_, "odom");
    pnh_.param<std::string>("body_frame", body_frame_, "base");
    pnh_.param<std::string>("laser_frame", laser_frame_,
                            "laser_livox_fastlio");
    pnh_.param<bool>("publish_tf", publish_tf_, true);

    const std::vector<double> initial_default =
        {0.0, -3.2, 0.263, 0.0, 0.0, M_PI / 2.0};
    const std::vector<double> fastlio_default = {0.0, 0.0, 0.0,
                                                  0.0, 0.0, 0.0};
    if (!read_pose_param(pnh_, "initial_pose", initial_default, initial_pose_) ||
        !read_pose_param(pnh_, "fastlio_initial_pose", fastlio_default,
                         fastlio_initial_pose_)) {
      throw std::runtime_error(
          "initial_pose and fastlio_initial_pose must be finite arrays of 6 values");
    }

    pnh_.param<bool>("sensor_only_anchor", sensor_only_anchor_, false);
    pnh_.param<double>("anchor_delay", anchor_delay_, 2.0);
    pnh_.param<std::string>("imu_topic", imu_topic_, "/trunk_imu");
    pnh_.param<bool>("use_imu_initial_yaw", use_imu_initial_yaw_, true);
    pnh_.param<double>("imu_initial_yaw_wait", imu_initial_yaw_wait_, 1.0);
    pnh_.param<double>("imu_initial_yaw_max_age", imu_initial_yaw_max_age_, 0.5);
    if (!std::isfinite(anchor_delay_) || anchor_delay_ < 0.0 ||
        !std::isfinite(imu_initial_yaw_wait_) ||
        !std::isfinite(imu_initial_yaw_max_age_)) {
      throw std::runtime_error("FAST-LIO2 alignment timing parameters are invalid");
    }
    imu_initial_yaw_wait_ = std::max(0.1, imu_initial_yaw_wait_);
    imu_initial_yaw_max_age_ = std::max(0.05, imu_initial_yaw_max_age_);

    std::vector<double> t_bl = {0.2, 0.0, 0.08};
    std::vector<double> rpy_bl = {0.0, M_PI / 4.0, 0.0};
    if (pnh_.hasParam("extrinsic_T") && !get_double_vec(pnh_, "extrinsic_T", t_bl)) {
      throw std::runtime_error("extrinsic_T must be a numeric array");
    }
    if (pnh_.hasParam("extrinsic_rpy") &&
        !get_double_vec(pnh_, "extrinsic_rpy", rpy_bl)) {
      throw std::runtime_error("extrinsic_rpy must be a numeric array");
    }
    if (t_bl.size() != 3 || rpy_bl.size() != 3 ||
        !finite_all(t_bl) || !finite_all(rpy_bl)) {
      throw std::runtime_error("extrinsic_T and extrinsic_rpy must have 3 finite values");
    }
    t_bl_ = Eigen::Vector3d(t_bl[0], t_bl[1], t_bl[2]);
    q_bl_ = Eigen::AngleAxisd(rpy_bl[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy_bl[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy_bl[0], Eigen::Vector3d::UnitX());

    ready_pub_ = nh_.advertise<std_msgs::Bool>(
        pnh_.param<std::string>("anchor_ready_topic", std::string("/fastlio2/anchor_ready")),
        1, true);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(out_odom_topic_, 5);
    scan_pub_ = nh_.advertise<sensor_msgs::PointCloud>(out_scan_topic_, 1);
    std_msgs::Bool ready;
    ready.data = aligned_;
    ready_pub_.publish(ready);

    if (!sensor_only_anchor_ && !use_imu_initial_yaw_) {
      lockAlignment(initial_pose_, fastlio_initial_pose_, "configured");
    }

    odom_sub_ = nh_.subscribe(odom_topic_, 5, &ScanTfRelay::odomCb, this);
    imu_sub_ = nh_.subscribe(imu_topic_, 20, &ScanTfRelay::imuCb, this);
    scan_sub_ = nh_.subscribe(scan_topic_, 1, &ScanTfRelay::scanCb, this);

    ROS_INFO("scan_tf_relay: %s -> %s (map=%s, laser=%s, publish_tf=%s)",
             odom_topic_.c_str(), out_odom_topic_.c_str(), map_frame_.c_str(),
             laser_frame_.c_str(), publish_tf_ ? "true" : "false");
    ROS_INFO("scan_tf_relay: %s initial alignment; simulator truth is disabled",
             sensor_only_anchor_ ? "sensor-only" : "configured");
  }

 private:
  void imuCb(const sensor_msgs::Imu::ConstPtr& msg) {
    const auto& q = msg->orientation;
    const std::vector<double> values = {q.x, q.y, q.z, q.w};
    if (!finite_all(values)) return;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(norm) || norm <= 1e-9 ||
        (!msg->orientation_covariance.empty() &&
         msg->orientation_covariance[0] < 0.0)) {
      return;
    }
    const double x = q.x / norm;
    const double y = q.y / norm;
    const double z = q.z / norm;
    const double w = q.w / norm;
    imu_yaw_ = std::atan2(2.0 * (w * z + x * y),
                          1.0 - 2.0 * (y * y + z * z));
    imu_stamp_ = msg->header.stamp.toSec();
    imu_valid_ = std::isfinite(imu_yaw_) && std::isfinite(imu_stamp_);
  }

  void lockAlignment(const std::vector<double>& deployment_pose,
                     const std::vector<double>& sensor_pose,
                     const std::string& reason) {
    const Eigen::Quaterniond q_ob0 = pose_quaternion(deployment_pose);
    const Eigen::Quaterniond q_cb0 = pose_quaternion(sensor_pose);
    q_oc_ = q_ob0 * q_cb0.inverse();
    t_oc_ = Eigen::Vector3d(deployment_pose[0], deployment_pose[1],
                            deployment_pose[2]) -
            q_oc_.normalized().toRotationMatrix() *
                Eigen::Vector3d(sensor_pose[0], sensor_pose[1], sensor_pose[2]);
    aligned_initial_yaw_ = deployment_pose[5];
    if (reason == "IMU initial yaw" && imu_valid_) {
      imu_anchor_yaw_ = imu_yaw_;
      imu_yaw_tracking_ = true;
    }
    aligned_ = true;
    std_msgs::Bool ready;
    ready.data = true;
    ready_pub_.publish(ready);
    ROS_INFO("scan_tf_relay: %s alignment locked", reason.c_str());
  }

  void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    SensorPose sensor;
    if (!read_sensor_pose(msg, sensor)) {
      ROS_WARN_THROTTLE(5.0, "scan_tf_relay: FAST-LIO2 odometry is invalid");
      return;
    }
    const double stamp = msg->header.stamp.toSec();
    if (!std::isfinite(stamp)) return;
    if (!first_odom_time_valid_) {
      first_odom_time_ = stamp;
      first_odom_time_valid_ = true;
      ROS_INFO("scan_tf_relay: first FAST-LIO2 pose received");
    }

    if (!aligned_) {
      if (sensor_only_anchor_ && stamp - first_odom_time_ >= anchor_delay_) {
        std::vector<double> pose = {sensor.position.x(), sensor.position.y(),
                                    sensor.position.z(), sensor.roll,
                                    sensor.pitch, sensor.yaw};
        lockAlignment(initial_pose_, pose, "sensor-only");
      } else if (use_imu_initial_yaw_ && !sensor_only_anchor_) {
        // Do not anchor on the first estimator sample.  FAST-LIO2 can publish
        // a finite but unconverged pose during IMU/map initialisation; wait
        // for the configured settle delay before fixing the frame offset.
        if (stamp - first_odom_time_ <
            std::max(anchor_delay_, imu_initial_yaw_wait_)) {
          return;
        }
        if (imu_valid_ && std::abs(stamp - imu_stamp_) <= imu_initial_yaw_max_age_) {
          std::vector<double> deployment = initial_pose_;
          deployment[5] = imu_yaw_;
          lockAlignment(deployment, fastlio_initial_pose_, "IMU initial yaw");
        } else if (stamp - first_odom_time_ >= imu_initial_yaw_wait_) {
          lockAlignment(initial_pose_, fastlio_initial_pose_,
                        "configured fallback");
        }
      }
    }
    if (!aligned_) return;

    Eigen::Quaterniond q_ob = q_oc_ * sensor.orientation;
    if (imu_yaw_tracking_ && imu_valid_ &&
        std::abs(stamp - imu_stamp_) <= imu_initial_yaw_max_age_) {
      const double yaw_delta = std::atan2(
          std::sin(imu_yaw_ - imu_anchor_yaw_),
          std::cos(imu_yaw_ - imu_anchor_yaw_));
      const double desired_yaw = aligned_initial_yaw_ + yaw_delta;
      const double current_yaw = std::atan2(
          2.0 * (q_ob.w() * q_ob.z() + q_ob.x() * q_ob.y()),
          1.0 - 2.0 * (q_ob.y() * q_ob.y() + q_ob.z() * q_ob.z()));
      const double correction = std::atan2(
          std::sin(desired_yaw - current_yaw),
          std::cos(desired_yaw - current_yaw));
      // Apply only a world-Z increment.  This preserves FAST-LIO2 roll/pitch
      // and avoids mixing Eigen's XYZ extraction convention with the ZYX
      // pose construction convention.
      q_ob = Eigen::AngleAxisd(correction, Eigen::Vector3d::UnitZ()) * q_ob;
      q_ob.normalize();
    }
    const Eigen::Vector3d t_ob =
        t_oc_ + q_oc_.normalized().toRotationMatrix() * sensor.position;
    const Eigen::Quaterniond q_ol = q_ob * q_bl_;
    const Eigen::Vector3d t_ol =
        t_ob + q_ob.normalized().toRotationMatrix() * t_bl_;

    if (publish_tf_) {
      // Unitree Estimator already owns odom -> base.  Broadcasting the same
      // edge here creates duplicate timestamps and competing parents.  The
      // relay only owns the FAST-LIO2 lidar edge needed by scan consumers.
      geometry_msgs::TransformStamped laser_tf;
      laser_tf.header.stamp = msg->header.stamp;
      laser_tf.header.frame_id = map_frame_;
      laser_tf.child_frame_id = laser_frame_;
      laser_tf.transform.translation.x = t_ol.x();
      laser_tf.transform.translation.y = t_ol.y();
      laser_tf.transform.translation.z = t_ol.z();
      laser_tf.transform.rotation.x = q_ol.x();
      laser_tf.transform.rotation.y = q_ol.y();
      laser_tf.transform.rotation.z = q_ol.z();
      laser_tf.transform.rotation.w = q_ol.w();
      broadcaster_.sendTransform(laser_tf);
    }

    nav_msgs::Odometry output = *msg;
    output.header.frame_id = map_frame_;
    output.child_frame_id = body_frame_;
    output.pose.pose.position.x = t_ob.x();
    output.pose.pose.position.y = t_ob.y();
    output.pose.pose.position.z = t_ob.z();
    output.pose.pose.orientation.x = q_ob.x();
    output.pose.pose.orientation.y = q_ob.y();
    output.pose.pose.orientation.z = q_ob.z();
    output.pose.pose.orientation.w = q_ob.w();
    odom_pub_.publish(output);
  }

  void scanCb(const sensor_msgs::PointCloud::ConstPtr& msg) {
    sensor_msgs::PointCloud output = *msg;
    output.header.frame_id = laser_frame_;
    scan_pub_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string odom_topic_, out_odom_topic_, scan_topic_, out_scan_topic_;
  std::string map_frame_, body_frame_, laser_frame_, imu_topic_;
  bool publish_tf_ = true;
  bool sensor_only_anchor_ = false;
  bool use_imu_initial_yaw_ = true;
  double anchor_delay_ = 2.0;
  double imu_initial_yaw_wait_ = 1.0;
  double imu_initial_yaw_max_age_ = 0.5;
  std::vector<double> initial_pose_;
  std::vector<double> fastlio_initial_pose_;
  Eigen::Vector3d t_bl_ = Eigen::Vector3d(0.2, 0.0, 0.08);
  Eigen::Quaterniond q_bl_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond q_oc_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t_oc_ = Eigen::Vector3d::Zero();
  bool aligned_ = false;
  bool first_odom_time_valid_ = false;
  double first_odom_time_ = 0.0;
  bool imu_valid_ = false;
  double imu_yaw_ = 0.0;
  double imu_stamp_ = 0.0;
  double imu_anchor_yaw_ = 0.0;
  double aligned_initial_yaw_ = 0.0;
  bool imu_yaw_tracking_ = false;
  ros::Subscriber odom_sub_, imu_sub_, scan_sub_;
  ros::Publisher odom_pub_, scan_pub_, ready_pub_;
  tf2_ros::TransformBroadcaster broadcaster_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_tf_relay");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  try {
    ScanTfRelay relay(nh, pnh);
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("scan_tf_relay: %s", error.what());
    return 1;
  }
  return 0;
}
