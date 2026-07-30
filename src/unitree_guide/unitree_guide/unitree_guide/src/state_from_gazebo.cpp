#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
// #include "tf2_ros/transform_listener.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>   // 将tf监听绑定到ROS回调函数


using namespace std;
ros::Publisher robotVelocity_BASE_frame_pub;
string robot_name = "a1";
nav_msgs::Odometry Odom;
double x=0, y=0, z=0, roll=0, pitch=0, yaw=0;


void callback_BASE(const gazebo_msgs::LinkStates::ConstPtr &msg) {
    int index = 0;
    for (auto &linkName : msg->name) {
        if (linkName == robot_name+"_gazebo::base")
            break;
        ++index;
    }
    if (index == static_cast<int>(msg->name.size())) {
        return;
    }
    ros::Rate rate(500);//延迟至100hz发布，避免重复发布

    // NOTE: map→odom TF is now published exclusively by AMCL.
    // The old fixed-identity broadcast here caused TF_REPEATED_DATA conflicts
    // that broke AMCL's localisation correction.  See explorer-v2 teb_navigation.

    // Gazebo ground-truth pose is already in the world frame ("map"), and
    // since AMCL publishes the corrective map→odom transform we just leave
    // the Gazebo data in the odom frame as-is (identity between map and odom
    // at this level — AMCL will shift odom relative to map as needed).
    tf::Point pt_map(msg->pose[index].position.x,msg->pose[index].position.y,msg->pose[index].position.z);
    tf::Point pt_odom = pt_map;              // identity (was transform_map2odom * pt_map)

    tf::Quaternion q_map(msg->pose[index].orientation.x,
                        msg->pose[index].orientation.y,
                        msg->pose[index].orientation.z,
                        msg->pose[index].orientation.w);
    tf::Quaternion q_odom = q_map;           // identity (was transform_map2odom * q_map)

    // 转换为odom的速度关系
    tf::Vector3 linear_vel(
        msg->twist[index].linear.x,
        msg->twist[index].linear.y,
        msg->twist[index].linear.z);
    tf::Vector3 transformed_linear_vel = linear_vel;   // identity

    tf::Vector3 angular_vel(
        msg->twist[index].angular.x,
        msg->twist[index].angular.y,
        msg->twist[index].angular.z);
    tf::Vector3 transformed_angular_vel = angular_vel;  // identity
    
    //发布base到odom的tf变换
    static tf::TransformBroadcaster bf2;
    tf::Transform transform_odom2base;
    transform_odom2base.setRotation(q_odom);
    transform_odom2base.setOrigin(pt_odom);

    bf2.sendTransform(tf::StampedTransform(transform_odom2base, ros::Time::now(), "odom", "base"));

    Odom.header.stamp = ros::Time::now();
    Odom.header.frame_id = "odom";
    Odom.child_frame_id = "base";

    // set the position
    Odom.pose.pose.position.x = pt_odom.x();
    Odom.pose.pose.position.y = pt_odom.y();
    Odom.pose.pose.position.z = pt_odom.z();

    Odom.pose.pose.orientation.w = q_odom.w();
    Odom.pose.pose.orientation.x = q_odom.x();
    Odom.pose.pose.orientation.y = q_odom.y();
    Odom.pose.pose.orientation.z = q_odom.z();


    // set the velocity
    Odom.twist.twist.linear.x= transformed_linear_vel.x();
    Odom.twist.twist.linear.y= transformed_linear_vel.y();
    Odom.twist.twist.linear.z= transformed_linear_vel.z();


    Odom.twist.twist.angular.x = transformed_angular_vel.x();
    Odom.twist.twist.angular.y = transformed_angular_vel.y();
    Odom.twist.twist.angular.z = transformed_angular_vel.z();


    robotVelocity_BASE_frame_pub.publish(Odom);
    rate.sleep();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "state_from_gazebo");
    ros::NodeHandle nh("~");
    ros::NodeHandle node;
    ros::Subscriber tfState_BASE_sub;

    // tf::TransformListener tf_listener_;

    if (argc != 7)   // x y z qx qy qz qw 
    {
        ROS_ERROR("Usage: static_transform_publisher x y z yaw pitch roll");
        return -1;
    }

    x = atof(argv[1]);
    y = atof(argv[2]);
    z = atof(argv[3]);

    double yaw   = atof(argv[4]);
    double pitch = atof(argv[5]);
    double roll  = atof(argv[6]);
  
    nh.param<std::string>("robot_name", robot_name, string("a1"));
    tfState_BASE_sub = node.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, callback_BASE);
    robotVelocity_BASE_frame_pub = node.advertise<nav_msgs::Odometry>("/Odometry_gazebo", 1);

    ros::spin();
    return 0;
}


