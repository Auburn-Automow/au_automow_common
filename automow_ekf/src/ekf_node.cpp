#include "ros/ros.h"

#include <sstream>

#include "automow_ekf.h"
#include "ax2550/StampedEncoders.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"

automow_ekf::Automow_EKF ekf;

ros::Publisher odom_pub;

void encoderCallback(const ax2550::StampedEncoders::ConstPtr& msg) {
    try {
        ekf.timeUpdate(msg->encoders.left_wheel, msg->encoders.right_wheel, msg->header.stamp.toSec());
    } catch(std::exception &e) {
        ROS_ERROR("Error with time update: %s", e.what());
    }
}

void ahrsCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    try {
        ekf.measurementUpdateAHRS(tf::getYaw(msg->orientation), msg->orientation_covariance[8]);
    } catch(std::exception &e) {
        ROS_ERROR("Error with ahrs update: %s", e.what());
    }
}

void odom_pubCallback(const ros::TimerEvent& e) {
    nav_msgs::Odometry msg;
    
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";
    msg.pose.pose.position.x = ekf.getNorthing();
    msg.pose.pose.position.y = ekf.getEasting();
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(ekf.getYaw());
    
    odom_pub.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "automow_ekf");
    
    ros::NodeHandle n;
    
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    
    ros::Subscriber encoder_subcriber = n.subscribe("encoders", 1000, encoderCallback);
    ros::Subscriber ahrs_subcriber = n.subscribe("imu/data", 1000, ahrsCallback);
    
    ros::Timer odometry_pub_timer = n.createTimer(ros::Duration(1.0/20), odom_pubCallback);
    
    ros::spin();
    
    return 0;
}