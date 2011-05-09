#include "ros/ros.h"

#include <sstream>

#include "automow_ekf.h"
#include "ax2550/StampedEncoders.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "magellan_dg14/UTMFix.h"
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
        // Must subtract pi/2 to the yaw, inorder to rotate it into the body fixed frame
        ekf.measurementUpdateAHRS(tf::getYaw(msg->orientation)-M_PI/2.0, msg->orientation_covariance[8]);
    } catch(std::exception &e) {
        ROS_ERROR("Error with ahrs update: %s", e.what());
    }
}

void gpsCallback(const magellan_dg14::UTMFix::ConstPtr& msg) {
    try {
        // ekf.measurementUpdateGPS(msg->northing, msg->easting, msg->position_covariance[0], msg->position_covariance[3]);
    } catch(std::exception &e) {
        ROS_ERROR("Error with ahrs update: %s", e.what());
    }
}

void odom_pubCallback(const ros::TimerEvent& e) {
    nav_msgs::Odometry msg;
    
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";
    msg.pose.pose.position.y = ekf.getNorthing();
    msg.pose.pose.position.x = ekf.getEasting();
    double yaw = ekf.getYaw();
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    // std::cout << msg.pose.pose.position.y << "," << msg.pose.pose.position.x << "," << yaw << std::endl;
    
    odom_pub.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "automow_ekf");
    
    // std::cout << "easting,northing,yaw" << std::endl;
    // std::cout << "easting,northing,yaw // No IMU" << std::endl;
    // std::cout << "imu_measurement,imu_measurement_wrapped,imu_prediction,imu_innovation, imu_innovation_wrapped" << std::endl;
    std::cout << "states e,n,t,rl,rr,wb,E,N,T" << std::endl;
    
    ros::NodeHandle n;
    
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    
    ros::Subscriber encoder_subcriber = n.subscribe("encoders", 1000, encoderCallback);
    ros::Subscriber ahrs_subcriber = n.subscribe("imu/data", 1000, ahrsCallback);
    
    ros::Timer odometry_pub_timer = n.createTimer(ros::Duration(1.0/20), odom_pubCallback);
    
    ros::spin();
    
    return 0;
}