#ifndef ROS_SETUP_H
#define ROS_SETUP_H

#include <ros.h>
#include <geometry_msgs/Twist.h>

unsigned long prev_ros_millis;
const unsigned long ros_period = 20; // ROS communication period [millis]

// Initialize ROS node handle, publishers, and subscribers
ros::NodeHandle nh;
geometry_msgs::Twist cmd_vel;

void RosCallBack(const geometry_msgs::Twist &twist_msg)
{
    cmd_vel.linear.x = twist_msg.linear.x;
    cmd_vel.angular.z = twist_msg.angular.z;
}

ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", RosCallBack);

void UpdateRos()
{
    // Publish and subscribe to ROS topics
    if (curr_millis - prev_ros_millis >= ros_period)
    {
        nh.spinOnce();
        prev_ros_millis = curr_millis;
    }
}

#endif
