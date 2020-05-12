
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include "std_msgs/Header.h"
#include "std_msgs/Int8.h"
#include <sstream>
#include <stdio.h>
#include <math.h>

int main (int argc, char **argv) {
    

ros::init(argc, argv, "talker");
ros::NodeHandle n;
ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/perception/scan_request", 1000);
ros::Publisher chatter_pub2 = n.advertise<std_msgs::Int8>("/perception/calculate_target_trigger", 1000);
ros::Duration loop_rate(3);
int count = 0;

while (ros::ok) {
    std_msgs::String msg;
    std_msgs::Int8 msg2;
    std::stringstream ss;
    int val;
    val = count;
    int ss2 = 1;
    ss << "AL";
    msg.data = ss.str();
    msg2.data = ss2;
    ROS_INFO("%s", msg.data.c_str());
    ROS_INFO("%i", msg2.data);
    ROS_INFO("%i", val);
    chatter_pub2.publish(msg2);
    ros::spinOnce();
    sleep(5);
    chatter_pub.publish(msg);
    loop_rate.sleep();
    ros::spinOnce();
    ++count;
}

}
