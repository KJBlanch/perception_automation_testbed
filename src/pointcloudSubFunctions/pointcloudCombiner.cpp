#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/io/pcd_io.h>
    
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

tf2_ros::Buffer tf_buffer_;
const std::string target_frame_ = "base_link";


sensor_msgs::PointCloud2 cloud_AL;
sensor_msgs::PointCloud2 cloud_AR;
sensor_msgs::PointCloud2 cloud_BL;
sensor_msgs::PointCloud2 cloud_BR;



void cloudALCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(2));

        tf2::doTransform(*msg, cloud_AL, transform);

    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
}

void cloudARCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(2));

        tf2::doTransform(*msg, cloud_AR, transform);

    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
}

void cloudBLCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(2));

        tf2::doTransform(*msg, cloud_BL, transform);

    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
}

void cloudBRCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(2));

        tf2::doTransform(*msg, cloud_BR, transform);

    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloudCombiner");
    ros::NodeHandle nh;
    tf2_ros::TransformListener tfListener(tf_buffer_);

    ros::Subscriber al_sub = nh.subscribe<sensor_msgs::PointCloud2>("/pico_flexx_AL_leg/points", 1, cloudALCallback);
    ros::Subscriber ar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/pico_flexx_AR_leg/points", 1, cloudARCallback);
    ros::Subscriber bl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/pico_flexx_BL_leg/points", 1, cloudBLCallback);
    ros::Subscriber br_sub = nh.subscribe<sensor_msgs::PointCloud2>("/pico_flexx_BR_leg/points", 1, cloudBRCallback);
    
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("combined_cloud", 1);

    ros::Rate rate(10);
    while(nh.ok()){
        sensor_msgs::PointCloud2 cloud_out;

        // Note: Combining Clouds seems to be quite slow. Might need to look into this at a later point.
        pcl::concatenatePointCloud(cloud_out, cloud_AL, cloud_out);
        pcl::concatenatePointCloud(cloud_out, cloud_AR, cloud_out);
        pcl::concatenatePointCloud(cloud_out, cloud_BL, cloud_out);
        pcl::concatenatePointCloud(cloud_out, cloud_BR, cloud_out);

        cloud_pub.publish(cloud_out);

        ros::spinOnce();
        rate.sleep();

    }
}


