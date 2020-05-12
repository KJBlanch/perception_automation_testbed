#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include "std_msgs/Header.h"
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include "std_msgs/Int8.h"
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "pointcloudStitcher.h"

class mapping_class {

public:
  bool publish;
  bool debug;
  int SLAM_state;
  int error;
  int localisation;
  bool capture;
  pcl::PointCloud<pcl::PointXYZ> map;
  pcl::PointCloud<pcl::PointXYZ> map2;
  pcl::PointCloud<pcl::PointXYZ> map3;
  std::vector <geometry_msgs::Point> map_points;
  std::string leg_name; /**< Current leg Name */
  std::string cid; /**< Cloud Frame ID */
  Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();
  
  const tf::TransformListener transform_current; /**< Required to apply the base transform to the planar logic*/
  AlignmentRepresentation cloudworld;

  void mapping_publisher();
  void pico_camera_control();
  void world_init();
  void world_update();
  void cloud_storage (const pcl::PCLPointCloud2ConstPtr& cloud_input); /**< Primary function. Takes in a point cloud, saves to SLAMmap */
  void update_global_transform();
  void monstarr_data_management(const pcl::PCLPointCloud2ConstPtr& cloud_mon_input);
  pcl::PointCloud<pcl::PointXYZ> storage_management(pcl::PointCloud<pcl::PointXYZ> cloud_in);

protected:
  ros::NodeHandle mapping_publish_node;
  ros::Publisher mapping_publisher_outcloud = mapping_publish_node.advertise <pcl::PCLPointCloud2> ("/perception/3dMap",10);
    
};