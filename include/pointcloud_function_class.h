/// include PCL requirements
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl_ros/transforms.h>
#include <iostream>

/// include fundamentals
#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <math.h>
#include "std_msgs/Header.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

/// include ROS Messaging
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include "std_msgs/Int8.h"
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/concave_hull.h>


/// Primary class that holds attributes and methods to run the core logic.
class pointcloud_function_class {

public:
  int open_state;
  int publish_response;
  int cyclic_timer;
  int cam_diagnosis;
  int publish_check; /**< If 1, tell the system to output to RVIZ a cloud for visualisation. */
  float footstep_safe_size;
  bool debug;

  std::vector <float> optimal_vector;
  std::vector <std::vector <float>> sorted_publish_collection;
  std::vector <float> target_input; /**< The Target for the robot*/
  std::vector <float> range_input; /**< Will be updated based upon which leg is running. Uses [0:2] Max XYZ, [3:5] Min XYZ*/
  std::vector <float> foot_target; /**< The expected tip target from this iteration */
  std::vector <float> standard_foot; /**< The standard footprint location */
  std::string cid; /**< Cloud Frame ID */
  std::string leg_name; /**< Current leg Name */
  std::vector <std::vector<float> > target_sweep_collection; /**< All the plane mean points that fall into potential tip targets*/
  ros::Subscriber point1;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> current_clusters;
  std::vector <geometry_msgs::Point> optimal_points;
  pcl::PointCloud<pcl::PointXYZ> current_cloud;
  
  const tf::TransformListener transform_current; /**< Required to apply the base transform to the planar logic*/
  
  void generate_target_tips_from_segments();
  void publisher();
  void target_tip_optimisation();
  float two_point_distance (std::vector<float> point_a, std::vector<float> point_b);
  void bounding_box (std::vector<std::vector<std::vector<float>> >coll_corners);
  float distance_to_line (std::vector<float> test_point, std::vector< std::vector<float> > plane_corners);
  auto best_target(std::vector<std::vector<float>> potentials, std::vector<float> target, int function_return); /**< Finds the point closest to a target */
  void leg_return (const std_msgs::String::ConstPtr& message_data); /**< Checks to see which leg is active */
  void core_spin (); /**< Initialises the other functions in the correct order and spins appropriately */
  void publish_cleanup (std_msgs::Int8 message_data); /**< Memory management and publisher  */
  void pc_cyclic_control();

protected:
  ros::NodeHandle bounding_box_publisher;
  ros::NodeHandle perception_publisher;
  ros::Publisher bounding_pub = bounding_box_publisher.advertise<visualization_msgs::Marker> ( "/perception/bounding_boxes_marker", 1 );;
  ros::Publisher xyz_pub = perception_publisher.advertise< geometry_msgs::Vector3 >( "/perception/scan_result", 1);
  ros::Publisher vis_pub = perception_publisher.advertise<visualization_msgs::Marker>( "/perception/visualization_marker", 1 );
  ros::Publisher slam_pub = perception_publisher.advertise<visualization_msgs::Marker>( "/perception/slam_map_beta", 1 );
};