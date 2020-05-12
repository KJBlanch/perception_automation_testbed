#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/ModelCoefficients.h>
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
#include <pcl/point_types.h>
#include <iostream>
#include "std_msgs/Header.h"
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include "std_msgs/Int8.h"

class segmentation_class {

public:

  bool debug;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> current_clusters;
  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  pcl::PointCloud <pcl::PointXYZRGBA> class_colored_cloud;

  void cloud_segmentation();
  void segment_color();
  
protected:
    ros::NodeHandle segment_publish;
    ros::Publisher pub = segment_publish.advertise <pcl::PCLPointCloud2> ("/perception/planes",10);
    
};