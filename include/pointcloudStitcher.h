

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>


//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// Define a new point representation for < x, y, z, curvature >
class AlignmentRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:

  bool downsample;
  bool save = true;
  double voxel_size;
  int kSearch;
  double maxCorrespondenceDistance;
  double transformationEpsilon;
  int iterations;

  void param_setup();
  Eigen::Matrix4f stitcher_trigger(PointCloud source, PointCloud target);
  void pairAlign(PointCloud cloud_src, PointCloud cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);
  

  AlignmentRepresentation(){
    nr_dimensions_ = 4;
  }

    // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray(const PointNormalT &p, float * out) const{
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};