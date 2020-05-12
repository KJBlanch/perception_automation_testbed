#include "segmentation_class.h"

void segmentation_class::cloud_segmentation() {

  //Create objects.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2::Ptr cloud_temp1 (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_temp2 (new pcl::PCLPointCloud2 ());


  //Retriever
  
  *cloud_input = this->input_cloud;
  if (this->debug == true) std::cout << "Processing " << this->input_cloud.points.size() << " points" << std::endl;
  

  this->input_cloud.width = 0;
  this->input_cloud.height = 0;
  this->input_cloud.points.clear();
  if (this->debug == true) std::cout << "Starting PointCloud Analysis" << std::endl;
  
  if (this->debug == true) std::cout << "If " << this->input_cloud.points.size() << " is equal to zero, memory management is working" << std::endl;
  
  
  
  pcl::toPCLPointCloud2(*cloud_input, *cloud_temp1);
  

  //Downsample the pointcloud
  pcl::VoxelGrid<pcl::PCLPointCloud2> ds;
  ds.setInputCloud (cloud_temp1);
  ds.setLeafSize (0.005f, 0.005f, 0.005f);
  ds.filter (*cloud_temp2);

  pcl::fromPCLPointCloud2(*cloud_temp2, *cloud);
  if (this->debug == true) std::cout << "Downsampled to " << cloud->points.size() << " points" << std::endl;

  //The remaining data structures are defined here. Note that they are all built off the PointXYZ template.
  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;

  //KD Tree is a version of the K-nearest neighbour algorithm that looks at the number of points that fall within a nearby radius.
  //Normals will use this search method to determine how many individual planes there are. It also uses normalisation to limit them to a single plane.
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  //Using the computed Normals cluster, and then runs it against the Region Growing algorithm.
  //This splits the above cloud into individual segments based off of neighbours, and split by smoothness and cuvature
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (300);
  reg.setMaxClusterSize (10000); // Total PointCloud of the camera is ~38000 without downsampling. This number should be moved to a variable that can shuffle accordingly.
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (50);
  reg.setInputCloud (cloud);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (5.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (5.0);

  //These next two lines are the only requirement to extract the cluster indices.
  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  //Use indices to build new PCL. 
  std::vector<int>::iterator logic_index;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> current_clusters_conv;
  std::vector< pcl::PointIndices >:: iterator clus;
  for (clus = clusters.begin(); clus != clusters.end(); clus++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_clusters_conv_single_clus (new pcl::PointCloud<pcl::PointXYZ>);

    for (logic_index = clus->indices.begin(); logic_index != clus->indices.end(); logic_index++) {

      float x,y,z;
        
      x = *(cloud->points[logic_index[0]].data);
      y = *(cloud->points[logic_index[0]].data + 1);
      z = *(cloud->points[logic_index[0]].data + 2);

      pcl::PointXYZ temp_point_xyz;
      temp_point_xyz.x = x;
      temp_point_xyz.y = y;
      temp_point_xyz.z = z;

      current_clusters_conv_single_clus->push_back(temp_point_xyz);  
    }
    current_clusters_conv.push_back(*current_clusters_conv_single_clus);
    current_clusters_conv_single_clus.reset(new pcl::PointCloud<pcl::PointXYZ>);
  }
  this->current_clusters = current_clusters_conv;

  //Memory Management.
  cloud_input.reset (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_temp1.reset (new pcl::PCLPointCloud2);
  cloud_temp2.reset (new pcl::PCLPointCloud2);
  tree.reset (new pcl::search::KdTree<pcl::PointXYZ>);
  normals.reset (new pcl::PointCloud <pcl::Normal>);
  cloud.reset (new pcl::PointCloud<pcl::PointXYZ>); 

}


//VISUAL METHOD - Generate a new random color for each plane. Honestly, this does make it look a little trippy - and the rand maths does not help efficiency.
    /*This has been left as is so you can see the visualisation. PCL does a terrible job of efficiency in the tutorials, as it's only meant to work on a single image,
    not real-time at 200Hz. Current logic is this -> Iterate through EVERY point in the pointcloud. Copy that to a new cloud. Iterate through every cluster.
    Iterate through every point in the cluster. Find the matching point in the new pointcloud. Change the colour. */

void segmentation_class::segment_color() {
  int point_counter = 0;
  pcl::PointCloud <pcl::PointXYZRGBA>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>); 
  std::vector<unsigned char> single_color;
  std::vector<std::vector<unsigned char>> colors;

  //Create a unique color for each plane
  for (size_t i = 0; i < this->current_clusters.size(); i++) {
      single_color.push_back (static_cast<unsigned char> (rand () % 256));
      single_color.push_back (static_cast<unsigned char> (rand () % 256));
      single_color.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back(single_color);
      single_color.clear();
    }
    
  //Iterate through the number of planes, and then iterate through each point in that plane, turning them the planes color.  
  for (int i = 0; i < this->current_clusters.size(); i++) {
    for (int i_p = 0; i_p < this->current_clusters[i].points.size(); i_p++) {
      pcl::PointXYZRGBA point;
      point.x = *(this->current_clusters[i].points[i_p].data);
      point.y = *(this->current_clusters[i].points[i_p].data + 1);
      point.z = *(this->current_clusters[i].points[i_p].data + 2);
      point.a = 255;
      point.r = colors[i][0];
      point.g = colors[i][1];
      point.b = colors[i][2]; 

      colored_cloud->points.push_back (point);
      point_counter++;
    }
  }
    //Store the cloud as a class attribute. 
  this->class_colored_cloud = *colored_cloud;
  
  if (this->debug == true) {
    std::cout << "Publishing a coloured cloud of " << colored_cloud->points.size() << " points" << std::endl;
    //this->pub = this->segment_publish.advertise <pcl::PCLPointCloud2> ("/perception/planes",10);
    
    pcl::PCLPointCloud2 outcloud; //Visualisation Cloud
    pcl::toPCLPointCloud2 (*colored_cloud, outcloud); //Converts the colored cloud to a PCL2 format for the publisher.
    outcloud.header.frame_id = "/base_link";
    
    this->pub.publish (outcloud); //Publishes the new PCL2 format
    colored_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
  }
}