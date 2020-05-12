#include "mapping_class.h"

void mapping_class::world_init () {

  this->cloudworld.param_setup();
}

void mapping_class::pico_camera_control() {
  ros::NodeHandle nh;
  ros::Subscriber point1;

  //Currently developed for four PMD cameras on a quad-legged robot. Needs to be resolved into a more flexible solution. 
  if (this->leg_name == "AL") point1 = nh.subscribe ("/pico_flexx_AL_leg/points", 1, &mapping_class::cloud_storage, this);
  if (this->leg_name == "AR") point1 = nh.subscribe ("/pico_flexx_AR_leg/points", 1, &mapping_class::cloud_storage, this);
  if (this->leg_name == "BL") point1 = nh.subscribe ("/pico_flexx_BL_leg/points", 1, &mapping_class::cloud_storage, this);
  if (this->leg_name == "BR") point1 = nh.subscribe ("/pico_flexx_BR_leg/points", 1, &mapping_class::cloud_storage, this);
  
  //subscribe to publish node
  while(this->publish == 0) ros::spinOnce();
}

void mapping_class::cloud_storage (const pcl::PCLPointCloud2ConstPtr& cloud_input) { //Takes input of PointCloud2 Datatype.
  //std::cout << "Testing Diagnosis Text - Cloud_CB Start" << std::endl;
  
  //Check to see which frame to transform to.
  
  //Stitching Method 1. 
  std::string transform_frame;
  
  if (this->localisation == 0) {
      transform_frame = "/map"; //World init, transform to world
  } else if (this->localisation == 1) {
      transform_frame = "/odom_ideal"; //No world init, transform to base_link
  } else {
      transform_frame = "/base_link"; //For further dev options.
  }
  
  //Convert and downsample to 5mm 
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_input);
  sor.setLeafSize (0.005f, 0.005f, 0.005f);
  sor.filter (*cloud_filtered);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tf (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_tf);

  //Transform to the pose of the robot
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sla (new pcl::PointCloud<pcl::PointXYZ>);
  ros::Time header_time = pcl_conversions::fromPCL(cloud_tf->header.stamp);
  this->transform_current.waitForTransform(transform_frame, cloud_tf->header.frame_id, header_time, ros::Duration(0.1));
  pcl_ros::transformPointCloud(transform_frame, *cloud_tf, *cloud_sla, this->transform_current);
  this->cid = cloud_sla->header.frame_id;



  
  if (this->map.points.size() == 0) {
    this->map = *cloud_sla;
  } else {
    this->map = this->map + *cloud_sla;

  }

//Stitching Method 2


//Memory Management
  //cloud_conv.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_filtered.reset(new pcl::PCLPointCloud2 ());
  cloud_tf.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_sla.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

pcl::PointCloud<pcl::PointXYZ> mapping_class::storage_management(pcl::PointCloud<pcl::PointXYZ> cloud_in) {
  //Downsample all points.
  pcl::PCLPointCloud2::Ptr cloud_s_m (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_s_m_update (new pcl::PCLPointCloud2 ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s_m_update_conv (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::toPCLPointCloud2(cloud_in, *cloud_s_m);
  
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_s_m);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_s_m_update);
  
  pcl::fromPCLPointCloud2(*cloud_s_m_update, *cloud_s_m_update_conv);

  return(*cloud_s_m_update_conv);

  cloud_s_m.reset(new pcl::PCLPointCloud2 ());
  cloud_s_m_update.reset(new pcl::PCLPointCloud2 ());
  cloud_s_m_update_conv.reset(new pcl::PointCloud<pcl::PointXYZ>);

}

void mapping_class::update_global_transform() {
  ros::NodeHandle monstarr_node;
  ros::Subscriber point2;
  this->capture = true;
  while (this->capture) {
    point2 = monstarr_node.subscribe ("/pico_flexx_body/points", 1, &mapping_class::monstarr_data_management, this);
    ros::spinOnce();
  }
}

void mapping_class::monstarr_data_management(const pcl::PCLPointCloud2ConstPtr& cloud_mon_input) {
  if (this->capture) {
  this->capture = false;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conv_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conv (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_man (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::fromPCLPointCloud2(*cloud_mon_input, *cloud_conv_in);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sla (new pcl::PointCloud<pcl::PointXYZ>);
  ros::Time header_time = pcl_conversions::fromPCL(cloud_mon_input->header.stamp);
  tf::TransformListener mon_transform;
  mon_transform.waitForTransform("/odom_ideal", cloud_mon_input->header.frame_id, header_time, ros::Duration(0.1));
  pcl_ros::transformPointCloud("/odom_ideal", *cloud_conv_in, *cloud_conv, mon_transform);

   
  //Mode 1 - Monstarr Stitching. 
  if (this->map2.points.size() == 0) {
    std::cout << "Initialising Monstar" << std::endl;
    this->map2 = *cloud_conv;
    this->map3 = *cloud_conv;
  //Mode 2  
  } else {
    std::cout << "Updating Global Map Via Monstarr" << std::endl;
    Eigen::Matrix4f current_transform;
    current_transform = this->cloudworld.stitcher_trigger(this->map2, *cloud_conv);
    this->global_transform *= current_transform;
    
    std::cerr << current_transform << std::endl;
    std::cerr << this->global_transform << std::endl;
    
    //cloud_man.header.frame_id = "odom_ideal";
    pcl::transformPointCloud(*cloud_conv, *cloud_man, this->global_transform);
    this->map3 = this->map3 + *cloud_conv;
    cloud_man.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    //this->map3 = mapping_class::storage_management(this->map3);
    this->cloudworld.save = false;
    this->map2 = *cloud_conv;
    cloud_conv.reset(new pcl::PointCloud<pcl::PointXYZ>);

  }

  if (this->debug) {
    pcl::PCLPointCloud2 current_map; //Visualisation Cloud
    pcl::toPCLPointCloud2 (this->map3, current_map); //Converts the colored cloud to a PCL2 format for the publisher.
    //transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    current_map.header.frame_id = "odom_ideal";
    this->mapping_publisher_outcloud.publish (current_map); //Publishes the new PCL2 format
  }
  }
}