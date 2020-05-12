#include "controller.h"


/// This spins up the core ROS functions. 
void controller::ros_setup() {
  
  //Initialise Node
  ros::NodeHandle nh_2;
  
  std::cout << "Core Check 1 - Node Initialised" << std::endl;
  
  //Build Scan Request Subscriber
  ros::Subscriber request = nh_2.subscribe ("/perception/scan_request", 1, &controller::leg_sweep_start, this);
  std::cout << "Core Check 2 - Scan Request Subscriber Built" << std::endl;
  
  //Build the Event Handler Subscriber
  ros::Subscriber point5 = nh_2.subscribe ("/perception/calculate_target_trigger", 1, &controller::leg_sweep_end, this);
  std::cout << "Core Check 3 - Event Handler Subscriber Built" << std::endl;

  //Build the Publishers
  std::cout << "Core Check 4 - Publishers Built" << std::endl;

  this->active_map.world_init();

  std::cout << "Core Check 5 - Map Initialised" << std::endl;

  std::cout << "All core checks complete. Spinning" << std::endl;

  //Spin
  ros::spin();
}


void controller::leg_sweep_start (const std_msgs::String::ConstPtr& message_data) {
  if (this->mapping == false) {
    this->active_map.map.points.clear();
    this->active_map.localisation = 2;
    std::string current_leg;
    current_leg = message_data->data.c_str();
    this->active_map.publish = 0;
    this->active_map.leg_name = current_leg;
    if (this->debug == true) {
      ROS_INFO_STREAM(current_leg);
      this->active_map.debug = true;
    }
    this->open_state = 1;
    this->active_map.pico_camera_control();
    this->active_map.update_global_transform();
  }
} 

/// This function executes when recieving a messate that the leg has finished the scan. 
void controller::leg_sweep_end (std_msgs::Int8 message_data) {
  if (message_data.data == 1) {
    ROS_INFO("Calling Monstarr");


    if (this->debug == true) ROS_INFO_STREAM(message_data);
    if (this->open_state == 1) {
      this->open_state = 0;

      this->active_map.publish = 1;
      if (this->debug == true) std::cout << "Segmenting" << std::endl;
      segmentation_class map_segmenter;
      map_segmenter.input_cloud = this->active_map.map;
      map_segmenter.cloud_segmentation();

      if (this->debug == true) {
        map_segmenter.debug = true;
        if (this->debug == true) std::cout << "Publishing Segments" << std::endl;
        map_segmenter.segment_color();
      }

      pointcloud_function_class delegate; 
      delegate.footstep_safe_size = this->footstep_safe_size;

      if (this->debug ==true) delegate.debug = true;

      for (int i = 0; i <= this->all_legs_ranges.size(); i++) {
        if (this->all_legs_ranges[i].name == this->active_map.leg_name) {
          delegate.range_input = this->all_legs_ranges[i].range;
          delegate.standard_foot = this->all_legs_ranges[i].target;
        } 
      }

      delegate.current_clusters = map_segmenter.current_clusters;
      delegate.current_cloud = map_segmenter.input_cloud;
      if (this->debug == true) std::cout << "Generating Potential Targets" << std::endl;
      
      delegate.generate_target_tips_from_segments();
      
      delegate.target_tip_optimisation();
    
      if (delegate.sorted_publish_collection.size() != 0) {
        if (this->debug == true) std::cout << "Publishing Targets" << std::endl;
        delegate.publisher();
        if (this->debug == true) std::cout << "Completed. Idling." << std::endl;
      } else {
        if (this->debug == true) std::cout << "No safe positions found within clusters. Awaiting new scan" << std::endl;
      }

      

      if (this->debug == true) std::cout << delegate.target_sweep_collection.size() << " " << message_data.data << std::endl;
      
      
      //Check to see if the sweep has been completed.
    }
  }

  

}