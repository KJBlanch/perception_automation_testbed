#include <iostream>
#include <ros/ros.h>
#include <future>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include "std_msgs/Int8.h"
#include <ros/spinner.h>
#include <ros/callback_queue.h>

#include "mapping_class.h"
#include "pointcloud_function_class.h"
#include "segmentation_class.h"

#include "leg_ranges.h"

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////
This class controls the mapping, segmentation and foot-placement selection classes, as well as the timings
required for the system to function. 



/////////////////////////////////////////////////////////////////////////////////////////////////////////*/

class controller {

public:
  bool debug;
  bool mapping;
  mapping_class active_map;
  int open_state;
  std::vector <leg_ranges> all_legs_ranges; /**< Vector made up of leg_ranges objects. One for each leg */
  float footstep_safe_size;

  
  void ros_setup();
  void leg_sweep_start (const std_msgs::String::ConstPtr& message_data);
  void leg_sweep_end (std_msgs::Int8 message_data);
};