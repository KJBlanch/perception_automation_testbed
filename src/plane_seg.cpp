#include "plane_seg.h"
#include "controller.h"

/**
 * This system is designed specifically for a multi-legged robot. Please see Readme.MD for more details. 
 * Built from PCL Tutorial - http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php - Although at this point it's nearly unrecognisable.

*/

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
#include <std_msgs/String.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>



int
main (int argc, char** argv)
{
  // Initialize ROS
  
  std::cout << "Spin Start" << std::endl;
  ros::init (argc, argv, "planes");
  std::cout << "Ros Init" << std::endl;
  ros::NodeHandle init;
  std::vector<float> target;
  std::vector<float> range;

  //HardCoded Ranges for each leg.
  leg_ranges al, ar, bl, br;

  //Quadrant is xpos-for, ypos-lef, zpos-up - ROS Standard.

  std::vector<float> AR_target, AL_target, BR_target, BL_target, AR_range, AL_range, BR_range, BL_range;


  if (init.getParam("/syropod/parameters/AR_target", AR_target)) {
    ar.target = AR_target;
  } else {
    std::cout << "No AR target located. Defaulting.." << std::endl;
    ar.target = {0.21,-0.2, -0.05};
  }
  if (init.getParam("/syropod/parameters/AL_target", AL_target)) {
    al.target = AL_target;
  } else {
    std::cout << "No AL target located. Defaulting.." << std::endl;
    al.target = {0.21, 0.2, -0.05};
  }
  if (init.getParam("/syropod/parameters/BR_target", BR_target)) {
    br.target = BR_target;
  } else {
    std::cout << "No BR target located. Defaulting.." << std::endl;
    br.target = {-0.055, -0.22, -0.05};
  }
  if (init.getParam("/syropod/parameters/BL_target", BL_target)) {
    bl.target = BL_target;
  } else {
    std::cout << "No BL target located. Defaulting.." << std::endl;
    bl.target = {-0.055, 0.22, -0.05};
  }


  if (init.getParam("/syropod/parameters/AR_range", AR_range)) {
    ar.range = AR_range;
  } else {
    std::cout << "No AR range located. Defaulting.." << std::endl;
    ar.range = {0.28, -0.06, 0.20, 0.11, -0.30, -0.18};
  }
  if (init.getParam("/syropod/parameters/AL_range", AL_range)) {
    al.range = AL_range;
  } else {
    std::cout << "No AL range located. Defaulting.." << std::endl;
    al.range = {0.28, 0.30, 0.20, 0.11, 0.06, -0.18};
  }
  if (init.getParam("/syropod/parameters/BL_range", BL_range)) {
    bl.range = BL_range;
  } else {
    std::cout << "No BL range located. Defaulting.." << std::endl;
    bl.range = {-0.04, 0.30, 0.20, -0.25, 0.06, -0.18};
  }
  if (init.getParam("/syropod/parameters/BR_range", BR_range)) {
    br.range = BR_range;
  } else {
    std::cout << "No BR range located. Defaulting.." << std::endl;
    br.range = {-0.04, -0.06, 0.20, -0.25, -0.30, -0.18};
  }

  ar.name = "AR";
  al.name = "AL";
  br.name = "BR";
  bl.name = "BL";
  
  std::vector <leg_ranges> all_legs;
  all_legs = {al, ar, bl, br};
  
  std::cout << "Validating Data.." << std::endl;
  for (int i = 0; i <= all_legs.size()-1; i++) {
    if ((all_legs[i].range.size() == 6) && (all_legs[i].target.size() == 3)) {
      std::cout << all_legs[i].name << " data validated. Continuing.. " << std::endl;
    } else {
      std::cout << all_legs[i].name << " data incorrect. Exiting" << std::endl;
      exit;
    }
  }
  
  controller holder;
  std::cout << "Class Created" << std::endl;
  
  holder.all_legs_ranges = all_legs;
  
  float footstep_size;

  if (init.getParam("/syropod/parameters/footstep_size", footstep_size)) {
    std::cout << "Footstep size located. Size = " << footstep_size << std::endl;
    holder.footstep_safe_size = footstep_size;
  } else {
    std::cout << "No footstep size located. Defaulting.." << std::endl;
    holder.footstep_safe_size = 0.027;
  }
  holder.mapping = false;
  holder.debug = true;
  
  std::cout << "Init Complete. Starting core functions.." << std::endl;
  //Run Everything!! DV.
  holder.ros_setup();
}
