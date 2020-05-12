#include "pointcloud_function_class.h"

float pointcloud_function_class::two_point_distance (std::vector<float> point_a, std::vector<float> point_b) {
  float distance = sqrt((pow((point_a[0]-point_b[0]),2)) + (pow((point_a[1]-point_b[1]),2)) + (pow((point_a[2]-point_b[2]),2)));
  return (distance);
}

///This function simply takes the two sets of corners (Outer corners of the plane, and the inner corners of the safe foot placement area) and returns
/// a marker topic for each. The outside box will be red, and the inside box will be gree (showing the safe placement for the foot. )
void pointcloud_function_class::bounding_box (std::vector< std::vector<std::vector<float> >> coll_corners) {
  //bounding_pub = bounding_box_publisher.advertise<visualization_msgs::Marker> ( "/perception/bounding_boxes_marker", 1 );
  //Build objects. 
  //Build lines
  
  //Display Lines
  for (int i = 0; i < coll_corners.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "Bounding_boxes";
    marker.id = i+1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(10);
    
    for (int it = 0; it < coll_corners[i].size(); it++) {
      if (coll_corners[i][it].size() != 0) {
        //std::cout << "Point pushed" << std::endl;
        geometry_msgs::Point temp_point;
        temp_point.x = coll_corners[i][it][0];
        temp_point.y = coll_corners[i][it][1];
        temp_point.z = coll_corners[i][it][2];
        marker.points.push_back(temp_point);
      }
    }
    if (coll_corners[i][0].size() != 0) {
      geometry_msgs::Point end_point;
      end_point.x = coll_corners[i][0][0];
      end_point.y = coll_corners[i][0][1];
      end_point.z = coll_corners[i][0][2];
      marker.points.push_back(end_point);
    }
    marker.scale.x = 0.005;
    
    marker.color.a = 1.0; 
    if ((i == 0) || (remainder(i, 2)) == 0) {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
    } else {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
    }
    marker.color.b = 0.0;
    std::cout << "Publishing Bounding Box" << std::endl;
    bounding_pub.publish( marker );
  }
  
}


///This function takes a test_point (XY) and returns the distance to lines made from four corners. 
///This function will need to be redone to do n numbers of corners, instead of the limit to 4.
float pointcloud_function_class::distance_to_line (std::vector<float> test_point, std::vector< std::vector<float> > plane_corners) {

  //Input Validation. 

  if (plane_corners.size() > 1) {
    
    //Build objects and determine lines. . 
    float distance;

    std::vector <std::vector <float> > line;
    
    std::vector<float> temp_line; 


    //std::cout << "DTL start it1" << std::endl;
    for (int it = 0; it < plane_corners.size(); it++) {
      if (it == plane_corners.size()-1) {
        temp_line = {plane_corners[it][0], plane_corners[it][1], plane_corners[0][0], plane_corners[0][1]};
        line.push_back(temp_line);
      } else {
        temp_line = {plane_corners[it][0], plane_corners[it][1], plane_corners[it+1][0], plane_corners[it+1][1]};
        line.push_back(temp_line);
      }
      
    }
    std::vector <float> distance_list;

    ///check to see point is within plane. 
    //std::cout << "DTL start it2" << std::endl;
    //reorganise corners.
    std::vector <float> verty, vertx;

    for (int it = 0; it < plane_corners.size(); it++) {
      if (plane_corners[it].size() > 1) {
        vertx.push_back(plane_corners[it][0]);
        verty.push_back(plane_corners[it][1]);
      }
    }

    if (plane_corners[0].size() > 1) {
      vertx.push_back(plane_corners[0][0]);
      verty.push_back(plane_corners[0][1]);
    }

    //Apply ray tracing - This creates an almost infinite 'ray' that starts at the point and travels along the x-axis. If it intersects a line, it flips 'c', (0 or 1). 
    //If it only intersects 1 line, it is within the plane and will return '1'. (Assuming plane is a square). However, the flip method will be good when this formula will be migrated to complex polygons. 
    int i, j, c = 0;
    //std::cout << "DTL start it3" << std::endl;
    for (i = 0, j = vertx.size()-1; i < vertx.size(); j = i++) {
      if ( ((verty[i]>test_point[1]) != (verty[j]>test_point[1])) && (test_point[0] < (vertx[j]-vertx[i]) * (test_point[1]-verty[i]) / (verty[j]-verty[i]) + vertx[i]) ) c = !c;
      //if (c == 1) std::cout << "FLIPPED" << std::endl;
    }
    //if point not in polygon, throw a negative value exception.
    if ( c == 0) {
      return(-1);
    }
    
    //Check to see how close the test point is to each line. Line identified as point A and B. Test point is Point C. Formula is;
    // ((Cx - Ax)*(By - Ay))- ((Cy - Ay)*(Bx-Ax))/sqrt(Bx - Ax)^2 + (By-Ay)^2
    //Add distance to list of distances for each line. 
    
    //std::cout << "DTL start it4" << std::endl;
    for (int it = 0; it < line.size(); it++) 
    {
      if (line[it].size() == 4) {
        float normalLength = hypot(line[it][2] - line[it][0], line[it][3] - line[it][1]);
        float current_distance = (float)((test_point[0] - line[it][0]) * (line[it][3] - line[it][1]) - (test_point[1] - line[it][1]) * (line[it][2] - line[it][0])) / normalLength;
        if (current_distance < 0) current_distance = -current_distance;
        distance_list.push_back(current_distance);
      }
    }

    //Iterate through each line to check which one is the shortest. Return the shortest. 
    if (distance_list.size() != 0) {
      distance = distance_list[0];
      //std::cout << "DTL start it5" << std::endl;
      for (int it = 0; it < distance_list.size(); it++) {
        if (distance > distance_list[it]) distance = distance_list[it];
      }
      //std::cout << "Checking Distance" << std::endl;
      //if (distance > this->footstep_safe_size) std::cout << distance << std::endl;
      return(distance);
    } else {
      return(-3);
    } 
  } else {
      //If input validation fails, throw a negative value exception. 
      //std::cout << "Not enough corners!" << std::endl;
    return(-2);
  }
}

/// This function looks at a group of potential targets (represented as xyz) and then iterated through to find out which one is closest to the target. 
auto pointcloud_function_class::best_target (std::vector<std::vector<float>> potentials, std::vector<float> target, int function_return) {
  //std::cout << "Starting BT Algorithm.." << std::endl;
  int i;
  float closest_distance;
  std::vector<float> returning_target;
  std::vector<float> returning_index = {0};
  //std::cout << potentials.size() << " " << target.size() << potentials[0].size() << std::endl;
  if ((potentials.size() != 0) && (target.size() == 3)) {
    returning_target = potentials[0];
    closest_distance = sqrt((pow((target[0]-potentials[0][0]),2)) + (pow((target[1]-potentials[0][1]),2)) + (pow((target[2]-potentials[0][2]),2)));
    //std::cout << "Built zero for comparison. Unpacking Potentials.." << std::endl;
    for (i = 0; i <= potentials.size()-1; i++) {
      float distance_x, distance_y, distance_z, total_distance;
      std::vector <float> frame;
      
      //Unpack the xyzs.
      frame = potentials[i];
       
      //std::cout << "Iterating through Potential " << i << std::endl; //FOR TESTING
      
      // Highschool Trig time. Total distance between two points can be represented by the following;
      // d2-d1 = sqrt( ((x2-x1)^2) + ((y2-y1)^2) + ((z2-z1)^2) ) 
      // This is pythagoras theory of a 3d space, which is just an expansion of a^2 + b^2 = c^2. 

      total_distance = sqrt((pow((target[0]-frame[0]),2)) + (pow((target[1]-frame[1]),2)) + (pow((target[2]-frame[2]),2)));
       
      if (total_distance < closest_distance) {
        closest_distance = total_distance;
        returning_target = frame;
        returning_index[0] = i;
        //std::cout << "New Best" << std::endl; //FOR TESTING
      }
    }
    //std::cout << "Completed BT" << std::endl;
    if (function_return == 0) return (returning_target);
    if (function_return == 1) return (returning_index);
  } else {
    //std::cout << "Breaking. BT Input Error" << std::endl;
  }
}




/// This is the primary function of the pointcloud_function_class. It is a smidge convulted. Each section is commented accordingly. 

void pointcloud_function_class::generate_target_tips_from_segments() {
  
  this->target_sweep_collection.clear();

  if (this->debug == true) std::cout << "Determining Positions..." << std::endl;
  
  if (this->cam_diagnosis == 1) {
    this->cam_diagnosis = 0;

    //Input target Error Check
    if (this->standard_foot.size() == 3) {
      std::cout << "Optimal Foot Placement Updated.." << std::endl;
    } else {
      std::cout << "Incorrect Optimal Placement Input. Size is " << this-standard_foot.size() << " out of 3. Breaking" << std::endl;
    return;
    }

    //input Range Error Check
    if (this->range_input.size() == 6) {
      std::cout << "Foot Range Updated.." << std::endl;
    } else {
      std::cout << "Incorect Ranging. Size is " << this-range_input.size() << " out of 6. Breaking" << std::endl;
      return;
    }
  
  }
    
  //Rebuilding range into an easier to read format.
  float max_x, max_y, max_z, min_x, min_y, min_z;
  max_x = this->range_input[0]; 
  max_y = this->range_input[1];
  max_z = this->range_input[2];
  min_x = this->range_input[3];
  min_y = this->range_input[4];
  min_z = this->range_input[5];


  //Retrieval. 
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  cloud = this->current_cloud;
  clusters = this->current_clusters;
  
  //std::cout << "Number of Clusters : " << clusters.size()<< std::endl; //For Testing - this will show the number of planes.

  std::cout << clusters.size() << " of clusters to process" << std::endl;
  
  if (clusters.size() != 0) {
    //This section builds the pointcloud and colours required for the visualisation. 
   
  
    std::vector< std::vector<std::vector<float> >> corners_array;
  
    //Before we start making the colours all pretty, we should perform and output the navigation logic;
    std::vector <float> closest_xyz;
    int counter = 0;

    //Iterate through the clusters.
    //TODO Break clusters up into smaller segments. Lot of iteration here. Gonna slow right down. Still better than the visualisation cloud_point.
    for (int clus = 0; clus < clusters.size(); clus++) {
      std::vector< std::vector <float> > potential_vectors;
      
      counter++;
      std::cout << "Processing Cluster " << counter << "..." << std::endl;
      int logic_index;


      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
      *cloud_filtered = clusters[clus];

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
      
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.01);
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      pcl::ProjectInliers<pcl::PointXYZ> proj;
      proj.setModelType (pcl::SACMODEL_PLANE);
      proj.setInputCloud (cloud_filtered);
      proj.setModelCoefficients (coefficients);
      proj.filter (*cluster_cloud);


      pcl::ConcaveHull<pcl::PointXYZ> chull;
      chull.setAlpha(0.1);
      chull.setInputCloud (cluster_cloud);

      chull.reconstruct (*cloud_hull);

      std::vector<std::vector <float> > corners;
      if (cloud_hull->size() < 700) {
        for (int it = 0; it < cloud_hull->points.size(); it++) {
          float hull_x, hull_y, hull_z;
          hull_x = cloud_hull->points[it].x;
          hull_y = cloud_hull->points[it].y;
          hull_z = cloud_hull->points[it].z;
          std::vector <float> corner_point_collate = {hull_x, hull_y, hull_z};
          corners.push_back(corner_point_collate);
        }
        
        std::vector<std::vector <float> > corners_sorted;
        for (int it = corners.size()-1; it >= 0; it--) {
          if (it == corners.size()-1) {
            //std::cout << "Starting" << std::endl;
            corners_sorted.push_back(corners[it]);
            corners.erase(corners.begin() + it);
          } else {
            corners_sorted.push_back(pointcloud_function_class::best_target(corners, corners_sorted[it-1], 0));
            corners.erase(corners.begin() + pointcloud_function_class::best_target(corners, corners_sorted[it-1], 1)[0]);
          }
        }
      

        //Collate the corners. 
        
        /*
        for (int i = 0; i < corners_sorted.size(); i++) {
          std::cout << "Outside corners  " << i << " " << corners_sorted[i][0] << " " << corners_sorted[i][1] << " " << corners_sorted[i][2] << std::endl;
        }
        */
        std::cout << "All corners determined" << std::endl;

        int safe_corner_check = 0;

        std::vector <float> safe_corner1;
        std::vector <float> safe_corner2;
        std::vector <float> safe_corner3;
        std::vector <float> safe_corner4;
      
        int b_box_print = 0;

        //Iterate through the points to find the safe corners (Assumption is that it is 2d, and only has 4 corners!!)


        pcl::PointCloud<pcl::PointXYZ>::Ptr safe_cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr RS_safe_cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        for (logic_index = 0; logic_index < cluster_cloud->points.size(); logic_index++) {
          
          float x,y,z;
            
          x = cluster_cloud->points[logic_index].x;
          y = cluster_cloud->points[logic_index].y;
          z = cluster_cloud->points[logic_index].z;

          pcl::PointXYZ temp_safe_point_xyz;
          temp_safe_point_xyz.x = x;
          temp_safe_point_xyz.y = y;
          temp_safe_point_xyz.z = z;

          std::vector <float> temp_vect;
          temp_vect = {x,y,z};
          //std::cout << "Testing safe bounds" << std::endl;
          //If the point is within safe distance from the edge made from the previous corners, check to see if it is within a safe z distance and if its a corner value. 
          
          if ((pointcloud_function_class::distance_to_line(temp_vect, corners_sorted) >= this->footstep_safe_size) && (min_z < z) && (max_z > z)) {
            safe_cluster_cloud->push_back(temp_safe_point_xyz);
            
          }

        }

        std::vector<std::vector <float> > safe_corners;
        pcl::PointCloud<pcl::PointXYZ>::Ptr safe_cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
        
        if (safe_cluster_cloud->size() != 0) {

          pcl::SACSegmentation<pcl::PointXYZ> seg2;
        
          seg2.setOptimizeCoefficients (true);
          seg2.setModelType (pcl::SACMODEL_PLANE);
          seg2.setMethodType (pcl::SAC_RANSAC);
          seg2.setDistanceThreshold (0.01);
          seg2.setInputCloud (safe_cluster_cloud);
          seg2.segment (*inliers, *coefficients);
          pcl::ProjectInliers<pcl::PointXYZ> proj2;
          proj2.setModelType (pcl::SACMODEL_PLANE);
          proj2.setInputCloud (safe_cluster_cloud);
          proj2.setModelCoefficients (coefficients);
          proj2.filter (*RS_safe_cluster_cloud);

          for (int it = 0; it < RS_safe_cluster_cloud->points.size(); it++) {
            float safe_x, safe_y, safe_z;
            safe_x = RS_safe_cluster_cloud->points[it].x;
            safe_y = RS_safe_cluster_cloud->points[it].y;
            safe_z = RS_safe_cluster_cloud->points[it].z;
            std::vector<float> safe_temp_vect = {safe_x, safe_y, safe_z};
            potential_vectors.push_back(safe_temp_vect);
          }

          pcl::ConvexHull<pcl::PointXYZ> safe_chull;
          safe_chull.setInputCloud (RS_safe_cluster_cloud);
          safe_chull.reconstruct (*safe_cloud_hull);

          

          for (int it = 0; it < safe_cloud_hull->points.size(); it++) {
            float hull_x, hull_y, hull_z;
            hull_x = safe_cloud_hull->points[it].x;
            hull_y = safe_cloud_hull->points[it].y;
            hull_z = safe_cloud_hull->points[it].z;
            std::vector <float> safe_corner_point_collate = {hull_x, hull_y, hull_z};
            safe_corners.push_back(safe_corner_point_collate);
          }
        }


        //Colates the two sets of corners for visualisation and publication. 
        std::cout << "Found Bounds " << corners_sorted.size() << " " << safe_corners.size() << std::endl;
        
        //if (b_box_print == 1) {
          //Build safe sector
        if ((corners_sorted.size() != 0) && (safe_corners.size() != 0)) {
          corners_array.push_back(corners_sorted);
          corners_array.push_back(safe_corners);
        }
      
        //Determine the best point if the optimal foot placement is not within a plane.
        if (potential_vectors.size() != 0) {
          for (int i = 0; i < potential_vectors.size(); i++) {
            this->target_sweep_collection.push_back(potential_vectors[i]);
          }
              
          std::cout << potential_vectors.size() << " Safe points within Range" << std::endl;
        
        }

        RS_safe_cluster_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        safe_cluster_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        safe_cloud_hull.reset(new pcl::PointCloud<pcl::PointXYZ>);
      }

      cluster_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
      cloud_hull.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }  
      
    if (corners_array.size() != 0) {
      if (this->debug == true) pointcloud_function_class::bounding_box(corners_array);
      corners_array.clear();
    
    
    }

  }
}


void pointcloud_function_class::target_tip_optimisation() {
  std::cout << "Recieved Step Request" << std::endl;
  std::vector <float> foot_target;
  std::vector <std::vector <float>> publish_collection;

  if ((this->target_sweep_collection.size() != 0) && (this->standard_foot.size() == 3)) {
      foot_target = pointcloud_function_class::best_target(this->target_sweep_collection, this->standard_foot, 0);
      publish_collection.push_back(foot_target);
      //std::cout << "Found best target. Sorting remaining.." << std::endl;
      int checker = 0;
      int counter = 0;

      while ((this->target_sweep_collection.size() > 10) && (counter < 8000)) {

      for (int i = this->target_sweep_collection.size()-1; i >= 0; i--) {
          for (auto it = 0; it < publish_collection.size(); it++) {
          if (pointcloud_function_class::two_point_distance(this->target_sweep_collection[i], publish_collection[it]) < this->footstep_safe_size) {
              this->target_sweep_collection.erase(this->target_sweep_collection.begin() + i);
              //std::cout << this->target_sweep_collection.size() << " Remaining" << std::endl;
              counter++;
              break;
          }
          }
      }
      if (this->target_sweep_collection.size() != 0) {
          float foot_index = pointcloud_function_class::best_target(this->target_sweep_collection, this->standard_foot, 1)[0];       
          publish_collection.push_back(this->target_sweep_collection[foot_index]);
          this->target_sweep_collection.erase(this->target_sweep_collection.begin() + foot_index);
          counter++;
          //std::cout << "Point Added" << std::endl;
          //std::cout << this->target_sweep_collection.size() << " Remaining" << std::endl;
      } else {
        counter == 8001;
      }
      }

      std::cout << "Positions Determined" << std::endl;
      this->target_sweep_collection.clear();
  } else {
    std::cout << "Debug Issue. Shouldn't be able to get here" << std::endl;
  }
  if (publish_collection.size() != 0) {
    this->sorted_publish_collection = publish_collection;
  }
}

void pointcloud_function_class::publisher () {
    //Create the RVIZ marker.
  if (this->debug == true) std::cout << "Building Markers..." << std::endl;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_link";
  marker.ns = "marker_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point temp_p;
  marker.scale.x = 0.005;
  marker.scale.y = 0.005;
  marker.scale.z = 0.005;
  marker.color.g = 0;
  marker.color.r = 1;
  marker.color.b = 0;
  marker.color.a = 1.0;

if (this->debug == true) std::cout << "Building Optimal Points Markers.." << std::endl;
  //Build the Vector3 Message
  std::vector <geometry_msgs::Vector3> publish_v3;
  if (this->sorted_publish_collection.size() != 0) {
    for (int it = 0; it < this->sorted_publish_collection.size()-1; it++) {
      if (this->sorted_publish_collection[it].size() == 3) {
        geometry_msgs::Vector3 foot_target_v3;
        temp_p.x = this->sorted_publish_collection[it][0];
        temp_p.y = this->sorted_publish_collection[it][1];
        temp_p.z = this->sorted_publish_collection[it][2];
        marker.points.push_back(temp_p);

        this->optimal_points.push_back(temp_p);

        foot_target_v3.x = this->sorted_publish_collection[it][0];
        foot_target_v3.y = this->sorted_publish_collection[it][1];
        foot_target_v3.z = this->sorted_publish_collection[it][2];

        publish_v3.push_back(foot_target_v3);
      }
    }
  }


  visualization_msgs::Marker marker2;
  marker2.header.frame_id = "/base_link";
  marker2.ns = "map_namespace";
  marker2.id = 2;
  marker2.type = visualization_msgs::Marker::CUBE_LIST;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.scale.x = 0.005;
  marker2.scale.y = 0.005;
  marker2.scale.z = 0.005;
  marker2.color.g = 0;
  marker2.color.r = 0;
  marker2.color.b = 1;
  marker2.color.a = 1.0;
  
  if (this->debug == true) std::cout << "Building Prototype Markers.." << std::endl;
  if (this->optimal_points.size() != 0) {
    for (int i = 0; this->optimal_points.size() > i; i++) {
        marker2.points.push_back(this->optimal_points[i]);
    }
  }

  //Publish the Marker.

  //xyz_pub = perception_publisher.advertise< geometry_msgs::Vector3 >( "/perception/scan_result", 1);
  //slam_pub = perception_publisher.advertise<visualization_msgs::Marker>( "/perception/slam_map_beta", 1 );
  //vis_pub = perception_publisher.advertise<visualization_msgs::Marker>( "/perception/visualization_marker", 1 );    


  slam_pub.publish ( marker2 );
  vis_pub.publish( marker );
  xyz_pub.publish( publish_v3[0] );

  std::cout << "Optimal Foot: " << this->standard_foot[0] << " " << this->standard_foot[1] << " " << this->standard_foot[2] << std::endl;
  if (publish_v3.size() != 0) std::cout << "Sent Target Tip Position" << publish_v3[0] << std::endl;

  //Empty the variables. 
  foot_target.clear();
  this->sorted_publish_collection.clear();
  publish_v3.clear();
  if (this->optimal_vector.size() != 0) this->optimal_vector.clear();
}


