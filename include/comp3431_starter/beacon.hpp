/*
 * beacon.hpp
 *
 *  Created on: 13/07/2016
 *      Author: Timothy Wiley
 */

#ifndef COMP3431_STARTER_BEACON_HPP_
#define COMP3431_STARTER_BEACON_HPP_

#include <ros/node_handle.h>
#include <string>
#include <vector>
#include <visualization_msgs/Marker.h>

#define BEACON_BLUE         "blue"
#define BEACON_GREEN        "green"
#define BEACON_PINK         "pink"
#define BEACON_YELLOW       "yellow"

namespace comp3431 {

class Beacon {
public:
    int id;
    std::string top;
    std::string bottom;
    visualization_msgs::Marker marker;
    Beacon(int id = -1, std::string top = "", std::string bottom = "") :
        id(id), top(top), bottom(bottom)
    {
      marker.id = id;
      marker.ns = "beacon";
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.header.frame_id = "map";
      marker.action = 0;

      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0; 
      marker.pose.orientation.w = 1.0;

      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.2;
   
      marker.lifetime = ros::Duration();
      marker.frame_locked = true;

    };
    Beacon(const Beacon& other) :
        id(other.id), top(other.top), bottom(other.bottom)
    {
      marker.id = id;
      marker.ns = "beacon";
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.header.frame_id = "map";
      marker.action = 0;

      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0; 
      marker.pose.orientation.w = 1.0;

      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 0.0;

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.2;
   
      marker.lifetime = ros::Duration();
      marker.frame_locked = true;

    };
    ~Beacon() {};


    static std::vector<Beacon> parseBeacons(ros::NodeHandle& paramNh);
};

}


#endif /* COMP3431_STARTER_BEACON_HPP_ */
