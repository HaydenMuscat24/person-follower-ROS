#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <comp3431_starter/BeaconInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <comp3431_starter/beacon.hpp>
#include "std_msgs/Int32.h"
using namespace std;
using namespace comp3431;

class BeaconMarker {
public:
  tf::TransformListener listener;
  visualization_msgs::MarkerArray markerArray;
  ros::Publisher marker_pub;
  ros::Publisher beacon_pub;
  ros::Subscriber beacon_sub;
  vector<Beacon> beacons;
  std_msgs::Int32 beacon_count;
  BeaconMarker() {
    beacon_count.data = 0;
    ros::NodeHandle paramNh("~");
    beacons = Beacon::parseBeacons(paramNh);
    for (Beacon& b : beacons) {      
      markerArray.markers.push_back(b.marker);
    }
    ros::NodeHandle n_handle("~");
    marker_pub = n_handle.advertise<visualization_msgs::Marker>("/comp3431/beacons",10,false);
    beacon_pub = n_handle.advertise<std_msgs::Int32>("/beaconCount",1,false);
    beacon_sub = n_handle.subscribe("/beacon", 1, &BeaconMarker::beaconCB, this);
  }
  
  ~BeaconMarker() {
  }

  void beaconCB(const comp3431_starter::BeaconInfo::ConstPtr& msg) {
    for (Beacon& b : beacons) {
      
      string bt = (b.top).c_str();
      string bb = (b.bottom).c_str();
      string mt = (msg->c1).c_str();
      string mb = (msg->c2).c_str();
      if (bt == mt && bb == mb) {
        ROS_INFO("Beacon: Top %s Bottom %s; Message: Top %s Bottom %s\n", (b.top).c_str(), (b.bottom).c_str(), (msg->c1).c_str(), (msg->c2).c_str());
        geometry_msgs::PoseStamped p;
        p.header.frame_id = "base_link";
        p.header.stamp = ros::Time(0);
        p.header.seq = b.id;
        p.pose.position.x = msg->x-0.1;
        p.pose.position.y = msg->y;
        p.pose.position.z = -0.2;
        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;
        p.pose.orientation.z = 0.0;
        p.pose.orientation.w = 1.0;
        
  //      float curX = b.marker.pose.position.x;
  //      float curY = b.marker.pose.position.y;

        geometry_msgs::PoseStamped ret;
        listener.transformPose("map", p, ret);     
        b.marker.pose = ret.pose;
        if (b.marker.color.a == 0.0) {
          
          beacon_count.data += 1;
          beacon_pub.publish(beacon_count);
        }
        b.marker.color.a = 1.0;
        marker_pub.publish(b.marker);
        ROS_INFO("Beacon found!\n");
        break;
      } 
      //ROS_INFO("Beacon found!\n");
    }
  }

  
  

};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "beacon_marker");
  BeaconMarker bm;
  while (ros::ok()) {
       // ROS_INFO("%s Spinning", LOG_START);

        ros::spin();
  }
  return 0;
}
