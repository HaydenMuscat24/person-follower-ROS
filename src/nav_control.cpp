#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sstream>

class NavControl {
public:
  double startX;
  double startY;
  int beaconCount = 0;
  bool isInit = false;
  ros::Publisher cmd_pub;
  ros::Subscriber path_sub;
  ros::Subscriber beacon_sub;

  NavControl() {
    ros::NodeHandle n_handle("~");
    ros::NodeHandle n_handle2("~");
    cmd_pub = n_handle.advertise<std_msgs::String>("/cmd",1,false);
    path_sub = n_handle.subscribe("/pathway", 1, &NavControl::pathCB, this);
    beacon_sub = n_handle2.subscribe("/beaconCount", 1, &NavControl::countCB, this);
  }
  
  ~NavControl() {
  }
  
  void pathCB(const nav_msgs::PathConstPtr& msg) {
    int size = (msg->poses).size()-1;
    geometry_msgs::Point pt = msg->poses[size].pose.position;
    ROS_INFO("path received\n");
    if (!isInit) {
      
      isInit = true;
      startX = msg->poses[0].pose.position.x;
      startY = msg->poses[0].pose.position.y;
    }
    
    if ((pt.x < (startX+0.5) && pt.x > (startX-0.5)) && (pt.y < (startY+0.5) && pt.y > (startY-0.5))) {
      ROS_INFO("In start region\n");
      if (beaconCount >= 4) {
        std_msgs::String cmd;
        std::stringstream ss;
        ss << "stop";
        cmd.data = ss.str();
        cmd_pub.publish(cmd);  
      }    
    }
  }

  void countCB(const std_msgs::Int32::ConstPtr& msg) {
    beaconCount = msg->data;
    ROS_INFO("BeaconCount: %i\n", beaconCount);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "navcontrol");
  NavControl nc;
  while (ros::ok()) {
       // ROS_INFO("%s Spinning", LOG_START);

    ros::spin();
  }
}
