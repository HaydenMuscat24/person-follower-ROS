/*
 * beacon_node.cpp
 *
 *  Created on: 13/07/2016
 *      Author: Timothy Wiley
 */

#include <ros/ros.h>

#include <comp3431_starter/beacon.hpp>
#include <comp3431_starter/image_recorder.hpp>
#include <cstdio>
#include <cstdlib>
#include <vector>

#define LOG_START    "COMP3431BeaconsNode ::"

namespace comp3431 {

int main_comp3431beacons(int argc, char** argv) {
    ros::init(argc, argv, "comp3431_starter_beacons");

    // Node Handle - Use '~' when loading config parameters
    // Use without '~' when publish/subscribe/service
    ros::NodeHandle paramNh("~");
   
    // Load beacons
    std::vector<Beacon> beacons;
    beacons = Beacon::parseBeacons(paramNh);
 //   ImageHandler ih;
    while (ros::ok()) {
    //    ROS_INFO("%s Spinning", LOG_START);

        ros::spin();
    }

    return 0;
}

} // namespace timothyw_turtlebot

// Actual main method outside of namespace
int main(int argc, char** argv) {
    comp3431::main_comp3431beacons(argc, argv);
}
