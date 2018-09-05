/*
 *
 *  Created on: 05/04/2014
 *      Author: Timothy Wiley
 */

#include <ros/ros.h>

#include <comp3431_starter/depth.hpp>

#include <cstdio>
#include <cstdlib>

#define LOG_START    "COMP3431DepthNode ::"

namespace comp3431 {

int main_comp3431Depth(int argc, char** argv) {
    ros::init(argc, argv, "comp3431_starter_depth");

    // Node Handle - Use '~' when loading config parameters
    // Use without '~' when publish/subscribe/service
    ros::NodeHandle nh("~");

    DepthProcessing depth;
    depth.configure();
    depth.startup();

    while (ros::ok()) {
        ROS_INFO("%s Spinning", LOG_START);

        ros::spin();
    }

    // Disconnect
    depth.shutdown();

    return 0;
}

} // namespace comp3431

// Actual main method outside of namespace
int main(int argc, char** argv) {
    comp3431::main_comp3431Depth(argc, argv);
}
