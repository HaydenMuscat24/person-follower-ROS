/*
 *
 *  Created on: 05/04/2014
 *      Author: Timothy Wiley
 */

#include <ros/ros.h>

#include <comp3431_starter/wallFollow.hpp>

#include <cstdio>
#include <cstdlib>

#define LOG_START    "COMP3431WallFollowNode ::"

namespace comp3431 {

int main_comp3431WallFollow(int argc, char** argv) {
    ros::init(argc, argv, "comp3431_starter_wallfollow");

    // Node Handle - Use '~' when loading config parameters
    // Use without '~' when publish/subscribe/service
    ros::NodeHandle nh("~");

    WallFollower follow;
    follow.configure();
    follow.startup();

    while (ros::ok()) {
        ROS_INFO("%s Spinning", LOG_START);

        ros::spin();
    }

    // Disconnect
    follow.shutdown();

    return 0;
}

} // namespace comp3431

// Actual main method outside of namespace
int main(int argc, char** argv) {
    comp3431::main_comp3431WallFollow(argc, argv);
}
