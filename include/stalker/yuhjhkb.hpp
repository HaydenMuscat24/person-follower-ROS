/*
 * controlWrapper.hpp
 *
 *  Created on: 04/07/2016
 *      Author: rescue
 */

#ifndef STALKER_SIMPLE_FOLLOW_HPP_
#define STALKER_SIMPLE_FOLLOW_HPP_

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <tf/transform_listener.h>

#include <sstream>

namespace stalker {

class WallFollower {
private:
    bool paused, stopped;

    ros::Subscriber scanSub, commandSub;
    ros::Publisher twistPub;
    tf::TransformListener tfListener;

    enum Side { LEFT, RIGHT };
    Side side;

public:
    WallFollower();
    virtual ~WallFollower() {};

    void configure();
    void startup();
    void shutdown();

    void callbackScan(const sensor_msgs::LaserScanConstPtr& scan);
    void callbackControl(const std_msgs::StringConstPtr& command);
};

} // namespace comp3431

#endif /* COMP3431_STARTER_WALLFOLLOW_HPP_ */
