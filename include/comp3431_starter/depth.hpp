/*
 * controlWrapper.hpp
 *
 *  Created on: 04/07/2016
 *      Author: rescue
 */

#ifndef COMP3431_STARTER_DEPTH_HPP_
#define COMP3431_STARTER_DEPTH_HPP_

#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>

namespace comp3431 {

class DepthProcessing {
public:
    DepthProcessing();
    virtual ~DepthProcessing();

    void configure();
    void startup();
    void shutdown();

    // Command callbacks
    void callback_depth(const sensor_msgs::ImageConstPtr& depthMsg);

private:
    // Configuration
    std::string depth_topic;

    // ROS publishers/subscribers
    ros::Subscriber depthSub;
};

} // namespace comp3431

#endif /* COMP3431_STARTER_DEPTH_HPP_ */
