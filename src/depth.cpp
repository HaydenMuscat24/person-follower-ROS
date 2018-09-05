/*
 * comp3431.cpp
 *
 *  Created on: 13/07/2016
 *      Author: Timothy Wiley
 */


#include <ros/ros.h>

#include <comp3431_starter/depth.hpp>

#include <sensor_msgs/image_encodings.h>

#include <algorithm>

#define LOG_START       "COMP3431Depth ::"
#define MAX_DEPTH       5.0
#define MIN_DEPTH       0.4

namespace comp3431 {

DepthProcessing::DepthProcessing() {
}

DepthProcessing::~DepthProcessing() {
}

void DepthProcessing::configure() {
    ros::NodeHandle paramNh("~");
    depth_topic = paramNh.param<std::string>("depthTopic", "depth");
    ROS_INFO("%s Using depth topic: %s", LOG_START, depth_topic.c_str());
}

void DepthProcessing::startup() {
    ros::NodeHandle paramNh("~");
    depthSub = paramNh.subscribe<sensor_msgs::Image>(depth_topic, 1, &DepthProcessing::callback_depth, this);
}

void DepthProcessing::shutdown() {
    depthSub.shutdown();
}

void DepthProcessing::callback_depth(const sensor_msgs::ImageConstPtr& depthMsg) {
    if (depthMsg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        float minDepth = MAX_DEPTH;
        float maxDepth = MIN_DEPTH;

        for(int i = 0; i < depthMsg->data.size(); i += depthMsg->step) {
            // Image data is array of unsigned char.
            // Therefore cast chars to actual float value.
            float floatVal = *reinterpret_cast<const float*>(&depthMsg->data[i]);

            // Depth image pixels may read zero or NAN, so filter
            if (floatVal >= MIN_DEPTH && floatVal <= MAX_DEPTH) {
                minDepth = std::min(minDepth, floatVal);
                maxDepth = std::max(maxDepth, floatVal);
            }
        }

        ROS_INFO("%s Depths min: %.2lf, max: %.2lf", LOG_START, minDepth, maxDepth);
    } else {
        ROS_WARN("%s Invalid image encoding: %s", LOG_START, depthMsg->encoding.c_str());
    }
}

} // namespace comp3431

