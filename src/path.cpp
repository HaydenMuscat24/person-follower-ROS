#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main( int argc, char** argv ) {

    // initial set up
    ros::init(argc, argv, "path");
    ros::NodeHandle n;
    ros::Rate r(1);

    // subscribers and publishers
    tf::TransformListener listener;
    ros::Publisher pathPub = n.advertise<nav_msgs::Path>("pathway", 1);

    // initialise the path
    int count = 0;
    nav_msgs::Path path;
    path.header.frame_id = "map";

    while (ros::ok()) {

      r.sleep();

        // update the path header
        path.header.seq = count;
        path.header.stamp = ros::Time(0);

        // add in a new pose at the end of the path
        geometry_msgs::PoseStamped newPose;
        newPose.header.seq = 0;
        newPose.header.stamp = ros::Time(0);
        newPose.header.frame_id = "/base_link";

        newPose.pose.position.x = 0;
        newPose.pose.position.y = 0;
        newPose.pose.position.z = 0;
        newPose.pose.orientation.x = 0.0;
        newPose.pose.orientation.y = 0.0;
        newPose.pose.orientation.z = 0.0;
        newPose.pose.orientation.w = 1.0;

        try {
          // add the new point to the path and publish the new path
          // if everything works
          listener.transformPose("/map", newPose, newPose);
          path.poses.push_back(newPose);
          pathPub.publish(path);
          count++;

        } catch (...) {
          // maybe slam or robot bringup isn't up yet.
          ROS_INFO("couldn't find either the map or base_link framce");
        }
    }
}
