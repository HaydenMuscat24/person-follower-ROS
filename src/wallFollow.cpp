/*
 * wall_follow.cpp
 *
 *  Created on: 15/07/2016
 *      Author: mmcgill
 */

#include <ros/ros.h>

#include <comp3431_starter/wallFollow.hpp>

#define BASE_FRAME  "base_link"
#define MAX_SIDE_LIMIT      0.50
#define MIN_APPROACH_DIST   0.30
#define MAX_APPROACH_DIST   0.50

#define ROBOT_RADIUS        0.20

#define MAX_SPEED       0.25
#define MAX_TURN        1.0

#define CLIP_0_1(X)         ((X) < 0?0:(X)>1?1:(X))

namespace comp3431 {

WallFollower::WallFollower() :
    paused(true), stopped(false), side(LEFT) {
}

void WallFollower::configure() {
}

void WallFollower::startup() {
    ros::NodeHandle nh;
    scanSub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, &WallFollower::callbackScan, this);
    commandSub = nh.subscribe<std_msgs::String>("cmd", 1, &WallFollower::callbackControl, this);
    twistPub = nh.advertise< geometry_msgs::Twist >("cmd_vel", 1, false);
}

void WallFollower::shutdown() {
    if (!paused) {
        paused = true;
        geometry_msgs::Twist t;
        t.linear.x = t.linear.y = t.linear.z = 0;
        t.angular.x = t.angular.y = t.angular.z = 0;

        twistPub.publish(t);
    }
}

void WallFollower::callbackScan(const sensor_msgs::LaserScanConstPtr& scan) {
    if (paused) {
        if (!stopped) {
            geometry_msgs::Twist t;
            t.linear.x = t.linear.y = t.linear.z = 0;
            t.angular.x = t.angular.y = t.angular.z = 0;

            twistPub.publish(t);
            stopped = true;
        }
        return;
    }

    tf::StampedTransform transform;
    // get laser to base_link transform
    try {
        tfListener.waitForTransform(BASE_FRAME, scan->header.frame_id, scan->header.stamp, ros::Duration(2.0));
        tfListener.lookupTransform(BASE_FRAME, scan->header.frame_id, scan->header.stamp, transform);
    } catch (tf::TransformException& tfe) {
        ROS_ERROR("Unable to get transformation.");
        return;
    }


    // Turn laser scan into a point cloud
    std::vector< tf::Vector3 > points;
    points.resize(scan->ranges.size());

    float XMaxSide = -INFINITY, XMinFront = INFINITY, angle = scan->angle_min;
    for (int n = 0; n < points.size(); ++n, angle += scan->angle_increment) {
        tf::Vector3 point(cos(angle) * scan->ranges[n], sin(angle) * scan->ranges[n], 0);

        // transfer point to base_link frame
        points[n] = point = transform * point;

        // Find max XS of a hit to the side of robot (abs(X) <= robot radius, Y > 0 left, Y < 0 right, abs(Y) <= limit)
        if (fabs(point.x()) <= ROBOT_RADIUS && fabs(point.y()) <= MAX_SIDE_LIMIT) {
            if ((side == LEFT && point.y() > 0) || (side == RIGHT && point.y() < 0)) {
                // Point is beside the robot
                if (point.x() > XMaxSide)
                    XMaxSide = point.x();
            }
        }


        // Find min XF of a hit in front of robot (X > 0, abs(Y) <= robot radius, X <= limit)
        if (point.x() > 0 && point.x() <= MAX_APPROACH_DIST && fabs(point.y()) <= ROBOT_RADIUS) {
            // Point is in front of the robot
            if (point.x() < XMinFront)
                XMinFront = point.x();
        }
    }


    ROS_DEBUG("Detected walls %.2f left, %.2f front\n", XMaxSide, XMinFront);
    float turn, drive;

    if (XMaxSide == -INFINITY) {
        // No hits beside robot, so turn that direction
        turn = 1;
        drive = 0;
    } else if (XMinFront <= MIN_APPROACH_DIST) {
        // Blocked side and front, so turn other direction
        turn = -1;
        drive = 0;
    } else {
        // turn1 = (radius - XS) / 2*radius  // Clipped to range (0..1)
        float turn1 = (ROBOT_RADIUS - XMaxSide) / (2 * ROBOT_RADIUS);
        turn1 = CLIP_0_1(turn1);

        // drive1 = (radius + XS) / 2*radius // Clipped to range (0..1)
        float drive1 = (ROBOT_RADIUS + XMaxSide) / (2 * ROBOT_RADIUS);
        drive1 = CLIP_0_1(drive1);

        // drive2 = (XF - min) / (limit - min)  // Clipped to range (0..1)
        float drive2 = (XMinFront - MIN_APPROACH_DIST) / (MAX_APPROACH_DIST - MIN_APPROACH_DIST);
        drive2 = CLIP_0_1(drive2);

        // turn2 = (limit - XF) / (limit - min) // Clipped to range (0..1)
        float turn2 = (MAX_APPROACH_DIST - XMinFront) / (MAX_APPROACH_DIST - MIN_APPROACH_DIST);
        turn2 = CLIP_0_1(turn2);

        // turn = turn1 - turn2
        turn = turn1 - turn2;

        // drive = drive1 * drive2
        drive = drive1 * drive2;
    }


    //if (side == RIGHT) {
    //    turn *= -1;
    //}

    // publish twist
    geometry_msgs::Twist t;
    t.linear.y = t.linear.z = 0;
    t.linear.x = drive * MAX_SPEED;
    t.angular.x = t.angular.y = 0;
    t.angular.z = turn * MAX_TURN;

    ROS_DEBUG("Publishing velocities %.2f m/s, %.2f r/s\n", t.linear.x, t.angular.z);
    twistPub.publish(t);
    stopped = false;
}

void WallFollower::callbackControl(const std_msgs::StringConstPtr& command) {
    ROS_INFO("Recieved %s message.\n", command->data.c_str());
    if (strcasecmp(command->data.c_str(), "start") == 0 ||
            strcasecmp(command->data.c_str(), "go") == 0 ||
            strcasecmp(command->data.c_str(), "begin") == 0 ||
            strcasecmp(command->data.c_str(), "resume") == 0) {
        paused = false;
    } else if (strcasecmp(command->data.c_str(), "stop") == 0 ||
            strcasecmp(command->data.c_str(), "pause") == 0 ||
            strcasecmp(command->data.c_str(), "halt") == 0) {
        paused = true;
    } else if (strcasecmp(command->data.c_str(), "toggle") == 0) {
        paused = !paused;
    }
}

} // namespace comp3431

