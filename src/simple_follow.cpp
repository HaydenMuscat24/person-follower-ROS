#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>

void cb_scan(const sensor_msgs::LaserScanConstPtr& scan);
void cb_command(const std_msgs::StringConstPtr& command);
void cb_angle(const std_msgs::Float64ConstPtr& angle_msg);
void stop_robot();

#define BASE_FRAME  "base_link"
#define ROBOT_RADIUS      0.20
#define SLOWDOWN_DIST     2.0
#define MAX_SPEED         0.5
#define MAX_TURN          0.5
#define MIN_APPROACH_DIST 0.5

// globals
double last_angle_time;
bool paused;
float angle;

ros::Subscriber scanSub ;
ros::Subscriber commandSub;
ros::Subscriber angleSub;
ros::Publisher twistPub;
//tf::TransformListener tfListener;

int main( int argc, char** argv ) {

    // initial set up
    ros::init(argc, argv, "simple_follow");
    ros::NodeHandle nh;
    ros::Rate r(1);

    last_angle_time = -5;
    paused = true;
    angle = 0;

    // subscribers and publishers
    scanSub    = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, &cb_scan);
    commandSub = nh.subscribe<std_msgs::String>("cmd", 1, &cb_command);
    angleSub   = nh.subscribe<std_msgs::Float64>("/angle", 5, &cb_angle);
    twistPub   = nh.advertise<geometry_msgs::Twist >("cmd_vel", 1, false);

    // pretty much done
    while (ros::ok()) {
        ROS_INFO("Spinning");
        ros::spin();
    }

    stop_robot();
    // try {
    //   // add the new point to the path and publish the new path
    //   // if everything works
    //   listener.transformPose("/map", newPose, newPose);
    //   path.poses.push_back(newPose);
    //   pathPub.publish(path);
    //   count++;
    //
    //   if (last_message != 1){
    //     ROS_INFO("Working Fine");
    //     last_message = 1;
    //   }
    //
    // } catch (...) {
    //   // maybe slam or robot bringup isn't up yet.
    //   if (last_message != 2){
    //     ROS_INFO("couldn't find either the map or base_link frame");
    //     last_message = 2;
    //   }
    // }
}

void stop_robot() {
    geometry_msgs::Twist t;
    t.linear.x = t.linear.y = t.linear.z = 0;
    t.angular.x = t.angular.y = t.angular.z = 0;
    twistPub.publish(t);
}

void cb_scan(const sensor_msgs::LaserScanConstPtr& scan) {

    bool lost_person = false;
    if (ros::Time::now().toSec() - last_angle_time > 1) lost_person = true;
    // if we're paused, don't do anything
    if (paused || lost_person) {
        if (paused) ROS_INFO("paused");
        if (lost_person) ROS_INFO("lost");
        stop_robot();
        return;
    }

    // tf::StampedTransform transform;
    // // get laser to base_link transform
    // try {
    //     tfListener.waitForTransform(BASE_FRAME, scan->header.frame_id, scan->header.stamp, ros::Duration(2.0));
    //     tfListener.lookupTransform(BASE_FRAME, scan->header.frame_id, scan->header.stamp, transform);
    // } catch (tf::TransformException& tfe) {
    //     ROS_ERROR("Unable to get transformation.");
    //     return;
    // }


    // Turn laser scan into a point cloud
    // I HAVE NO IDEA WHAT HE FUCK IS HAPPENING HERE
    std::vector< tf::Vector3 > points;
    points.resize(scan->ranges.size());

    float XMinFront = INFINITY, laser_angle = scan->angle_min;
    for (int n = 0; n < points.size(); ++n, laser_angle += scan->angle_increment) {
        tf::Vector3 point(cos(laser_angle) * scan->ranges[n], sin(laser_angle) * scan->ranges[n], 0);

        // transfer point to base_link frame
        points[n] = point = point; //transform * point;

        // Find min XF of a hit in front of robot (X > 0, abs(Y) <= robot radius, X <= limit)
        if (point.x() > 0 && fabs(point.y()) <= ROBOT_RADIUS) {
            // Point is in front of the robot
            if (point.x() < XMinFront)
                XMinFront = point.x();
        }
    }

    float drive = 1;
    if (XMinFront < SLOWDOWN_DIST){
        drive = (XMinFront - MIN_APPROACH_DIST) / (SLOWDOWN_DIST - MIN_APPROACH_DIST);
        if (drive < - 0.1) drive = -0.1;
    }

    // if the robot is getting close to something, slow it down
    ROS_INFO("Drive: %.2f, angle: %.2f", drive, angle * MAX_TURN);


    geometry_msgs::Twist t;
    t.linear.y = t.linear.z = 0;
    t.linear.x = drive * MAX_SPEED;
    t.angular.x = t.angular.y = 0;
    t.angular.z = angle * MAX_TURN;

    twistPub.publish(t);
    angle -= 0.7 * angle * MAX_TURN;
}

void cb_angle(const std_msgs::Float64ConstPtr& angle_msg) {
    last_angle_time = ros::Time::now().toSec();
    angle = angle_msg->data / 10.0 * -1.0;
    if (abs(angle) > 1) angle = angle / abs(angle);
}

void cb_command(const std_msgs::StringConstPtr& command) {
    ROS_INFO("Recieved %s message.\n", command->data.c_str());
    if (strcasecmp(command->data.c_str(), "start") == 0 )  {
        paused = false;
    }
    else if (strcasecmp(command->data.c_str(), "stop") == 0) {
        paused = true;
        stop_robot();
    }
}
