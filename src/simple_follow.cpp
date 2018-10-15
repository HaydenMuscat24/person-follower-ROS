#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>

void cb_scan(const sensor_msgs::LaserScanConstPtr& scan);
void cb_command(const std_msgs::StringConstPtr& command);
void cb_angle(const std_msgs::Float32ConstPtr& angle_msg);
void stop_robot();

#define BASE_FRAME  "base_link"
#define ROBOT_RADIUS      0.20
#define MAX_SPEED         0.5
#define MAX_TURN          0.8
#define OPTIMUM_DIST      3.0
// #define SLOWDOWN_DIST     1.0

// globals
double last_angle_time;
bool paused;
float angle;

ros::Subscriber scanSub ;
ros::Subscriber commandSub;
ros::Subscriber angleSub;
ros::Publisher twistPub;

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
    angleSub   = nh.subscribe<std_msgs::Float32>("/angle", 5, &cb_angle);
    twistPub   = nh.advertise<geometry_msgs::Twist >("cmd_vel", 1, false);

    // pretty much done
    while (ros::ok()) {
        ROS_INFO("Spinning");
        ros::spin();
    }

    stop_robot();
}

void stop_robot() {
    geometry_msgs::Twist t;
    t.linear.x = t.linear.y = t.linear.z = 0;
    t.angular.x = t.angular.y = t.angular.z = 0;
    twistPub.publish(t);
}

void spin_robot() {
    geometry_msgs::Twist t;
    t.linear.x = t.linear.y = t.linear.z = 0;
    t.angular.x = t.angular.y = 0;
    t.angular.z = 0.2;
    twistPub.publish(t);
}

void cb_scan(const sensor_msgs::LaserScanConstPtr& scan) {

    if (paused) {
        ROS_INFO("paused");
        stop_robot();
        return;
    }


    // Really Lost?
    if (ros::Time::now().toSec() - last_angle_time > 4) {
        ROS_INFO("Really lost");
        spin_robot();
        return;
    }

    // Lost?
    if (ros::Time::now().toSec() - last_angle_time > 1) {
        angle = 0;
    }

    // Turn laser scan into a point cloud
    // I HAVE NO IDEA WHAT HE FUCK IS HAPPENING HERE
    std::vector< tf::Vector3 > points;
    points.resize(scan->ranges.size());

    float XMinFront = INFINITY, laser_angle = scan->angle_min;
    for (int n = 0; n < points.size(); ++n, laser_angle += scan->angle_increment) {
        tf::Vector3 point(cos(laser_angle) * scan->ranges[n], sin(laser_angle) * scan->ranges[n], 0);

        // Find min XF of a hit in front of robot (X > 0, abs(Y) <= robot radius, X <= limit)
        if (point.x() > 0 && fabs(point.y()) <= ROBOT_RADIUS) {
            // Point is in front of the robot
            if (point.x() < XMinFront) XMinFront = point.x();
        }
    }

    // if we found a minimum laser in front, calibrate speed toward an optimum distance
    float drive = (XMinFront == INFINITY) ? 0 : (XMinFront - OPTIMUM_DIST) / 10.0;

    // cut excess speed off
    if (fabs(drive) > 1) drive = drive / fabs(drive);

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

void cb_angle(const std_msgs::Float32ConstPtr& angle_msg) {
    last_angle_time = ros::Time::now().toSec();
    angle = angle_msg->data / 20.0 * -1.0;
    if (abs(angle) > 1) angle = angle / abs(angle);
}

void cb_command(const std_msgs::StringConstPtr& command) {
    ROS_INFO("Recieved %s message.\n", command->data.c_str());
    if (strcasecmp(command->data.c_str(), "start") == 0 )  {
        paused = false;
    } else if (strcasecmp(command->data.c_str(), "stop") == 0) {
        paused = true;
        stop_robot();
    }
}
