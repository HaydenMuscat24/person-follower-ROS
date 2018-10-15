#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

void cb_command(const std_msgs::StringConstPtr& command);
// void cb_angle(const std_msgs::Float64ConstPtr& angle_msg);
void stop_robot();
void jitter(bool start);

// globals
float angle;
enum Action {startScan, endScan, Turn, nothing};
Action action = nothing;

// ros::Subscriber scanSub ;
ros::Subscriber commandSub;
// ros::Subscriber angleSub;
ros::Publisher twistPub;

int main( int argc, char** argv ) {

    // initial set up
    ros::init(argc, argv, "initialiser");
    ros::NodeHandle nh;
    ros::Rate r(1);

    // subscribers and publishers
    commandSub = nh.subscribe<std_msgs::String>("cmd", 1, &cb_command);
    twistPub   = nh.advertise<geometry_msgs::Twist >("cmd_vel", 5, false);

    // pretty much done
    while (ros::ok()) {
        ros::spinOnce();
        switch (action) {
            case startScan:  jitter(true);  break;
            case endScan:    jitter(false); break;
            // case Turn:    turn();    break;
        }
    }
}

void stop_robot() {
    geometry_msgs::Twist t;
    t.linear.x = t.linear.y = t.linear.z = 0;
    t.angular.x = t.angular.y = t.angular.z = 0;
    twistPub.publish(t);
}

// if starting the scan, forward and back.
// if ending the scan, back and forward
void jitter(bool start) {

    float SPEED = 1;
    float DUR = 0.3;

    if (start) SPEED *= -1;

    geometry_msgs::Twist t;
    t.linear.y = t.linear.z = 0;
    t.angular.z = t.angular.x = t.angular.y = 0;

    // part 1
    t.linear.x = SPEED;
    twistPub.publish(t);
    ros::Duration(DUR).sleep();

    // part 2
    t.linear.x = -SPEED;
    twistPub.publish(t);
    ros::Duration(DUR).sleep();

    t.linear.x = 0;
    twistPub.publish(t);

    action = nothing;
}


void cb_command(const std_msgs::StringConstPtr& command) {
    ROS_INFO("Recieved %s message.\n", command->data.c_str());

    const auto string = command->data.c_str();

         if (strcasecmp(string, "startScan")  == 0)  action = startScan;
    else if (strcasecmp(string, "endScan") == 0)  action = endScan;
    else if (strcasecmp(string, "turn")    == 0)  action = Turn;
    else if (strcasecmp(string, "stop")    == 0) {
        action = nothing;
        stop_robot();
    }
}
