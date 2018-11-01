#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void cb_twist(const geometry_msgs::Twist vel);
void stop_robot();

// globals
double lastTwistTime = 0;

ros::Publisher  twistPub;
ros::Subscriber twistSub;

int main( int argc, char** argv ) {

    // initial set up
    ros::init(argc, argv, "killSwitch");
    ros::NodeHandle nh;
    ros::Rate r(1);

    twistSub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &cb_twist);
    twistPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    while (ros::ok()) {
        ros::Duration(0.1).sleep();

        // If more than a second has passed, send a stop command
        if (ros::Time::now().toSec() - lastTwistTime > 2) stop_robot();
    }
}

void stop_robot() {
    geometry_msgs::Twist t;
    t.linear.x  = t.linear.y  = t.linear.z  = 0;
    t.angular.x = t.angular.y = t.angular.z = 0;
    twistPub.publish(t);
}

void cb_twist(const geometry_msgs::Twist vel) {
    lastTwistTime = ros::Time::now().toSec();
}
