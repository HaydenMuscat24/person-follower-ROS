#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

class PersonMarker {
	ros::Subscriber scanSub ;
	ros::Subscriber angleSub;
	ros::Publisher markerPub;
	tf::TransformListener listener;
	float detected_angle;

public:
	PersonMarker() {
		detected_angle = 0.0;
		ros::NodeHandle nh("~");
		ros::NodeHandle nh2("~");
		scanSub    = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &PersonMarker::cb_scan, this);
		angleSub   = nh2.subscribe<std_msgs::Float32>("/angle", 5, &PersonMarker::cb_angle, this);
		markerPub  = nh.advertise<visualization_msgs::Marker>("/marker", 10, false);
		ROS_INFO("Initialised\n");
	}

	~PersonMarker() {		
	}

	void cb_scan(const sensor_msgs::LaserScanConstPtr& scan) {
		// get distance at detected_angle
		float laser_angle = scan->angle_min;
		float my_x = 0;
		float my_y = 0;

		for (int n = 0; n < scan->ranges.size(); ++n, laser_angle += scan->angle_increment) {
			if (detected_angle - 2 < laser_angle && laser_angle < detected_angle + 2) {
				my_x = cos(laser_angle) * scan->ranges[n];
				my_y = sin(laser_angle) * scan->ranges[n];
				break;
			}
		}

		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.ns = "stalker";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;

		geometry_msgs::PoseStamped p;
		p.header.frame_id = "base_link";
		p.header.stamp = ros::Time(0);
		p.header.seq = 0;
		p.pose.position.x = my_x;
		p.pose.position.y = my_y;
		p.pose.position.z = 0;
		p.pose.orientation.x = 0.0;
		p.pose.orientation.y = 0.0;
		p.pose.orientation.z = 0.0;
		p.pose.orientation.w = 1.0;

		
		geometry_msgs::PoseStamped ret;        
		
		listener.transformPose("/map", p, ret);     	
		
		marker.pose = ret.pose;

		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		

		markerPub.publish(marker);
	}

	void cb_angle(const std_msgs::Float32ConstPtr& angle_msg) {
	    detected_angle = angle_msg->data / 10.0 * -1.0;
	}
};


int main( int argc, char** argv ) {
	ros::init(argc, argv, "person_marker");
	PersonMarker pm;	

	while (ros::ok()) {
		ROS_INFO("Spinning");
		ros::spin();
	}
}


