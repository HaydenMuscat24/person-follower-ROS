#include <ros/ros.h>
#include <ros/node_handle.h>

#include <string>
#include <vector>

#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#define PINK_LOW cv::Scalar(145, 95, 150)
#define PINK_HIGH cv::Scalar(170, 255, 255)
#define BLUE_LOW cv::Scalar(95, 50, 100)
#define BLUE_HIGH cv::Scalar(110, 255, 255)
#define YELLOW_LOW cv::Scalar(15, 220, 120)
#define YELLOW_HIgh cv::Scalar(35, 255, 200)
#define GREEN_LOW cv::Scalar(65, 100, 50)
#define GREEN_HIGH cv::Scalar(95, 255, 190)

#define IMAGE_WIDTH 640 
#define IMAGE_HEIGHT 480
#define FOV_WIDTH 62.0 
#define FOV_HEIGHT 48.5


class ImageHandler {


public:
 //  ros::NodeHandle nh;
 //  image_transport::ImageTransport it;
 //  image_transport::Subscriber im_subscriber;
 //  image_transport::Subscriber dp_subscriber;
 //  ros::Publisher beacon_publisher;
   cv::Mat dep_im;

   ImageHandler();
   ~ImageHandler();
   void im_callback(const sensor_msgs::ImageConstPtr& msg);
   void dp_callback(const sensor_msgs::ImageConstPtr& msg);

private:
   void get_beacons(cv::Mat& im);
   void pub_beacons(std::string top, std::string bottom, int xpos, int ypos);
};
