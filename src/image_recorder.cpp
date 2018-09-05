#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <stdint.h>
#include <comp3431_starter/BeaconInfo.h>


#define PINK_LOW    cv::Scalar(150, 95, 100)
#define PINK_HIGH   cv::Scalar(190, 255, 255)
#define BLUE_LOW    cv::Scalar(90, 100, 100)
#define BLUE_HIGH   cv::Scalar(100, 255, 250)
#define YELLOW_LOW  cv::Scalar(20, 100, 100)
#define YELLOW_HIGH cv::Scalar(30, 255, 255)
#define GREEN_LOW   cv::Scalar(65, 100, 50)
#define GREEN_HIGH  cv::Scalar(95, 255, 190)

#define IMAGE_WIDTH 640  
#define IMAGE_HEIGHT 480 
#define FOV_WIDTH 70.0   
#define FOV_HEIGHT 43.0  

static const std::string OPENCV_WINDOW = "Image";
using namespace std;


class ImageConverter
{
  ros::NodeHandle nh;
  ros::Publisher beacon_pub;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  image_transport::Subscriber dp_sub;
  cv::Mat dep_im;
public:
  ImageConverter()
    : it(nh)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub = it.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::im_callback, this);
    dp_sub = it.subscribe("/camera/depth/image_raw", 1, &ImageConverter::dp_callback, this);

    ros::NodeHandle n_handle("~");
    beacon_pub = n_handle.advertise<comp3431_starter::BeaconInfo>("/beacon", 1, false);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void im_callback(const sensor_msgs::ImageConstPtr& msg)
  {
	
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    ImageConverter::targetBeacons(cv_ptr->image);
    
    // Update GUI Window
    

  }

  void dp_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try 
    {
      cv_ptr = cv_bridge::toCvCopy(msg, "32FC1");
    }
    catch (cv_bridge::Exception& exp) {
      ROS_ERROR("cv_bridge exception: %s", exp.what());
      return; 
    }
    dep_im = cv_ptr->image;
  //  cv::imshow("Depth", cv_ptr->image);
  //   
    if (dep_im.rows > 0) {
      cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
      cv::imshow("Depth", dep_im);
      cv::waitKey(3);
    }      
  }
  
  void targetBeacons(cv::Mat& im) {
    cv::Mat hsv;
    cv::Mat frame_mask, frame_threshold;

    cv::cvtColor(im, hsv, CV_BGR2HSV);

    cv::inRange(hsv, PINK_LOW, PINK_HIGH, frame_mask);   
    cv::medianBlur(frame_mask, frame_mask, 5); 
    vector<cv::Point2f> pink_beacons = getPoints(frame_mask, im);
    
    cv::inRange(hsv, GREEN_LOW, GREEN_HIGH, frame_mask);
    cv::medianBlur(frame_mask, frame_mask, 5);
    vector<cv::Point2f> green_beacons = getPoints(frame_mask, im);
    //cv::bitwise_and(im, im, frame_threshold,frame_mask);
    
    //cv::imshow(OPENCV_WINDOW, frame_mask);    
    cv::inRange(hsv, YELLOW_LOW, YELLOW_HIGH, frame_mask);
    cv::medianBlur(frame_mask, frame_mask, 5);
    vector<cv::Point2f> yellow_beacons = getPoints(frame_mask, im);
    
    cv::inRange(hsv, BLUE_LOW, BLUE_HIGH, frame_mask);
    cv::medianBlur(frame_mask, frame_mask, 5);
    vector<cv::Point2f> blue_beacons = getPoints(frame_mask, im); 
    
    
    for(vector<cv::Point2f>::iterator p = pink_beacons.begin(); p != pink_beacons.end(); p++) {
      for (vector<cv::Point2f>::iterator y = yellow_beacons.begin(); y != yellow_beacons.end(); y++) {
        if (abs(y->x - p->x) < 5) {
          int xpos = (int)(y->x + p->x)/2;
          int ypos = (int)(y->y + p->y)/2;
          if (y->y < p->y) {
            publish_info("yellow", "pink", xpos, ypos);
          } else {
            publish_info("pink", "yellow", xpos, ypos);
          }
        }
      }
      for (vector<cv::Point2f>::iterator g = green_beacons.begin(); g != green_beacons.end(); g++) {
        if (abs(g->x - p->x) < 5) {
          int xpos = (int)(g->x + p->x)/2;
          int ypos = (int)(g->y + p->y)/2;
          if (g->y < p-> y) {
            publish_info("green", "pink", xpos, ypos);
          } else {
            publish_info("pink", "green", xpos, ypos);
          }
        }
      }
      for (vector<cv::Point2f>::iterator b = blue_beacons.begin(); b != blue_beacons.end(); b++) {
        if (abs(b->x - p->x) < 5) {
          int xpos = (int)(b->x + p->x)/2;
          int ypos = (int)(b->y + p->y)/2;
          if (b->y < p-> y) {
            publish_info("blue", "pink", xpos, ypos);
          } else {
            publish_info("pink", "blue", xpos, ypos);
          }
        }
      }
    }
//    cv::imshow(OPENCV_WINDOW, im);
    cv::waitKey(3);
  }
  
  vector<cv::Point2f> getPoints(cv::Mat& mask, cv::Mat& image) {
    cv::Mat canny;
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;

    /// Detect edges using canny
    Canny(mask, canny, 100, 100*5, 3 );
    /// Find contours
    findContours( canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    /// Get the moments
    vector<cv::Moments> mu(contours.size());
    for(int i = 0; i < contours.size(); i++) { 
        mu[i] = moments(contours[i], false);
    }

    ///  Get the mass centers:
    vector<cv::Point2f> mc;
    for( int i = 0; i < contours.size(); i++ ) {  
        cv::Point2f p( mu[i].m10/(mu[i].m00 + 1e-5), mu[i].m01/(mu[i].m00 + 1e-5));
        if (arcLength(contours[i], true) < 200) continue;
        mc.push_back(p);
    }    
    return mc;
  }
  
  void publish_info(string c1, string c2, int xpos, int ypos) {
    int radius = 15;
    int iMin = xpos - radius; int jMin = ypos - radius;
    int iMax = xpos + radius; int jMax = ypos + radius;
    if (iMin < 0) iMin = 0; if (iMax > IMAGE_WIDTH) iMax = IMAGE_WIDTH;
    if (jMin < 0) jMin = 0; if (jMax > IMAGE_HEIGHT) jMax = IMAGE_HEIGHT;
    float total = 0;
    int count = 0;
    for (int i = iMin; i < iMax; i++) {
      for (int j = jMin; j < jMax; j++) {
        float temp = dep_im.at<float>(j, i);
        if (temp >= 500 && temp <= 1500) {
          total += temp;
          count++;
        }
      }
    }
    float depth = total/count;
    
    float dep = depth/1000;
    
    if (dep > 0.5 && dep < 1.5) {
      float angle = ((float)xpos/IMAGE_WIDTH -0.5) * FOV_WIDTH;
      float localX = dep*cos(-angle*M_PI/180);
      float localY = dep*sin(-angle*M_PI/180);
      comp3431_starter::BeaconInfo message;
      message.c1 = c1.c_str();
      message.c2 = c2.c_str();
      message.angle = angle;
      message.depth = depth;
      message.x = localX;
      message.y = localY;
      beacon_pub.publish(message);
      ROS_INFO("Beacon Top: %s, Bottom: %s; D: %.4f; Angle: %.4f; X: %.4f; Y: %.4f.\n", c1.c_str(), c2.c_str(), dep, angle, localX, localY);      
    }
    
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
