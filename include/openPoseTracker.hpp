// C++ std libraries
#include <cstdlib>
#include <vector>
#include <thread>

// 3rd party
#include <gflags/gflags.h> // used by openpose
#include <opencv2/opencv.hpp>

// Open Pose
#include <openpose/headers.hpp>

// Ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

// // OpenPose
DEFINE_string(model_pose,   "MPI_4_layers", "`COCO` (too larger for my gpu), `MPI`, `MPI_4_layers`.");
DEFINE_string(model_folder, "/home/sassbot/openpose/models/", "Folder path for the models.");
DEFINE_string(net_resolution,           "-1x240",       "Multiples of 16. If it is increased, the accuracy potentially increases. If it is"
                                                        " decreased, the speed increases. For maximum speed-accuracy balance, it should keep the"
                                                        " closest aspect ratio possible to the images or videos to be processed. Using `-1` in"
                                                        " any of the dimensions, OP will choose the optimal aspect ratio depending on the user's"
                                                        " input value. E.g. the default `-1x368` is equivalent to `656x368` in 16:9 resolutions,"
                                                        " e.g. full HD (1980x1080) and HD (1280x720) resolutions.");
DEFINE_string(output_resolution,        "-1x-1",        "The image resolution (display and output). Use \"-1x-1\" to force the program to use the"
                                                        " input image resolution.");
DEFINE_int32(num_gpu_start, 0, "GPU device start number.");
DEFINE_double(scale_gap,                0.3,            "Scale gap between scales. No effect unless scale_number > 1. Initial scale is always 1."
                                                        " If you want to change the initial scale, you actually want to multiply the"
                                                        " `net_resolution` by your desired initial scale.");
DEFINE_int32(scale_number,              1,              "Number of scales to average.");


#define IMG_WIDTH  320
#define IMG_HEIGHT 240

// taken from https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/src/openpose/pose/poseParameters.cpp
/*
        {0,  "Head"},
        {1,  "Neck"},
        {2,  "RShoulder"},
        {3,  "RElbow"},
        {4,  "RWrist"},
        {5,  "LShoulder"},
        {6,  "LElbow"},
        {7,  "LWrist"},
        {8,  "RHip"},
        {9,  "RKnee"},
        {10, "RAnkle"},
        {11, "LHip"},
        {12, "LKnee"},
        {13, "LAnkle"},
        {14, "Chest"},
        {15, "Background"}
*/
struct Limb {
    int id;
    int partA;
    int partB;
    float ratio; // ratio of width to length
    int x_off;  // for printing if desire
    int y_off;  // ignored otherwise
    std::string name;
};

struct Person {
    int id;
    float angle;
};

#define NUM_LIMBS  12   // changing this might require other code changes
const Limb limbs[NUM_LIMBS] {
    {0,  0, 1,   0.6, 20, 10, "neck"},
    {1,  1, 14,  0.8, 20, 20, "torso"},
    {2,  2, 3,   0.3, 26, 18, "R upper arm"},
    {3,  3, 4,   0.2, 30, 25, "R lower arm"},
    {4,  5, 6,   0.3, 14, 18, "L upper arm"},
    {5,  6, 7,   0.2, 10, 25, "L lower arm"},
    {6,  14, 11, 0.5, 16, 27, "L abs"},
    {7,  11, 12, 0.3, 16, 35, "L thigh"},
    {8,  12, 13, 0.2, 16, 42, "L shin"},
    {9,  14, 8,  0.5, 24, 27, "R abs"},
    {10, 8,  9,  0.3, 24, 35, "R thigh"},
    {11, 9, 10,  0.2, 24, 42, "R shin"}
};

// for use in the histogram

#define HUE_BINS 16
typedef std::vector<float> Histogram;

// for use in printing boxes
cv::Mat hsvToBGR(HUE_BINS+2, 1, CV_8UC3);

/*
commands to put back in if you want to render the poseKeypoints

// op::PoseCpuRenderer       poseRenderer{poseModel, 0.5, true, 0.6};
// poseRenderer.initializationOnThread();
// op::CvMatToOpOutput *cvMatToOpOutput;
// op::OpOutputToCvMat *opOutputToCvMat;
// auto outputArray = cvMatToOpOutput->createArray(rgbImage->image, scaleInputToOutput, outputResolution);
// poseRenderer.renderPose(outputArray, poseKeypoints, scaleInputToOutput);
// auto outputImage = opOutputToCvMat->formatToCvMat(outputArray);
// sensor_msgs::ImagePtr msg = cv_bridge::rgbImage(std_msgs::Header(), "bgr8", outputImage).toImageMsg();
*/
