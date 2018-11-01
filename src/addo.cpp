// ------------------------- OpenPose Library Tutorial - Real Time Pose Estimation -------------------------
// If the user wants to learn to use the OpenPose library, we highly recommend to start with the `examples/tutorial_*/` folders.
// This example summarizes all the funcitonality of the OpenPose library:
    // 1. Read folder of images / video / webcam  (`producer` module)
    // 2. Extract and render body keypoint / heatmap / PAF of that image (`pose` module)
    // 3. Extract and render face keypoint / heatmap / PAF of that image (`face` module)
    // 4. Save the results on disc (`filestream` module)
    // 5. Display the rendered pose (`gui` module)
    // Everything in a multi-thread scenario (`thread` module)
    // Points 2 to 5 are included in the `wrapper` module
// In addition to the previous OpenPose modules, we also need to use:
    // 1. `core` module:
        // For the Array<float> class that the `pose` module needs
        // For the Datum struct that the `thread` module sends between the queues
    // 2. `utilities` module: for the error & logging functions, i.e. op::error & op::log respectively
// This file should only be used for the user to take specific examples.

// C++ std library dependencies
#include <atomic>
#include <chrono> // `std::chrono::` functions and classes, e.g. std::chrono::milliseconds
#include <cstdio> // sscanf
#include <cstdlib>
#include <string>
#include <thread> // std::this_thread
#include <vector>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable


// Other 3rdpary depencencies
#include <gflags/gflags.h> // DEFINE_bool, DEFINE_int32, DEFINE_int64, DEFINE_uint64, DEFINE_double, DEFINE_string
#include <boost/thread/thread.hpp>

// OpenPose dependencies

// Option a) Importing all modules
#include <openpose/headers.hpp>

// Option b) Manually importing the desired modules. Recommended if you only intend to use a few modules.
// #include <openpose/core/headers.hpp>
// #include <openpose/experimental/headers.hpp>
// #include <openpose/face/headers.hpp>
// #include <openpose/filestream/headers.hpp>
// #include <openpose/gui/headers.hpp>
// #include <openpose/pose/headers.hpp>
// #include <openpose/producer/headers.hpp>
// #include <openpose/thread/headers.hpp>
// #include <openpose/utilities/headers.hpp>
// #include <openpose/wrapper/headers.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
// #include <unsw_vision_msgs/SkeletonList.h>
// #include <unsw_vision_msgs/BodyPartDetection.h>
// #include <unsw_vision_msgs/Skeleton.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

using namespace op;

namespace vision {

// See all the available parameter options withe the `--help` flag. E.g. `./build/examples/openpose/openpose.bin --help`.
// Note: This command will show you flags for other unnecessary 3rdparty files. Check only the flags for the OpenPose
// executable. E.g. for `openpose.bin`, look for `Flags from examples/openpose/openpose.cpp:`.
// Debugging

DEFINE_int32(logging_level,             3,              "The logging level. Integer in the range [0, 255]. 0 will output any log() message, while"
                                                        " 255 will not output any. Current OpenPose library messages are in the range 0-4: 1 for"
                                                        " low priority messages and 4 for important ones.");
// OpenPose
DEFINE_string(model_pose,               "COCO",         "Model to be used. E.g. `COCO` (18 keypoints), `MPI` (15 keypoints, ~10% faster), "
                                                        "`MPI_4_layers` (15 keypoints, even faster but less accurate).");
DEFINE_string(model_folder,             "/home/hsr/.ros/models/",      "Folder path (absolute or relative) where the models (pose, face, ...) are located.");
DEFINE_string(net_resolution,           "-1x240",       "Multiples of 16. If it is increased, the accuracy potentially increases. If it is"
                                                        " decreased, the speed increases. For maximum speed-accuracy balance, it should keep the"
                                                        " closest aspect ratio possible to the images or videos to be processed. Using `-1` in"
                                                        " any of the dimensions, OP will choose the optimal aspect ratio depending on the user's"
                                                        " input value. E.g. the default `-1x368` is equivalent to `656x368` in 16:9 resolutions,"
                                                        " e.g. full HD (1980x1080) and HD (1280x720) resolutions.");
DEFINE_string(output_resolution,        "-1x-1",        "The image resolution (display and output). Use \"-1x-1\" to force the program to use the"
                                                        " input image resolution.");
DEFINE_int32(num_gpu_start,             0,              "GPU device start number.");
DEFINE_double(scale_gap,                0.3,            "Scale gap between scales. No effect unless scale_number > 1. Initial scale is always 1."
                                                        " If you want to change the initial scale, you actually want to multiply the"
                                                        " `net_resolution` by your desired initial scale.");
DEFINE_int32(scale_number,              1,              "Number of scales to average.");

class OpenPoseNodelet : public nodelet::Nodelet {


    ros::NodeHandlePtr nh;
    image_transport::Publisher skeletonImagePub;
    image_transport::Subscriber imageSub;
    ros::Publisher posePub;

    sensor_msgs::ImageConstPtr lastImage;
    cv_bridge::CvImagePtr lastCVImage;

    boost::thread *processThread;
    bool isRunning;
    std::mutex imageMutex;
    std::mutex mutex;
    std::condition_variable processWait;

  public:

    OpenPoseNodelet() {
        ROS_INFO("Creating OpenPoseNodelet");
        isRunning = true;
    }

    ~OpenPoseNodelet() {
        isRunning = false;
        processWait.notify_all();
        processThread->join();
        delete(processThread);
    }

    virtual void onInit() {

        ROS_INFO("OpenPoseNodelet::onInit starting");
        nh = ros::NodeHandlePtr(&getPrivateNodeHandle());
        processThread = new boost::thread(boost::bind(&OpenPoseNodelet::processRun, this));

        // subscribe image
        image_transport::ImageTransport it(*nh);
        imageSub = it.subscribe("image", 1, &OpenPoseNodelet::imageCallback, this);
        // skeletonImagePub = it.advertise("skeleton/image", 1);
        // posePub = nh->advertise<unsw_vision_msgs::SkeletonList>("/openpose/pose", 1);
        ROS_INFO("OpenPoseNodelet::onInit finished");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

        try {
            std::lock_guard<std::mutex> lock(imageMutex);
            lastImage = msg;
            lastCVImage = cv_bridge::toCvCopy(msg, "bgr8");
            processWait.notify_all();
        } catch (cv_bridge::Exception& e) {
            return;
        }

    }

    void processRun() {
        ROS_INFO("OpenPoseNodelet::processRun starting thread");
        std::unique_lock<std::mutex> sleep_lock(mutex);

        // Step 2 - Read Google flags (user defined configuration)
        // outputSize
        const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
        // netInputSize
        const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
        // poseModel
        const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);

        if (FLAGS_scale_gap <= 0. && FLAGS_scale_number > 1)
            op::error("Incompatible flag configuration: scale_gap must be greater than 0 or scale_number = 1.",
                    __LINE__, __FUNCTION__, __FILE__);

        // Step 3 - Initialize all required classes
        op::ScaleAndSizeExtractor scaleAndSizeExtractor(netInputSize, outputSize, FLAGS_scale_number, FLAGS_scale_gap);
        op::CvMatToOpInput cvMatToOpInput{poseModel};
        op::CvMatToOpOutput cvMatToOpOutput;
        op::PoseExtractorCaffe poseExtractorCaffe{poseModel, FLAGS_model_folder, FLAGS_num_gpu_start};
        op::OpOutputToCvMat opOutputToCvMat;

        // Step 4 - Initialize resources on desired thread (in this case single thread, i.e. we init resources here)
        poseExtractorCaffe.initializationOnThread();

        // Step 1 - Read and load image, error if empty (possibly wrong path)
        const op::Point<int> imageSize{640, 480};
        //const op::Point<int> imageSize{cv_ptr->image.cols, cv_ptr->image.rows};

        // Step 2 - Get desired scale sizes
        std::vector<double> scaleInputToNetInputs;
        std::vector<op::Point<int>> netInputSizes;
        double scaleInputToOutput;
        op::Point<int> outputResolution;
        std::tie(scaleInputToNetInputs, netInputSizes, scaleInputToOutput, outputResolution)
            = scaleAndSizeExtractor.extract(imageSize);

        ROS_INFO("OpenPoseNodelet::processRun ready");
        sensor_msgs::ImageConstPtr image;
        cv_bridge::CvImagePtr cv_ptr;
        while(isRunning){
            processWait.wait(sleep_lock);
            {
                std::lock_guard<std::mutex> lock(imageMutex);
                cv_ptr = lastCVImage;
                image = lastImage;
            }
            if(!isRunning){
                break;
            }
            ros::Time t = ros::Time::now();

            if (cv_ptr->image.empty()) return;

            // Step 3 - Format input image to OpenPose input and output formats
            const auto netInputArray = cvMatToOpInput.createArray(cv_ptr->image, scaleInputToNetInputs, netInputSizes);
            //auto outputArray = cvMatToOpOutput.createArray(cv_ptr->image, scaleInputToOutput, outputResolution);

            // Step 4 - Estimate poseKeypoints
            poseExtractorCaffe.forwardPass(netInputArray, imageSize, scaleInputToNetInputs);
            const auto poseKeypoints = poseExtractorCaffe.getPoseKeypoints();

            // // publish annotations.
            // unsw_vision_msgs::SkeletonList skeletonList;
            // skeletonList.header = image->header;
            // skeletonList.image_w = cv_ptr->image.cols;
            // skeletonList.image_h = cv_ptr->image.rows;
            //
            //
            // const int num_people = poseKeypoints.getSize(0);
            // const int num_bodyparts = poseKeypoints.getSize(1);
            //
            // for (size_t person_idx = 0; person_idx < num_people; person_idx++) {
            //     unsw_vision_msgs::Skeleton person;
            //     for (size_t bodypart_idx = 0; bodypart_idx < num_bodyparts; bodypart_idx++) {
            //         size_t final_idx = 3*(person_idx*num_bodyparts + bodypart_idx);
            //         unsw_vision_msgs::BodyPartDetection bodypart;
            //         bodypart.part_id = bodypart_idx;
            //         bodypart.x = poseKeypoints[final_idx];
            //         bodypart.y = poseKeypoints[final_idx+1];
            //         bodypart.confidence = poseKeypoints[final_idx+2];
            //         person.body_part.push_back(bodypart);
            //     }
            //     skeletonList.list.push_back(person);
            // }
            // posePub.publish(skeletonList);

            // publish result image with annotation.
            /*
            if (!FLAGS_result_image_topic.empty()) {
                poseRenderer->renderPose(outputArray, poseKeypoints);
                //faceRenderer->renderFace(outputArray, faceKeypoints);
                auto outputImage = opOutputToCvMat->formatToCvMat(outputArray);

                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage).toImageMsg();
                skeletonImagePub.publish(msg);
            }
            */
        }
    }

};

}
PLUGINLIB_EXPORT_CLASS(vision::OpenPoseNodelet, nodelet::Nodelet)
