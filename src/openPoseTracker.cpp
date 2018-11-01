#include "openPoseTracker.hpp"

#define IMAGE_PRINTING true
#define HIST_PRINTING  false // needs IMAGE_PRINTING true
#define CORR_PRINTING  true // needs IMAGE_PRINTING true
#define LIMB_PRINTING  true // needs IMAGE_PRINTING true

// discard use of limb if below this level
#define MIN_POSE_KEY_CERTAINTY 0.3

// for use in the histograms
#define INIT_HIST_SAVES 20
#define MIN_BRIGHTNESS  15
#define SAT_FOR_GREY    30

// minimum correlation to decide its the original person
// and the minimum number of limbs that have had to have been measured to get such
#define MIN_CORRELATION 0.75
#define MIN_MATCHES     5

using namespace op;

// Globals
cv_bridge::CvImagePtr rgbImagePtr;
cv::Mat hsvImage(IMG_WIDTH, IMG_HEIGHT, CV_8UC3);

float avgBin;
int toSave = NUM_LIMBS;
Histogram originalLimbHist[NUM_LIMBS];
int histsToSave[NUM_LIMBS];
float originalVariances[NUM_LIMBS];

volatile bool newImage = false;

ros::Subscriber imgSub;
ros::Publisher  imgPub;
ros::Publisher  cmdPub;
ros::Publisher  anglePub;

// function declarationsss
void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
void sendCommand(std::string string);
void sendAngle(float angle);
float toAngle(float x);

Person centralPersonIdx(Array<float> poseKeypoints);
float compareLimbHistograms(Array<float> poseKeypoints);
void saveLimbHistogramsOf(Array<float> poseKeypoints, int person);
void paintSavedLimbCircles();
void highlightPixel(int x, int y);

void paintHistogram(cv::Point point, Histogram &hist);
void smoothOutHistogram(Histogram &hist);
void normaliseHistogram(Histogram &hist);
Histogram getLimbHistogram(cv::Point2f pA, cv::Point2f pB, float ratio);
float getHistCorrelation(Histogram &originalHist, float originalVariance, Histogram &hist);

int main(int argc, char** argv ) {

    // set 0 set up ros topics
    ros::init(argc, argv, "OpenPose");
    ros::NodeHandle nh;
    ros::Rate r(1);

    imgSub   = nh.subscribe("/camera/color/image_raw/compressed", 1, &imageCallback);
    imgPub   = nh.advertise<sensor_msgs::Image>("/poseImage", 1);
    cmdPub   = nh.advertise<std_msgs::String>("/cmd", 1);
    anglePub = nh.advertise<std_msgs::Float32>("/angle", 1);

    // step 1 initialise some globals
    if (HIST_PRINTING){
        float binSize = 180.0 / (float) HUE_BINS;
        for (int i=0; i< HUE_BINS; i++) {
            hsvToBGR.at<cv::Vec3b>(cv::Point(i, 0)) = cv::Vec3b((int)binSize*(0.5+i), 250, 240);
        }
        // grey bins
        hsvToBGR.at<cv::Vec3b>(cv::Point(HUE_BINS, 0)) = cv::Vec3b(0, 0, 180);
        hsvToBGR.at<cv::Vec3b>(cv::Point(HUE_BINS+1, 0)) = cv::Vec3b(0, 0, 40);
        cv::cvtColor(hsvToBGR, hsvToBGR, CV_HSV2BGR);
    }
    avgBin = 1.0 / (float)(HUE_BINS + 1);
    for (int i=0; i<NUM_LIMBS; i++) {
        histsToSave[i] = INIT_HIST_SAVES;
        originalLimbHist[i] = Histogram(HUE_BINS + 2, 0.0);
    }

    // Step 2 use flags to set up some network parameters
    const auto outputSize   = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
    const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
    const auto poseModel    = op::flagsToPoseModel(FLAGS_model_pose);

    // Step 3 - Initialize networks
    op::ScaleAndSizeExtractor scaleAndSizeExtractor(netInputSize, outputSize, FLAGS_scale_number, FLAGS_scale_gap);
    op::CvMatToOpInput        cvMatToOpInput{poseModel};
    op::PoseExtractorCaffe    poseExtractorCaffe{poseModel, FLAGS_model_folder, FLAGS_num_gpu_start};

    // Step 4 - Initialize resources on desired
    poseExtractorCaffe.initializationOnThread();

    // Not really sure but ok. scaly things
    const op::Point<int> imageSize{IMG_WIDTH, IMG_HEIGHT};
    std::vector<double> scaleInputToNetInputs;
    std::vector<op::Point<int>> netInputSizes;
    double scaleInputToOutput;
    op::Point<int> outputResolution;
    std::tie(scaleInputToNetInputs, netInputSizes, scaleInputToOutput, outputResolution)
            = scaleAndSizeExtractor.extract(imageSize);


    // ------------------------------------------------------------------------
    // Step 5 Initialise the histograms
    // ------------------------------------------------------------------------
    ROS_INFO("OpenPoseNode network set up, Initialising histograms");
    while(ros::ok()){

        // Wait for an image. Spin once allows a callback to occur
        ros::spinOnce();
        if (!newImage) continue;

        // Pass it through the network
        const auto netInputArray = cvMatToOpInput.createArray(rgbImagePtr->image, scaleInputToNetInputs, netInputSizes);
        poseExtractorCaffe.forwardPass(netInputArray, imageSize, scaleInputToNetInputs);
        const auto poseKeypoints = poseExtractorCaffe.getPoseKeypoints();

        // find central person to scan.
        Person centralPerson = centralPersonIdx(poseKeypoints);
        if (centralPerson.id != -1) {

            sendCommand("turn");
            sendAngle(centralPerson.angle);

            // only start scanning if angle small enough
            if (fabs(centralPerson.angle) < 10) {

                cvtColor(rgbImagePtr->image, hsvImage, cv::COLOR_BGR2HSV);
                saveLimbHistogramsOf(poseKeypoints, centralPerson.id);
            }
        }

        if (IMAGE_PRINTING) {
            paintSavedLimbCircles();
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbImagePtr->image).toImageMsg();
            imgPub.publish(msg);
        }

        // finally, reset the image lock, and go to the next section if done
        newImage = false;
        if (toSave <= 0) break;
    }

    // ------------------------------------------------------------------------
    // Step 6 Tracking the person
    // ------------------------------------------------------------------------
    ROS_INFO("Histograms Saved, Tracking the person");

    while(ros::ok()){

        // Wait for an image. Spin once allows a callback to occur
        ros::spinOnce();
        if (!newImage) continue;

        // Pass it through the network
        const auto netInputArray = cvMatToOpInput.createArray(rgbImagePtr->image, scaleInputToNetInputs, netInputSizes);
        poseExtractorCaffe.forwardPass(netInputArray, imageSize, scaleInputToNetInputs);
        const auto poseKeypoints = poseExtractorCaffe.getPoseKeypoints();

        // find correspondences.
        cvtColor(rgbImagePtr->image, hsvImage, cv::COLOR_BGR2HSV);
        float angle = compareLimbHistograms(poseKeypoints);

        if (IMAGE_PRINTING) {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbImagePtr->image).toImageMsg();
            imgPub.publish(msg);
        }

        // finally, reset the image lock, and go to the next section if done
        newImage = false;
    }
}

void sendCommand(std::string string) {
    std_msgs::String msg;
    msg.data = string;
    cmdPub.publish(msg);
}

void sendAngle(float angle) {
    std_msgs::Float32 msg;
    msg.data = angle;
    anglePub.publish(msg);
}

void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    if (newImage) return;
    rgbImagePtr = cv_bridge::toCvCopy(msg, "bgr8");
    newImage = true;
}

void paintSavedLimbCircles() {
    for (const auto limb : limbs) {
        cv::Vec3b color(90, 255, 30);   // green
        if (histsToSave[limb.id] > 0) color = cv::Vec3b(80, 100, 255); // orange
        if (histsToSave[limb.id] == INIT_HIST_SAVES) color = cv::Vec3b(30, 30, 255); // red
        circle(rgbImagePtr->image, cv::Point(limb.x_off, limb.y_off), 3, color, -1);
    }
}

float compareLimbHistograms(Array<float> poseKeypoints) {

    const int num_people = poseKeypoints.getSize(0);
    const int num_bodyparts = poseKeypoints.getSize(1);

    float bestProb = 0;
    float bestAngle;

    for (int person = 0; person < num_people; person++) {
        float avgXOffest = 0;
        float avProb  = 0;
        float numLimbs = 0;
        for (const auto limb : limbs) {
            int partA_idx = 3*(person*num_bodyparts + limb.partA);
            int partB_idx = 3*(person*num_bodyparts + limb.partB);

            // check proabilities of both
            if (poseKeypoints[partA_idx+2] < MIN_POSE_KEY_CERTAINTY ||
                poseKeypoints[partB_idx+2] < MIN_POSE_KEY_CERTAINTY)
                continue;

            cv::Point2f pointA(poseKeypoints[partA_idx], poseKeypoints[partA_idx+1]);
            cv::Point2f pointB(poseKeypoints[partB_idx], poseKeypoints[partB_idx+1]);

            Histogram hist = getLimbHistogram(pointA, pointB, limb.ratio);
            float correlation = getHistCorrelation(originalLimbHist[limb.id], originalVariances[limb.id], hist);

            avgXOffest += pointA.x;
            avProb += correlation;
            numLimbs ++;

            if (IMAGE_PRINTING && CORR_PRINTING) {
                cv::Vec3b color((int)255*correlation, (int)255*correlation, (int)255*(1-correlation));
                putText(rgbImagePtr->image, std::to_string((int)(correlation*100)), pointA, cv::FONT_HERSHEY_DUPLEX, 0.3, color, 0.5 );
            }
        }

        avProb  /= numLimbs;
        avgXOffest /= numLimbs;

        if (numLimbs > MIN_MATCHES && avProb > bestProb) {
            bestProb  = avProb;
            bestAngle = toAngle(avgXOffest);
        }
    }

    // publish the best angle if the correlation is high enough
    if (bestProb > MIN_CORRELATION) {
        ROS_INFO("Matched a person correlation %lf at angle %lf", bestProb, bestAngle);
        sendCommand("follow");
        sendAngle(bestAngle);
    }
}

float getHistCorrelation(Histogram &originalHist, float originalVariance, Histogram &newHist) {
    float newVariance = 0;
    float covariance = 0;
    for (int i=0; i < originalHist.size(); i++) {
        covariance += (originalHist[i] - avgBin) * (newHist[i] - avgBin);
        float stdev = newHist[i] - avgBin;
        newVariance  += stdev * stdev;
    }
    float correlation = covariance/sqrt(originalVariance*newVariance);
    if (correlation < 0) correlation = 0;
    return correlation;
}

float getVariance(Histogram &hist){
    float variance = 0;
    for (int i=0; i < hist.size(); i++) {
        float stdev = hist[i] - avgBin;
        variance += stdev * stdev;
    }
    return variance;
}

void saveLimbHistogramsOf(Array<float> poseKeypoints, int person){

    const int num_bodyparts = poseKeypoints.getSize(1);

    for (const auto limb : limbs) {

        // don't bother remembering old ones
        if (histsToSave[limb.id] <= 0) continue;

        int partA_idx = 3*(person*num_bodyparts + limb.partA);
        int partB_idx = 3*(person*num_bodyparts + limb.partB);

        // check proabilities of both
        if (poseKeypoints[partA_idx+2] < MIN_POSE_KEY_CERTAINTY ||
            poseKeypoints[partB_idx+2] < MIN_POSE_KEY_CERTAINTY)
            continue;

        cv::Point2f pointA(poseKeypoints[partA_idx], poseKeypoints[partA_idx+1]);
        cv::Point2f pointB(poseKeypoints[partB_idx], poseKeypoints[partB_idx+1]);

        // add the histogram
        Histogram newHist = getLimbHistogram(pointA, pointB, limb.ratio);
        for (int i=0; i<newHist.size(); i++) originalLimbHist[limb.id][i] += newHist[i];
        histsToSave[limb.id] -= 1;

        // If the end is hit, normalise the histogram
        if (histsToSave[limb.id] <= 0) {
            toSave -= 1;
            normaliseHistogram(originalLimbHist[limb.id]);
            originalVariances[limb.id] = getVariance(originalLimbHist[limb.id]);
        }
    }
}

void highlightPixel(int x, int y){
    cv::Vec3b pix = rgbImagePtr->image.at<cv::Vec3b>(cv::Point(x, y));
    pix[0] = (255 + pix[0])/2;
    pix[1] = (255 + pix[1])/2;
    pix[2] = (255 + pix[2])/2;
    rgbImagePtr->image.at<cv::Vec3b>(cv::Point(x, y)) = pix;
}

Histogram getLimbHistogram(cv::Point2f pA, cv::Point2f pB, float ratio) {

    // stride is the width of the limb, found by multiplying the
    // length of the limb by its width ratio
    float dx = fabs(pA.x - pB.x);
    float dy = fabs(pA.y - pB.y);
    float halfStride = ratio * sqrt(dx*dx + dy*dy)/2;

    // count up the pixels as you go
    Histogram histogram(HUE_BINS, 0);
    float numlightGreys = 1; // start at 1 to avoid odd possibility of no pixels
    float numDarkGreys  = 1; // start at 1 to avoid odd possibility of no pixels

    // horizontal or vertical sweeps depend on longer side. if x is
    // larger, then doing vertical strides
    if (dx > dy) {
        cv::Point2f left  = pA.x < pB.x ? pA : pB;
        cv::Point2f right = pA.x > pB.x ? pA : pB;
        float gradient = (right.y - left.y) / (right.x - left.x);
        float y_base = left.y;
        for (int x = (int)left.x; x <= (int)right.x; x++) {

            // get strip bounds
            y_base += gradient;
            int y1 = (int) (y_base - halfStride);
            int y2 = (int) (y_base + halfStride);
            if (y1 < 0) y1 = 0;
            if (y2 >= IMG_HEIGHT) y2 = IMG_HEIGHT - 1;

            // print vertical strip
            for (int y = y1; y <= y2; y++) {

                cv::Vec3b hsv = hsvImage.at<cv::Vec3b>(cv::Point(x, y));
                // ignore if too dark, or add as grey if not saturated
                if      (hsv[2] < MIN_BRIGHTNESS) numDarkGreys ++;
                else if (hsv[1] < SAT_FOR_GREY) {
                    if (hsv[2] < 40) numDarkGreys ++;
                    else numlightGreys ++;
                } else  {
                    histogram[hsv[0]*HUE_BINS/180]++;
                }


                if (IMAGE_PRINTING && LIMB_PRINTING) highlightPixel(x, y);
            }
        }

    // Horizontal strides then...
    } else {
        cv::Point2f left  = pA.y < pB.y ? pA : pB;
        cv::Point2f right = pA.y > pB.y ? pA : pB;
        float x_gradient = (right.x - left.x) / (right.y - left.y);
        float x_base = left.x;
        for (int y = (int)left.y; y <= (int)right.y; y++) {

            // get strip bounds
            x_base += x_gradient;
            int x1 = (int) (x_base - halfStride);
            int x2 = (int) (x_base + halfStride);
            if (x1 < 0) x1 = 0;
            if (x2 >= IMG_WIDTH) x2 = IMG_WIDTH - 1;

            // print horizontal strip
            for (int x = x1; x <= x2; x++) {

                cv::Vec3b hsv = hsvImage.at<cv::Vec3b>(cv::Point(x, y));
                // ignore if too dark, or add as grey if not saturated
                if      (hsv[2] < MIN_BRIGHTNESS) numDarkGreys ++;
                else if (hsv[1] < SAT_FOR_GREY) {
                    if (hsv[2] < 40) numDarkGreys ++;
                    else numlightGreys ++;
                } else  {
                    histogram[hsv[0]*HUE_BINS/180]++;
                }

                if (IMAGE_PRINTING && LIMB_PRINTING) highlightPixel(x, y);
            }
        }
    }

    // smooth out the histogram
    smoothOutHistogram(histogram);

    // add the grey
    histogram.push_back(numlightGreys);
    histogram.push_back(numDarkGreys);

    // normalise
    normaliseHistogram(histogram);

    if (IMAGE_PRINTING && HIST_PRINTING) paintHistogram(pA, histogram);

    return histogram;
}

void paintHistogram(cv::Point point, Histogram &hist) {
    for (int i = 0; i < hist.size(); i++) {
        cv::Point UL(point);
        cv::Point LR(point);
        UL.x += 3*i;
        LR.x += 3*(i+1);
        if (LR.x >= IMG_WIDTH || UL.y < 0) continue;
        LR.y -= hist[i]*60;
        cv::Vec3b color = hsvToBGR.at<cv::Vec3b>(cv::Point(i, 0));
        cv::rectangle(rgbImagePtr->image, UL, LR, color, -1);
    }
}

void smoothOutHistogram(Histogram &hist) {

    // hues
    Histogram temp(hist.size(), 0);
    for (int i = 0; i < HUE_BINS; i++) {

        // left
        if (i == 0) temp[i] += hist[HUE_BINS-1] * 0.1;
        else        temp[i] += hist[i-1]        * 0.1;

        // center
        temp[i] += hist[i] * 0.8;

        // right
        if (i == HUE_BINS-1) temp[i] += hist[0]   * 0.1;
        else                 temp[i] += hist[i+1] * 0.1;
    }

    // greys
    temp[HUE_BINS]   = hist[HUE_BINS] * 0.8 + hist[HUE_BINS+1] * 0.2;
    temp[HUE_BINS+1] = hist[HUE_BINS+1] * 0.8 + hist[HUE_BINS] * 0.2;

    // put back in original
    for (int i = 0; i < hist.size(); i++) hist[i] = temp[i];
}

void normaliseHistogram(Histogram &hist) {
    float sum = 0;
    for (int i = 0; i < hist.size(); i++) sum += hist[i];
    for (int i = 0; i < hist.size(); i++) hist[i] /= sum;
}

float toAngle(float x) {
    float width = (float) IMG_WIDTH;
    return (x - width/2) / (width/2) * (70/2) ;
}

// returns -1 if person not found with enough confidence
Person centralPersonIdx(Array<float> poseKeypoints) {

    const int num_people = poseKeypoints.getSize(0);
    const int num_bodyparts = poseKeypoints.getSize(1);

    Person best = {-1, -1000};

    for (int person_idx = 0; person_idx < num_people; person_idx++) {
        float avg_certainty = 0;
        float avg_x = 0;
        for (int bodypart_idx = 0; bodypart_idx < num_bodyparts; bodypart_idx++) {
            int final_idx = 3*(person_idx*num_bodyparts + bodypart_idx);
            avg_certainty += poseKeypoints[final_idx+2];
            avg_x += poseKeypoints[final_idx+0];
        }

        avg_certainty /= (float) num_bodyparts;
        float angle = toAngle(avg_x / (float) num_bodyparts);

        // update the best person
        if (avg_certainty > MIN_POSE_KEY_CERTAINTY &&
            fabs(angle)   < fabs(best.angle) ) {
            best.id = person_idx;
            best.angle = angle;
        }
    }
    return best;
}
