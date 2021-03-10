#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/persistence.hpp>

#define ESCAPE      27
#define SPACE       32
#define BACKSPACE   8

static cv_bridge::CvImagePtr cv_ptr;
static cv::Mat frame;
static const char *window_name_live = "Live Image";
static const char *window_name_forward_tracking = "Forward Tracking";
static const char *window_name_reverse_tracking = "Reverse Tracking";
static const char *window_name_verified_tracking = "Verified Tracking";
static const char *forwardDataFileName = "forwardData.yaml";
static const char *reverseDataFileName = "reverseData.yaml";

//
// subscribe to the image_raw topic to be notified when a new image
// is available.  Assign the image pointer to the Mat.
//
void getImage(const sensor_msgs::Image raw_image)
{
    //
    // convert raw image to Mat data type using cv bridge
    //
    try {
        cv_ptr = cv_bridge::toCvCopy(raw_image,
                                     sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    frame = cv_ptr->image;
}

//
// compare the x and y values of 2 points
// return true if the x and y values are within max error of each other
// otherwise return false
//
bool comparePoints(cv::Point2f point1, cv::Point2f point2, float maxDiff)
{
    float xError = fabs(point1.x - point2.x);
    float yError = fabs(point1.y - point2.y);
    if (xError < maxDiff && yError < maxDiff) {
        return true;
    }
    else {
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "optical_flow");
    ros::NodeHandle nh;
    //
    // set the loop rate used by spin to control while loop execution
    // this is an integer that equates to loops/second
    //
    ros::Rate loop_rate = 10;
    //
    // instantiate the subscriber for the raw image messages
    //
    ros::Subscriber image_sub = nh.subscribe("image_raw", 1000, getImage);
    //
    // loop while processing images and tracking keypoint motion
    // create overlay to depict motion and show image with overlay
    //
    int count = 0;
    ROS_INFO_STREAM("hit <Esc> to close image window...");
    bool trackingComplete = false;
    bool featuresInitialized = false;
    cv::Mat firstImage;
    cv::Mat secondImage;
    cv::Mat firstGrayImg;
    cv::Mat secondGrayImg;
    cv::Mat forwardTrackingImage;
    cv::Mat reverseTrackingImage;
    cv::Mat verifiedTrackingImage;
    std::vector<cv::Point2f> firstForwardPoints;
    std::vector<cv::Point2f> secondForwardMatches;
    std::vector<cv::Point2f> firstReverseMatches;
    std::vector<cv::Point2f> firstVerifiedPoints;
    std::vector<cv::Point2f> secondVerifiedPoints;
    std::vector<uchar> forwardTrackedFeatures;
    std::vector<uchar> reverseTrackedFeatures;
    const int maxCorners = 500;
    const double qualityLevel = .03;
    const double minDistance = 10.0;
    const int blockSize = 5;
    bool useHarris = true;
    double harrisFreeParameter = .04;
    const int searchWinSize = 10;
    const int maxCount = 20;
    double epsilon = .03;
    cv::TermCriteria termCrit;
    while(ros::ok()) {
        //
        // capture first frame and find the good features to track
        // which will be used with successive images during tracking
        //
        if (!featuresInitialized) {
            if(frame.rows > 0 && frame.cols > 0) {
                frame.copyTo(firstImage);
                frame.copyTo(forwardTrackingImage);
                frame.copyTo(reverseTrackingImage);
                frame.copyTo(verifiedTrackingImage);
                //
                // convert to grayscale
                //
                cv::cvtColor(firstImage, firstGrayImg, cv::COLOR_BGR2GRAY);
                //
                // call good features to track to find corners
                //
                cv::goodFeaturesToTrack(firstGrayImg,
                                        firstForwardPoints,
                                        maxCorners,
                                        qualityLevel,
                                        minDistance,
                                        cv::noArray(),
                                        blockSize,
                                        useHarris,
                                        harrisFreeParameter
                                        );
                //
                // refine pixel locations to subpixel accuracy
                //
                // set the half side length of the search window to 10
                // for a 20 x 20 search window
                //
                termCrit = cv::TermCriteria(
                            cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                            maxCount,
                            epsilon);
                cv::cornerSubPix(firstGrayImg,
                                 firstForwardPoints,
                                 cv::Size(searchWinSize, searchWinSize),
                                 cv::Size(-1, -1),
                                 termCrit
                                 );
                //
                // done initializing, set the flag to true
                //
                featuresInitialized = true;
            }
        }
        else if(count > 5 && !trackingComplete) {
            if(frame.rows > 0 && frame.cols > 0) {
                //
                // capture second frame and estimate the camera motion
                // using pyramid Lucas-Kanade
                //
                frame.copyTo(secondImage);
                //
                // convert to grayscale
                //
                cv::cvtColor(secondImage, secondGrayImg, cv::COLOR_BGR2GRAY);
                //
                // call pyramid Lucas Kanade
                //
                int maxPyramidLevel = 5;
                epsilon = .3;
                termCrit = cv::TermCriteria(
                            cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                            maxCount,
                            epsilon);
                cv::calcOpticalFlowPyrLK(firstGrayImg,
                                         secondGrayImg,
                                         firstForwardPoints,
                                         secondForwardMatches,
                                         forwardTrackedFeatures,
                                         cv::noArray(),
                                         cv::Size(searchWinSize * 2 + 1,
                                                  searchWinSize * 2 + 1),
                                         maxPyramidLevel,
                                         termCrit
                                         );
                //
                // now, find reverse matches
                // use the matches from the first run as the starting point
                // and then run pyramid lucas kanade with the images in
                // reverse order.
                //
                // compare the forward and reverse point matches and
                // only keep those that ocurred in both directions
                //
                // call pyramid Lucas Kanade
                //
                maxPyramidLevel = 5;
                epsilon = .3;
                termCrit = cv::TermCriteria(
                            cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                            maxCount,
                            epsilon);
                cv::calcOpticalFlowPyrLK(secondGrayImg,
                                         firstGrayImg,
                                         secondForwardMatches,
                                         firstReverseMatches,
                                         reverseTrackedFeatures,
                                         cv::noArray(),
                                         cv::Size(searchWinSize * 2 + 1,
                                                  searchWinSize * 2 + 1),
                                         maxPyramidLevel,
                                         termCrit
                                         );
                //
                // compare the first set of starting features to the
                // reverse matches, only keep points that match
                //
                for (int i = 0;
                     i < static_cast<int>(secondForwardMatches.size());
                     ++i) {
                    //
                    // if a reverse match was found
                    //
                    if (reverseTrackedFeatures[i] == 1) {
                        if (comparePoints(firstForwardPoints[i], firstReverseMatches[i], (float)10.0)) {
                            firstVerifiedPoints.push_back(firstReverseMatches[i]);
                            secondVerifiedPoints.push_back(secondForwardMatches[i]);
                        }
                    }
                }
                //
                // overlay the points onto the first color image and dispay it
                // use a for loop, draws lines between first points and
                // second points, as long as a feature was found, draw
                // each line on the tracked image, then show the image in
                // a named window
                //
                cv::Scalar lineColorBlue(255, 0, 0);
                cv::Scalar lineColorRed(0, 0, 255);
                int lineThickness = 3;
                int lineType = cv::LINE_AA;
                for (int i = 0;
                     i < static_cast<int>(firstForwardPoints.size());
                     ++i) {
                    if (forwardTrackedFeatures[i] == 1) {
                        cv::line(forwardTrackingImage,
                                 firstForwardPoints[i],
                                 secondForwardMatches[i],
                                 lineColorBlue,
                                 lineThickness,
                                 lineType
                                 );
                    }
                    if (reverseTrackedFeatures[i] == 1) {
                        cv::line(reverseTrackingImage,
                                 secondForwardMatches[i],
                                 firstReverseMatches[i],
                                 lineColorRed,
                                 lineThickness,
                                 lineType
                                 );
                    }
                    else {
                        continue;
                    }
                }
                //
                // overlay the points onto the first color image and display it
                // use a for loop, draws lines between first points and
                // second points, then show the image in
                // a named window
                //
                cv::Scalar lineColorGreen(0, 255, 0);
                for (int i = 0;
                     i < static_cast<int>(firstVerifiedPoints.size());
                     ++i) {
                    cv::line(verifiedTrackingImage,
                             firstVerifiedPoints[i],
                             secondVerifiedPoints[i],
                             lineColorGreen,
                             lineThickness,
                             lineType
                             );
                }
            }
            trackingComplete = true;
            ROS_INFO_STREAM("tracking complete...");
            //
            // display the tracking images
            //
            if (frame.rows > 0 && frame.cols > 0) {
                if (trackingComplete) {
                    cv::namedWindow( window_name_forward_tracking,
                                     cv::WINDOW_NORMAL || cv::WINDOW_KEEPRATIO);
                    cv::imshow(window_name_forward_tracking, forwardTrackingImage);
                    cv::namedWindow( window_name_reverse_tracking,
                                     cv::WINDOW_NORMAL || cv::WINDOW_KEEPRATIO);
                    cv::imshow(window_name_reverse_tracking, reverseTrackingImage);
                    cv::namedWindow( window_name_verified_tracking,
                                     cv::WINDOW_NORMAL || cv::WINDOW_KEEPRATIO);
                    cv::imshow(window_name_verified_tracking, verifiedTrackingImage);
                }
            }

        }
        else if (count == 10){
            //
            // clear the verified points vectors
            //
            firstVerifiedPoints.clear();
            secondVerifiedPoints.clear();
            featuresInitialized = false;
            trackingComplete = false;
            count = 0;
        }
        else {
            ++count;
            char c = (char)(cv::waitKey(1));
            //
            // if char is escape, break out of while loop
            //
            if (c == ESCAPE) {
                break;
            }
            //
            // display the live image
            //
            cv::namedWindow( window_name_live,
                             cv::WINDOW_NORMAL || cv::WINDOW_KEEPRATIO);
            cv::imshow(window_name_live, frame);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    cv::destroyAllWindows();
}
