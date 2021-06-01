#include <ros/ros.h>
#include "sony_camera_interface_header.hpp"
#include <opencv4/opencv2/video/tracking.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/core/matx.hpp>
#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <vector>

static constexpr int max_points = 300;
static constexpr double threshold = 250.0;
static constexpr double distance_threshold = 10.0;

int main(int argc, char **argv)
{
    //
    // initialize ROS and create unique node name
    //
    ros::init(argc, argv, "visual_odometry_node");
    //
    // declare a node handle
    //
    ros::NodeHandle nh;
    //
    // set the maximum number of messages storable in the queue buffer
    //
    int queue_size = 1000;
    //
    // set the loop rate used by spin to control while loop execution
    // this is an integer that equates to loops/second
    //
    ros::Rate loop_rate = 2.0;
    //
    // instantiate a CameraControl object to enable
    // access to service callback and to encapsulate
    // the request value
    //
    CameraControl camera_control;
    ROS_INFO_STREAM("Camera control object, command value = " << camera_control.get_control_value());
    //
    // instantiate a service to be called to save an image to file
    //
    ros::ServiceServer camera_command_service = nh.advertiseService(
                "visual_odometry_node/camera_command",
                &CameraControl::callback,
                &camera_control);
    //
    // instantiate the subscriber for the image messages
    //
    image_transport::ImageTransport it(nh);
    ImageControl image_control;
    image_transport::Subscriber sub = it.subscribe("uvc_camera/image", 
                                        1, 
                                        &ImageControl::image_data_callback, 
                                        &image_control);

    //
    // create a variable to hold the loop count
    //
    int loop_count = 0;
    int final_count = loop_count;
    //
    // main ROS loop
    //
    int image_count = 1;
    bool show = false;
    //
    // shared file names
    //
    std::string base_path = "/catkin_ws/";
    std::string image1_name = "visual_odometry_image_1.jpg";
    std::string image2_name = "visual_odometry_image_2.jpg";
    std::string image3_name = "visual_odometry_image_3.jpg";
    std::string gray1_name = "gray_image_1.jpg";
    std::string gray2_name = "gray_image_2.jpg";
    std::string gray3_name = "gray_image_3.jpg";
    std::string homedir = getenv("HOME");
    bool initialized = false;
    bool navigating = false;
    cv::Mat image1;
    cv::Mat image2;
    cv::Mat camera_pose_R;
    cv::Mat camera_pose_t;
    std::vector<cv::Point3f> trajectory;
    double scale = 1.0;
    //
    // reading intrinsic parameters
    //
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    std::cout << "reading camera intrinsics..." << std::endl;
    cv::FileStorage fs("camera_intrinsics.yaml", cv::FileStorage::READ);
    if(!fs.isOpened()) {
        ROS_INFO_STREAM("camera not calibrated");
    }
    else {
        fs["camera_matrix"] >> camera_matrix;
        fs["dist_coeffs"] >> dist_coeffs;
        fs.release();
    }
    while (ros::ok() && loop_count > -1) {
        ++loop_count;
        final_count = loop_count;
        //
        // Check if grabbed frame is actually full with some content
        //
        image2 = image_control.get_image_data();
        ROS_INFO_STREAM("command value = " << (int)camera_control.get_control_value());
        if (40 == camera_control.get_control_value()) {
            //
            // prepare for shutdown
            //
            loop_count = -2;
        }
        else if (41 == camera_control.get_control_value()) {
            //
            // toggle display of live image
            //
            if (show) {
                show = false;
            }
            else {
                show = true;
            }
        }
        else if (44 == camera_control.get_control_value()) {
            //
            // toggle navigation variable
            //
            if (navigating) {
                navigating = false;
                initialized = false;
            }
            else {
                navigating = true;
                initialized = false;
            }
        }
        else {
            camera_control.set_control_value(idle);
        }
        if (show) {
            cv::namedWindow("view", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
            int width = 800;
            int height = 600;
            cv::resizeWindow("view", width, height);
            cv::imshow("view", image1);
            cv::waitKey(1);
        }
        else {
            cv::destroyAllWindows();
        }
        //
        // the visual odometry computations are executed until service call
        // sets the navigating variable to false
        //
        if (navigating) {
            //
            // if this is the start of navigating, we need initilaize the 
            // data in image2.
            // Otherwise, image2 is set to image1 at the end of processing
            //
            if (!initialized) {
                image1 = image2.clone();
                trajectory.clear();
                
            }
            //
            // visual odometry by pose extraction from feature correspondence
            //
            // first, run optical flow with ORB detector
            //
            // Steps
            // 1) read image 1 from file
            // 2) convert image 1 to grayscale
            // 3) find ORB features to track
            // 4) refine to subpixel corners
            // 5) read image 2 from file
            // 6) convert to grayscale
            // 7) calculate pyramid LK optical flow
            // 8) initialize camera pose in world coordinates
            // 9) find essential matrix image 1 and 2
            // 10) recover pose image 1 and 2
            // 11) transform camera pose and concatenate
            // 12) find ORB features to track on image 2
            // 13) refine to subpixel corners
            // 14) read image 3 from file
            // 15) convert image 3 to grayscale
            // 16) calculate pyramid LK optical flow
            // 17) find essential matrix image 1 and 2
            // 18) recover pose image 1 and 2
            // 19) transform camera pose and concatenate
            //           
            cv::Mat gray1;
            cv::cvtColor(image1, gray1, cv::COLOR_BGR2GRAY);
            //
            // create ORB detector to find corners in first image
            //
            int 	n_features = max_points;
            float 	scale_factor = 1.2f;
            int 	n_levels = 8;
            int 	edge_threshold = 31;
            int 	first_level = 0;
            int 	WTA_K = 2;
            cv::ORB::ScoreType 	score_type = cv::ORB::HARRIS_SCORE;
            int 	patch_size = 31;
            int 	fast_threshold = 20; 
            cv::Ptr<cv::ORB> orb_detector = cv::ORB::create(n_features,
                                                        scale_factor,
                                                        n_levels,
                                                        edge_threshold,
                                                        first_level,
                                                        WTA_K,
                                                        score_type,
                                                        patch_size,
                                                        fast_threshold);
            //
            // detect ORB keypoints in image 1
            //
            std::vector<cv::KeyPoint> keypoints1;
            orb_detector->detect(gray1, keypoints1);
            //
            // convert keypoints to vec2f for call to calcOpticalFlowPyrLK
            //
            std::vector<cv::Point2f> points1;
            cv::KeyPoint::convert(keypoints1, points1);
            //
            // refine pixel locations to subpixel accuracy
            //
            // set the half side length of the search window to 10
            // for a 20 x 20 search window
            //
            const int max_count = 20;
            double epsilon = .03;
            cv::TermCriteria term_crit = cv::TermCriteria(
                            cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                            max_count,
                            epsilon);
            const int search_win_size = 10;
            cv::cornerSubPix(gray1,
                            points1,
                            cv::Size(search_win_size, search_win_size),
                            cv::Size(-1, -1),
                            term_crit
                            );
            cv::Mat gray2;
            cv::cvtColor(image2, gray2, cv::COLOR_BGR2GRAY);
            //
            // call pyramid Lucas Kanade for images 1 and 2
            //
            int max_pyramid_level = 5;
            epsilon = .3;
            term_crit = cv::TermCriteria(
                                cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                                max_count,
                                epsilon);
            std::vector<cv::Point2f> points2;
            std::vector<uchar> flow_found_status;
            cv::calcOpticalFlowPyrLK(gray1,
                                    gray2,
                                    points1,
                                    points2,
                                    flow_found_status,
                                    cv::noArray(),
                                    cv::Size(search_win_size * 2 + 1,
                                                search_win_size * 2 + 1),
                                    max_pyramid_level,
                                    term_crit
                                    );
            std::vector<cv::Point2f> points1_trim;
            std::vector<cv::Point2f> points2_trim;
            //
            // only keep points when flow was found
            //
            for (int i = 0;
                i < static_cast<int>(points1.size());
                ++i) {
                if (flow_found_status[i] == 1) {
                    points1_trim.push_back(points1[i]);
                    points2_trim.push_back(points2[i]);
                }
                else {
                    continue;
                }
            }
            //
            // calculate descriptors for second set of points
            //
            // first, convert points to keypoints
            //
            std::vector<cv::KeyPoint> keypoints1_trim;
            cv::KeyPoint::convert(points1_trim, keypoints1_trim);
            std::vector<cv::KeyPoint> keypoints2;
            cv::KeyPoint::convert(points2_trim, keypoints2);
            //
            // compute descriptors
            //
            cv::Mat descriptors1;
            orb_detector->compute(gray1, keypoints1_trim, descriptors1);
            cv::Mat descriptors2;
            orb_detector->compute(gray2, keypoints2, descriptors2);
            //
            // filter descriptors and keypoints, rejecting features that do 
            // not match closely enough, based on L2 norm()
            //
            auto max_rows = std::min(descriptors1.rows, descriptors2.rows);
            //
            // initialize row to 0, auto type
            //
            auto row = max_rows - max_rows;
            double delta_x_sum = 0.0;
            std::vector<cv::Point2f> points1_final;
            std::vector<cv::Point2f> points2_final;
            int count = 0;
            for (row = 0; row < max_rows; ++row) {
                cv::Mat row_mat1 = descriptors1.row(row);
                cv::Mat row_mat2 = descriptors2.row(row);
                double dist = cv::norm(row_mat1 - row_mat2);
                if (dist < threshold) {
                    //
                    // descriptors match, use the points
                    // draw lines and write image to file
                    //
                    // compute slope
                    //
                    ++count;
                    points1_final.push_back(points1_trim[row]);
                    points2_final.push_back(points2_trim[row]);
                }
                else {
                    continue;
                }
            }
            std::cout << "number tracked points = " << count << std::endl;
            //
            // calculate the essential matrix for the first 2 images and
            // corresponding camera poses.
            // the mask values are 0 if the point pair is an outlier,
            // and 1 if the point pair is an inlier
            //                 
            //
            // calculate essential matrix
            //
            cv::Mat inlier_mask;
            if (count > 5) {
                std::cout << "calculating essential matrix ..." << std::endl;
                cv::Mat essential_mat = cv::findEssentialMat(points1_final,
                                                            points2_final,
                                                            camera_matrix,
                                                            cv::RANSAC,
                                                            .99,
                                                            1.0,
                                                            inlier_mask);
                std::cout << "Essential Matrix = " << essential_mat << std::endl;
                //
                // determine the relative rotation matrix, R, and relative 
                // translation matrix, T between the first 2 camera positions.
                //
                cv::Mat rotation_mat;
                cv::Mat translation_mat;                
                //
                // recover the relative pose
                //
                cv::Mat pose_inlier_mask;
                std::cout << "pose recovery..." << std::endl;
                int num_good_inliers = cv::recoverPose(essential_mat,
                                                        points1_final,
                                                        points2_final,
                                                        camera_matrix,
                                                        rotation_mat,
                                                        translation_mat,
                                                        distance_threshold,
                                                        pose_inlier_mask);
                std::cout << "number good inliers = " << num_good_inliers << std::endl;
                if (num_good_inliers < 1) {
                    std::cout << "pose recovery failed..." << std::endl;
                }
                else {
                    std::cout << "Recovered Pose Rotation Matrix = " << rotation_mat << std::endl;
                    std::cout << "Recovered Pose Translation Matrix = " << translation_mat << std::endl;
                    //
                    // initialize the camera pose to first relative pose, and scale trajectory
                    //
                    if (!initialized) {
                        camera_pose_R = rotation_mat.clone();
                        camera_pose_t = translation_mat.clone();
                    }
                    else {
                        //
                        // transform the camera pose, translation first
                        //
                        camera_pose_t = camera_pose_t + scale * (camera_pose_R * translation_mat);
                        camera_pose_R = camera_pose_R * rotation_mat;
                    }
                    //
                    // update the trajectory
                    //
                    cv::Point3f curr_point(camera_pose_t.at<double>(0), 
                                            camera_pose_t.at<double>(1), 
                                            camera_pose_t.at<double>(2));
                    trajectory.push_back(curr_point);
                    //
                    // print out results
                    //
                    std::cout << "x: " << curr_point.x 
                                << ", y: " << curr_point.y 
                                << ", z: " << curr_point.z << std::endl;
                    //
                    // prepare data for the next iteration
                    //
                    if (!initialized) {
                        initialized = true;
                    }
                }
                image1 = image2.clone();
            }
            else {
                std::cout << "tracking failed..." << std::endl;
                continue;
            }
            // end 
        }
        ros::spinOnce();
        //
        // sleep or block or go inactive until the next loop iteration
        //
        loop_rate.sleep();
        //
    }// ---- End Loop
}
