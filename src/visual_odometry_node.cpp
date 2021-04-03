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

static constexpr int max_points = 100;

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
    std::string gray1_name = "gray_image_1.jpg";
    std::string gray2_name = "gray_image_2.jpg";
    std::string homedir = getenv("HOME");
    while (ros::ok() && loop_count > -1) {
        ++loop_count;
        final_count = loop_count;
        //
        // Check if grabbed frame is actually full with some content
        //
        cv::Mat data = image_control.get_image_data();
        if (show) {
            cv::namedWindow("view", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
            int width = 800;
            int height = 600;
            cv::resizeWindow("view", width, height);
            cv::imshow("view", data);
            cv::waitKey(1);
        }
        else {
            cv::destroyAllWindows();
        }
        ROS_INFO_STREAM("command value = " << (int)camera_control.get_control_value());
        if (30 == camera_control.get_control_value()) {
            //
            // prepare for shutdown
            //
            loop_count = -2;
        }
        else if (31 == camera_control.get_control_value()) {
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
        else if (32 == camera_control.get_control_value()) {
            //
            // save image to file, default compression
            //
            std::stringstream path_ss;
            path_ss << homedir;
            path_ss << base_path;
            if (image_count == 1) {
                path_ss << image1_name;
            }
            if (image_count == 2) {
                path_ss << image2_name;
            }
            std::string path;
            path_ss >> path;
            cv::imwrite(path, data);
            ++image_count;
            if (image_count == 3) {
                //
                // reset count and overwrite old data
                //
                image_count = 1;
            }
        }
        else if (34 == camera_control.get_control_value()) {
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
            // 8) calculate lines between matching points and overlay line 
            //    on first image
            //           
            std::stringstream path1_ss;
            path1_ss << homedir;
            path1_ss << base_path;
            path1_ss << image1_name;
            std::string path1;
            path1_ss >> path1;
            //
            // read image 1, convert to grayscale
            //
            cv::Mat image1 = cv::imread(path1);
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
            // detect ORB keypoints
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
            std::stringstream path2_ss;
            path2_ss << homedir;
            path2_ss << base_path;
            path2_ss << image2_name;
            std::string path2;
            path2_ss >> path2;
            //
            // read image 2, convert to grayscale
            //
            cv::Mat image2 = cv::imread(path2);
            cv::Mat gray2;
            cv::cvtColor(image2, gray2, cv::COLOR_BGR2GRAY);
            //
            // call pyramid Lucas Kanade
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
            cv::Scalar line_color(0, 0, 255);
            int line_thickness = 5;
            int line_type = cv::LINE_AA;
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
                    cv::line(image1,
                            points1[i],
                            points2[i],
                            line_color,
                            line_thickness,
                            line_type
                            );
                }
                else {
                    continue;
                }
            }
            std::string orb_tracked_name = "orb_tracked_image.jpg";
            std::stringstream tracked_ss;
            tracked_ss << homedir;
            tracked_ss << base_path;
            tracked_ss << orb_tracked_name;
            std::string tracked_path;
            tracked_ss >> tracked_path;
            cv::imwrite(tracked_path, image1);
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
            const double threshold = 250.0;
            line_color = cv::Scalar(0, 255, 0);
            cv::Mat image3 = cv::imread(path2);
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
                        double x1 = points1_trim[row].x;
                        double y1 = points1_trim[row].y;
                        double x2 = points2_trim[row].x;
                        double y2 = points2_trim[row].y;
                        double delta_y = abs(y2 - y1);
                        ++count;
                        delta_x_sum += (abs(x2 - x1));
                        if (delta_y < threshold) {
                            points1_final.push_back(points1_trim[row]);
                            points2_final.push_back(points2_trim[row]);
                        }
                    }
                    else {
                        continue;
                    }
            }
            double delta_x_ave = delta_x_sum/(double)count;
            double img_width = (double)image3.cols;
            double percent_overlap = 100.0 * (img_width - delta_x_ave)/img_width;
            std::string orb_tracked_trim_name = "orb_tracked_image_trim.jpg";
            std::stringstream tracked_trim_ss;
            tracked_trim_ss << homedir;
            tracked_trim_ss << base_path;
            tracked_trim_ss << orb_tracked_trim_name;
            std::string tracked_trim_path;
            tracked_trim_ss >> tracked_trim_path;
            cv::imwrite(tracked_trim_path, image3);
            //
            // calculate the essential matrix
            //

            //
            // calculate the essential matrix for the 2 images and
            // corresponding camera poses.
            // the mask values are 0 if the point pair is an outlier,
            // and 1 if the point pair is an inlier
            //
            cv::Mat camera_matrix;
            cv::Mat dist_coeffs;
            //
            // reading intrinsic parameters
            //
            std::cout << "reading camera intrinsics..." << std::endl;
            cv::FileStorage fs("intrinsics.yaml", cv::FileStorage::READ);
            if(!fs.isOpened()) {
                ROS_INFO_STREAM("camera not calibrated");
            }
            else {
                fs["camera_matrix"] >> camera_matrix;
                fs["dist_coeffs"] >> dist_coeffs;
                fs.release();
                cv::Mat inlier_mask;
                std::cout << "calculating essential matrix..." << std::endl;
                cv::Mat essential_mat = cv::findEssentialMat(points1,
                                                             points2,
                                                             camera_matrix,
                                                             cv::RANSAC,
                                                             .99,
                                                             1.0,
                                                             inlier_mask);    
                //
                // determine the realtive rotation matrix, R, and relative 
                // translation matrix, T between the 2 camera positions.
                //
                // this function allows us to retrieve the triangulated object points.
                //
                cv::Mat rotation_mat;
                cv::Mat translation_mat;
                double distance_threshold = 10.0;
                //
                // define a matrix to hold triangulated 3D object points
                //
                cv::Mat object_points;
                //
                // several versions of this function exist. we will use the 
                // version that triangulates the object points
                //
                int num_good_inliers = cv::recoverPose(essential_mat,
                                                       points1,
                                                       points2,
                                                       camera_matrix,
                                                       rotation_mat,
                                                       translation_mat,
                                                       distance_threshold,
                                                       cv::noArray(),
                                                       object_points);
                if (num_good_inliers < 1) {
                    std::cout << "pose recovery failed..." << std::endl;
                }
                //
                // refine the camera pose for each image
                //
                // we can call one of the solvePNPRefine functions:
                // solvePnPRefineLM uses the Levenberg-Marquard iterative algorithm.
                // solvePnPRefineVVS() uses a virtual visual servoing scheme.
                //
                // pass in the object points from pose recovery.
                //
                cv::Mat rot_mat1(rotation_mat);
                cv::Mat trans_mat1(translation_mat);
                epsilon = FLT_EPSILON;
                term_crit = cv::TermCriteria(
                            cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 
                            max_count, 
                            epsilon);
                cv::solvePnPRefineLM(object_points,
                                     points1,
                                     camera_matrix,
                                     dist_coeffs,
                                     rot_mat1,
                                     trans_mat1,
                                     term_crit
                                     );
                //
                // repeat for the second image
                //
                cv::Mat rot_mat2(rot_mat1);
                cv::Mat trans_mat2(trans_mat1);
                cv::solvePnPRefineLM(object_points,
                                     points1,
                                     camera_matrix,
                                     dist_coeffs,
                                     rot_mat1,
                                     trans_mat1,
                                     term_crit
                                     );
                //
                // extract camera motion from translation matrices
                // 
                // subtract trans_mat1 from trans_mat2, result is incremental
                // camera motion
                //
                cv::Mat delta_trans = trans_mat2 - trans_mat1;
                //
                // parse specific elements of transalation if needed
                // to determine motion along x or y axes
                //
                double x_trans = delta_trans.at<double>(0);
                double y_trans = delta_trans.at<double>(1);
            }
            // end 
        }
        else {
            camera_control.set_control_value(idle);
        }
        ros::spinOnce();
        //
        // sleep or block or go inactive until the next loop iteration
        //
        loop_rate.sleep();
        //
    }// ---- End Loop

    cv::destroyAllWindows();
}
