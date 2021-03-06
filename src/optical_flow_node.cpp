#include <ros/ros.h>
#include "sony_camera_interface_header.hpp"
#include <opencv4/opencv2/video/tracking.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/core/matx.hpp>
#include <iostream>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <vector>

static constexpr int max_points = 100;
static constexpr double threshold = 250.0;

int main(int argc, char **argv)
{
    //
    // initialize ROS and create unique node name
    //
    ros::init(argc, argv, "optical_flow_node");
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
                "optical_flow_node/camera_command",
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
    std::string image1_name = "optical_flow_image_1.jpg";
    std::string image2_name = "optical_flow_image_2.jpg";
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
        if (20 == camera_control.get_control_value()) {
            //
            // prepare for shutdown
            //
            loop_count = -2;
        }
        else if (21 == camera_control.get_control_value()) {
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
        else if (22 == camera_control.get_control_value()) {
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
        else if (23 == camera_control.get_control_value()) {
            //
            // run optical flow with corner detector
            //
            // Steps
            // 1) read image 1 from file
            // 2) convert image 1 to grayscale
            // 3) find good features to track
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
            // call good features to track to find corners in first image
            //
            std::vector<cv::Point2f> points1;
            const int max_corners = max_points;
            const double quality_level = .03;
            const double min_distance = 10.0;
            const int block_size = 5;
            bool use_harris = true;
            double harris_free_parameter = .04;
            cv::goodFeaturesToTrack(gray1,
                                    points1,
                                    max_corners,
                                    quality_level,
                                    min_distance,
                                    cv::noArray(),
                                    block_size,
                                    use_harris,
                                    harris_free_parameter
                                    );
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
            std::vector<cv::Point2f> new_points;
            std::vector<uchar> flow_found_status;
            cv::calcOpticalFlowPyrLK(gray1,
                                    gray2,
                                    points1,
                                    new_points,
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
            //
            // only keep points when flow was found
            //
            for (int i = 0;
                i < static_cast<int>(points1.size());
                ++i) {
                if (flow_found_status[i] == 1) {
                    cv::line(image1,
                            points1[i],
                            new_points[i],
                            line_color,
                            line_thickness,
                            line_type
                            );
                }
                else {
                    continue;
                }
            }
            std::string corner_tracked_name = "corner_tracked_image.jpg";
            std::stringstream tracked_ss;
            tracked_ss << homedir;
            tracked_ss << base_path;
            tracked_ss << corner_tracked_name;
            std::string tracked_path;
            tracked_ss >> tracked_path;
            cv::imwrite(tracked_path, image1);
        }
        else if (24 == camera_control.get_control_value()) {
            //
            // run optical flow with ORB detector
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
            auto row = max_rows - max_rows;
            line_color = cv::Scalar(0, 255, 0);
            cv::Mat image3 = cv::imread(path2);
            double delta_x_sum = 0.0;
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
                            cv::line(image3,
                                points1_trim[row],
                                points2_trim[row],
                                line_color,
                                line_thickness,
                                line_type
                                );
                        }
                    }
                    else {
                        continue;
                    }
            }
            double delta_x_ave = delta_x_sum/(double)count;
            double img_width = (double)image3.cols;
            double percent_overlap = 100.0 * (img_width - delta_x_ave)/img_width;
            cv::Point top = cv::Point(delta_x_ave, 0.0);
            cv::Point bottom = cv::Point(delta_x_ave, (double)image3.rows);
            line_color = cv::Scalar(255, 0, 0);
            cv::line(image3, top, bottom, line_color, line_thickness, line_type);
            std::string orb_tracked_trim_name = "orb_tracked_image_trim.jpg";
            std::stringstream tracked_trim_ss;
            tracked_trim_ss << homedir;
            tracked_trim_ss << base_path;
            tracked_trim_ss << orb_tracked_trim_name;
            std::string tracked_trim_path;
            tracked_trim_ss >> tracked_trim_path;
            cv::imwrite(tracked_trim_path, image3);
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
