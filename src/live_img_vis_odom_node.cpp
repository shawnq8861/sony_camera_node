#include <ros/ros.h>
#include <std_msgs/String.h>
#include "sony_camera_interface_header.hpp"
#include <opencv4/opencv2/video/tracking.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/core/matx.hpp>
#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <sstream>

static constexpr int max_points = 100;

int main(int argc, char **argv)
{
    //
    // initialize ROS and create unique node name
    //
    ros::init(argc, argv, "live_img_vis_odom_node");
    //
    // declare a node handle
    //
    ros::NodeHandle nh;
    //
    // set the maximum number of messages storable in the queue buffer
    //
    int queue_size = 1000;
    //
    // instantiate a CameraControl object to enable
    // access to service callback and to encapsulate
    // the request value
    //
    CameraControl camera_control;
    ROS_INFO_STREAM("Camera control object, command value = " << camera_control.get_control_value());
    //
    // instantiate a service to be called to receive commands
    //
    ros::ServiceServer camera_command_service = nh.advertiseService(
                "sony_camera_node/camera_command",
                &CameraControl::callback,
                &camera_control);
    //
    // set the loop rate used by spin to control while loop execution
    // this is an integer that equates to loops/second
    //
    ros::Rate loop_rate = 2.0;
    //
    // set up video capture
    //
    cv::VideoCapture cap;
    int deviceID = 2;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    //
    // open selected camera using selected API
    //
    cap.open(deviceID, apiID);
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    //
    // instantiate a Mat object to hold the image frame data
    //
    cv::Mat frame;
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
    while (ros::ok() && loop_count > -1) {
        ++loop_count;
        final_count = loop_count;
        //
        // acquire the live image and assign it to the image block
        //
        cap >> frame;
        //
        // Check if grabbed frame is actually full with some content
        //
        if(!frame.empty()) {
            cv::waitKey(1);
            if (show) {
                cv::namedWindow("view", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
                int width = 800;
                int height = 600;
                cv::resizeWindow("view", width, height);
                cv::imshow("view", frame);
                cv::waitKey(1);
                /* code */
            }
            else {
                cv::destroyAllWindows();
            }
        }
        ROS_INFO_STREAM("test command value = " << (int)camera_control.get_control_value());
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
            // visual odometry by pose extraction from feature correspondence
            //
            // first, run optical flow with ORB detector
            //
            // Steps
            // 1) capture image 1
            // 2) convert image 1 to grayscale
            // 3) find ORB features to track
            // 4) refine to subpixel corners
            // 5) capture image 2
            // 6) convert to grayscale
            // 7) calculate pyramid LK optical flow
            // 8) when overlap > threshold
            //      a) track
            //      b) recover pose
            //      c) save object points
            //      d) refine poses
            //      e) calc translation
            //      f) set image 2 to image 1
            //      g) repeat from 5)
            //           
            //
            // read image 1, convert to grayscale
            //
            cv::Mat image1 = frame;
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
            std::vector<cv::Point2f> points2;
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
            //
            // capture image 2, convert to grayscale
            //

            //
            // need a loop here
            // loop until camera moves enough to track motion
            //

            double percent_overlap = 0.0;
            while (percent_overlap < 5.0) {
                cap >> frame;
                cv::Mat image2 = frame;
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
                double img_width = (double)image2.cols;
                percent_overlap = 100.0 * (img_width - delta_x_ave)/img_width;
            }
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
    ROS_INFO_STREAM("exited ROS loop, loop count = " << final_count);
    cv::destroyAllWindows();

    std::exit(EXIT_SUCCESS);
}