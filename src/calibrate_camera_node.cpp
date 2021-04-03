#include <ros/ros.h>
#include "sony_camera_interface_header.hpp"
#include <opencv4/opencv2/video/tracking.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/core/matx.hpp>
#include <opencv4/opencv2/core/persistence.hpp>
#include <iostream>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <vector>

static constexpr int max_points = 100;
static constexpr int max_images = 12;
static constexpr int col_count = 9;
static constexpr int row_count = 6;
static constexpr double square_size = 31.75;  // size of chess board square

int main(int argc, char **argv)
{
    //
    // initialize ROS and create unique node name
    //
    ros::init(argc, argv, "calibrate_camera_node");
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
    std::string homedir = getenv("HOME");
    std::string base_file_name = "calibration_image_";
    std::string corner_file_name = "calibration_corner_image_";
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
            // save images to file, default compression
            //
            std::stringstream path_ss;
            path_ss << homedir;
            path_ss << base_path;
            path_ss << base_file_name;
            path_ss << image_count;
            path_ss << ".jpg";
            std::string path;
            path_ss >> path;
            cv::imwrite(path, data);
            ++image_count;
            if (image_count > max_images) [
                image_count = 0;
            ]
        }
        else if (34 == camera_control.get_control_value()) {
            //
            // calibrate camera
            //
            // initialize the 3D object corner values
            //
            std::vector<Point3f> object_corners;
            std::vector<Point2f> image_corners;
            double X = 0.0f;    // horizontal world coordinate
            double Y = 0.0f;    // vertical world coordinate
            for (int row = 0; row < row_count; ++row) {
                for (int col = 0; col < col_count; ++col) {
                    object_corners.push_back(Point3f(X, Y, 0.0f));
                    std::cout << "X = " << X << ", Y = " << Y << std::endl;
                    Y += square_size;
                }
                Y = 0.0f;
                X += square_size;
            }
            std::cout << "starting calibration loop" << std::endl;
            vector< vector<Point3f> > object_points;
            vector< vector<Point2f> > image_points;
            for (int index = 0; index < max_images; ++index) {
                //
                // build up jpeg image file name and read data from file
                //

                std::stringstream path_ss;
                path_ss << homedir;
                path_ss << base_path;
                path_ss << base_file_name;
                path_ss << index + 1;
                path_ss << ".jpg";
                std::string path;
                path_ss >> path;

                
                std::cout << "reading file: " << path << std::endl;

                cv::Mat corner_img = imread(path, cv::IMREAD_COLOR);
                cv::cvtColor(corner_img, corner_img, cv::COLOR_BGR2GRAY);
                if (corner_img.empty()) {
                    std::cout << "could not read image..." << std::endl;
                }
                else {
                    cv::Size checker_size(col_count, row_count);
                    bool corners_found = cv::findChessboardCorners(corner_img,
                                                              checker_size,
                                                              image_corners,
                                                              cv::CALIB_CB_ADAPTIVE_THRESH
                                                              + cv::CALIB_CB_NORMALIZE_IMAGE);
                    if (corners_found) {
                        cv::cornerSubPix(corner_img, 
                                        image_corners, 
                                        cv::Size(11, 11), 
                                        cv::Size(-1, -1), 
                                        cv::TermCriteria(cv::CV_TERMCRIT_EPS + cv::CV_TERMCRIT_ITER, 30, 0.1)
                                        );
                        cv::cvtColor(corner_img, corner_img, cv::COLOR_GRAY2BGR);
                        drawChessboardCorners(corner_img,
                                              checker_size,
                                              cv::Mat(image_corners),
                                              corners_found);

                        string cornerWinName("Corner Image ");
                        cornerWinName.append(number);

                        std::stringstream path_ss;
                        path_ss << homedir;
                        path_ss << base_path;
                        path_ss << corner_file_name;
                        path_ss << index + 1;
                        path_ss << ".jpg";
                        std::string path;
                        path_ss >> path;
                        cv::imwrite(path, corner_img);

                        //
                        // save the image and object corners
                        //
                        object_points.push_back(object_corners);
                        image_points.push_back(image_corners);
                    }
                    else {
                        std::cout << "could not find corners for image " << number << std::endl;
                    }
                }
            }
            //
            // calculate intrinsic matrix values and radial distortion coefficients
            //
            cv::Mat camera_matrix(3, 3, CV_64F);
            cv::Mat dist_coeffs(5, 1, CV_64F);
            std::vector<cv::Mat> rVecs;
            std::vector<cv::Mat> tVecs;
            double reproj_err = calibrateCamera (objectPoints,
                                                imagePoints,
                                                inputImage.size(),
                                                cameraMatrix,
                                                distCoeffs,
                                                rVecs,
                                                tVecs);
            std::cout << "reprojection error = " << reproj_err << std::endl;
            std::cout << "instrinsic matrix values:" << std::endl << std::endl;
            std::cout << "fx = " << cameraMatrix.at<double>(0, 0) << std::endl;
            std::cout << cameraMatrix.at<double>(0, 1) << std::endl;
            std::cout << "u0 = " << cameraMatrix.at<double>(0, 2) << std::endl;
            std::cout << cameraMatrix.at<double>(1, 0) << std::endl;
            std::cout << "fy = " << cameraMatrix.at<double>(1, 1) << std::endl;
            std::cout << "v0 = " << cameraMatrix.at<double>(1, 2) << std::endl;
            std::cout << cameraMatrix.at<double>(2, 0) << std::endl;
            std::cout << cameraMatrix.at<double>(2, 1) << std::endl;
            std::cout << cameraMatrix.at<double>(2, 2) << std::endl;
            std::cout << "distortion matrix values:" << std::endl << std::endl;
            std::cout << "k1 = " << distCoeffs.at<double>(0) << std::endl;
            std::cout << "k2 = "<< distCoeffs.at<double>(1) << std::endl;
            std::cout << "p1 = "<< distCoeffs.at<double>(2) << std::endl;
            std::cout << "p2 = "<< distCoeffs.at<double>(3) << std::endl;
            std::cout << "k3 = "<< distCoeffs.at<double>(4) << std::endl;
            //
            // save intrinsic parameters to yaml file
            //
            cv::FileStorage fs("camera_intrinsics.yaml", cv::FileStorage::WRITE);
            if (fs.isOpened()) {
                fs << "camera_matrix" << camera_matrix << "dist_coeffs" << dist_coeffs;
                fs.release();
            }
            else {
                cout << "Error: can not save the intrinsic parameters\n";
            }
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
