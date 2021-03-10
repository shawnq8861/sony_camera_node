#include <ros/ros.h>
#include "sony_camera_interface_header.hpp"
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <stdio.h>
#include <string>
#include <stdlib.h>

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
        ROS_INFO_STREAM("test command value = " << (int)camera_control.get_control_value());
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
            std::string homedir = getenv("HOME");
            std::stringstream path_ss;
            path_ss << homedir;
            path_ss << "/catkin_ws_test/optical_flow_image_";
            path_ss << image_count;
            path_ss << ".jpg";
            std::string path;
            path_ss >> path;
            ROS_INFO_STREAM(path);
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
            std::string image1_name = "optical_flow_image_1.jpg";
            std::string image2_name = "optical_flow_image_2.jpg";
            std::string gray1_name = "gray_image_1.jpg";
            std::string gray2_name = "gray_image_2.jpg";
            std::string tracked_name = "tracked_image.jpg";
            std::string homedir = getenv("HOME");
            std::stringstream path1_ss;
            path1_ss << homedir;
            path1_ss << "/catkin_ws/";
            path1_ss << image1_name;
            std::string path1;
            path1_ss >> path1;
            ROS_INFO_STREAM(path1);
            //
            // read image 1, convert to grayscale
            //
            cv::Mat image1 = cv::imread(path1);
            cv::Mat gray1;
            cv::cvtColor(image1, gray1, cv::COLOR_BGR2GRAY);
            //
            // call good features to track to find corners
            //
            std::vector<cv::Point2f> points1;
            const int max_corners = 500;
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
            path2_ss << "/catkin_ws/";
            path2_ss << image2_name;
            std::string path2;
            path2_ss >> path2;
            ROS_INFO_STREAM(path2);
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
            std::stringstream tracked_ss;
            tracked_ss << homedir;
            tracked_ss << "/catkin_ws/";
            tracked_ss << tracked_name;
            std::string tracked_path;
            tracked_ss >> tracked_path;
            ROS_INFO_STREAM(tracked_path);
            cv::imwrite(tracked_path, image1);
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
