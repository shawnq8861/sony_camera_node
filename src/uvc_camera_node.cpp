#include <ros/ros.h>
#include <std_msgs/String.h>
#include "sony_camera_interface_header.hpp"
#include <iostream>
#include <sstream>

int main(int argc, char **argv)
{
    //
    // initialize ROS and create unique node name
    //
    ros::init(argc, argv, "uvc_camera_node");
    //
    // declare a node handle
    //
    ros::NodeHandle nh;
    //
    // set the maximum number of messages storable in the queue buffer
    //
    int queue_size = 1000;
    //
    // calling advertise instantiates a Publisher object
    //
    ros::Publisher pub = nh.advertise<std_msgs::String>(
                "uvc_camera_hello", queue_size);
    //
    // instantiate a publisher for camera images
    //
    image_transport::ImageTransport it(nh);
    image_transport::Publisher uvc_image_pub = it.advertise("uvc_camera/image", 1);
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
                "sony_camera_node/camera_command",
                &CameraControl::callback,
                &camera_control);
    //
    // set the loop rate used by spin to control while loop execution
    // this is an integer that equates to loops/second
    //
    ros::Rate loop_rate = 10;
    //
    // publish the messages in a while loop as long as our node is
    // in good standing as a ROS node
    //
    // create a msg object to hold the string
    //
    std_msgs::String msg;
    std::stringstream msg_ss;
    //
    // set up video capture
    //
    cv::VideoCapture cap;
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_GSTREAMER;      // 0 = autodetect default API
    //
    // open selected camera using selected API
    //
    cap.open(deviceID, apiID);
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    else {
        double api = cap.get(cv::CAP_PROP_BACKEND);
        ROS_INFO_STREAM("backend = " << api);
        double frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        ROS_INFO_STREAM("frame width = " << frame_width);
        double frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        ROS_INFO_STREAM("frame height = " << frame_height);
        double frame_rate = cap.get(cv::CAP_PROP_FPS);
        ROS_INFO_STREAM("frame rate = " << frame_rate);
        double fps = frame_rate * 5.0;
        cap.set(cv::CAP_PROP_FPS, fps);
        frame_rate = cap.get(cv::CAP_PROP_FPS);
        ROS_INFO_STREAM("frame rate = " << frame_rate);
        frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        ROS_INFO_STREAM("frame width = " << frame_width);
        frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        ROS_INFO_STREAM("frame height = " << frame_height);
        cv::Mat frame;
        cap >> frame;
        ROS_INFO_STREAM("frame cols = " << frame.cols);
        ROS_INFO_STREAM("frame rows = " << frame.rows);
    }
    //
    // instantiate a Mat object to hold the image frame data
    //
    cv::Mat frame;
    sensor_msgs::ImagePtr img_msg;
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
        msg_ss << "loop count: " << loop_count;
        msg.data = msg_ss.str();
        //
        // publish the message to the topic
        //
        pub.publish(msg);
        //
        // acquire the live image and assign it to the image block
        //
        cap >> frame;
        //
        // Check if grabbed frame is actually full with some content
        //
        if(!frame.empty()) {
            img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            uvc_image_pub.publish(img_msg);
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
        if (0 == camera_control.get_control_value()) {
            //
            // prepare for shutdown
            //
            loop_count = -2;
        }
        else if (1 == camera_control.get_control_value()) {
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
        else if (2 == camera_control.get_control_value()) {
            //
            // save image to file, default compression
            //
            std::string homedir = getenv("HOME");
            std::stringstream path_ss;
            path_ss << homedir;
            path_ss << "/catkin_ws/saved_image_";
            path_ss << image_count;
            path_ss << ".jpg";
            std::string path;
            path_ss >> path;
            ROS_INFO_STREAM(path);
            cv::imwrite(path, frame);
            ++image_count;
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