#ifndef CAMERA_HEADER_HPP
#define CAMERA_HEADER_HPP
//
// sony camera interface header
//
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sony_camera_node/CameraCommand.h>

static const int idle = 99;

class CameraControl
{
public:
    CameraControl()
    {control_value = idle;};
    ~CameraControl(){};
    int get_control_value()
    {
        return control_value;
    }
    void set_control_value(int value)
    {
        control_value = idle;
    }
    bool callback(sony_camera_node::CameraCommandRequest& request, 
                  sony_camera_node::CameraCommandResponse& response)
    {
        if (request.command > -1 && request.command <= idle) {
            control_value = request.command;
            ROS_INFO_STREAM("service called, command value = " << (int)request.command);
            return true;
        }
        else {
            return false;
        }
    }
private:
    int control_value;
};

//
// create an image class to encapsulate the image data
//
class ImageControl
{
public:
    ImageControl(){};
    ~ImageControl(){};
    void image_data_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            data = cv::Mat(cv_ptr->image);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
        
    }
    cv::Mat get_image_data()
    {
        return data;
    }
private:
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat data;
};

#endif
