#include "ros/ros.h"
#include <sony_camera_node/CameraCommand.h>
#include <cstdlib>
#include "sony_camera_interface_header.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_interface");
    if (argc != 2) {
        ROS_INFO_STREAM("usage: send commands to camera");
        return EXIT_FAILURE;
    }
    ros::NodeHandle n;
    ros::ServiceClient camera_command_client = 
        n.serviceClient<sony_camera_node::CameraCommand>("sony_camera_node/camera_command");
    ros::ServiceClient optical_flow_command_client = 
        n.serviceClient<sony_camera_node::CameraCommand>("optical_flow_node/camera_command");
    sony_camera_node::CameraCommand camera_command_server;
    camera_command_server.request.command = atoll(argv[1]);
    if (20 > camera_command_server.request.command) {
        if (camera_command_client.call(camera_command_server)) {
            ROS_INFO_STREAM("Test called service sony_camera_node/camera_command");
            camera_command_server.request.command = (long long)idle;
            if (camera_command_client.call(camera_command_server)) {
                ROS_INFO_STREAM("Test called service sony_camera_node/camera_command");
            }
            else {
                ROS_ERROR_STREAM("Failed to call service sony_camera_node/camera_command");
                return EXIT_FAILURE;
            }
        }
        else {
            ROS_ERROR_STREAM("Failed to call service sony_camera_node/camera_command");
            return EXIT_FAILURE;
        }
    }
    else {
        if (optical_flow_command_client.call(camera_command_server)) {
            ROS_INFO_STREAM("Called service optical_flow_node/camera_command");
            camera_command_server.request.command = (long long)idle;
            if (optical_flow_command_client.call(camera_command_server)) {
                ROS_INFO_STREAM("Called service optical_flow_node/camera_command");
            }
            else {
                ROS_ERROR_STREAM("Failed to call service optical_flow_node/camera_command");
                return EXIT_FAILURE;
            }
        }
        else {
            ROS_ERROR_STREAM("Failed to call service optical_flow_node/camera_command");
            return EXIT_FAILURE;
        }
    }

  return EXIT_SUCCESS;
}