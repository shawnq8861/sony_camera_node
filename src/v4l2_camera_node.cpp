#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sony_camera_node/CameraCommand.h>
#include <sstream>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include "sony_camera_interface_header.hpp"

int printCapabilities(int fd)
{
    int retVal = 0;
    struct v4l2_capability caps = {};
    //
    // query camera driver settings
    //
    retVal = ioctl(fd, VIDIOC_QUERYCAP, &caps);
    if (retVal < 0) {
        perror("Querying Capabilities");
        return 1;
    }
    ROS_INFO_STREAM("Camera Driver Capabilities:\n"
         << "  Driver: " << caps.driver << std::endl
         << "  Card: " << caps.card << std::endl
         << "  Bus: " << caps.bus_info << std::endl
         << "  Version: " << ((caps.version>>16)&&0xff) << "."
         << ((caps.version>>24)&&0xff) << std::endl
         << std::showbase << std::internal << std::setfill('0')
         << "  Capabilities: " << std::hex << caps.capabilities
         << std::dec);
    //
    // get the current camera settings
    //
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    retVal = ioctl(fd, VIDIOC_G_FMT, &fmt);
    if (retVal < 0) {
        perror("Getting Pixel Format");
        return 1;
    }
    char pixelFormat[5] = {0};
    strncpy(pixelFormat, (char *)&fmt.fmt.pix.pixelformat, sizeof(fmt.fmt.pix.pixelformat));
    ROS_INFO_STREAM("Camera Parameters:" << std::endl
         << "  Width: " << fmt.fmt.pix.width << std::endl
         << "  Height: " << fmt.fmt.pix.height << std::endl
         << "  PixFmt: " << pixelFormat << std::endl
         << "  Field: " << fmt.fmt.pix.field);

        return 0;
}

int initMmap(int fd, struct v4l2_buffer& buf, uint8_t*& buffer)
{
    int retVal = 0;
    struct v4l2_requestbuffers req = {0};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    //
    // request that a buffer be allocated internally on the camera
    //
    ROS_INFO_STREAM("requesting buffers...");
    retVal = ioctl(fd, VIDIOC_REQBUFS, &req);
    if (retVal < 0) {
        perror("Requesting Buffer");
        return 1;
    }
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    //
    // query status of buffer to verify buffer before calling mmap
    //
    ROS_INFO_STREAM("query buffer...");
    retVal = ioctl(fd, VIDIOC_QUERYBUF, &buf);
    if (retVal < 0) {
        perror("Querying Buffer");
        return 1;
    }
    //
    // map internal camera buffer to process memory space
    //
    ROS_INFO_STREAM("mapping memory...");
    buffer = (uint8_t *)mmap (NULL, buf.length, PROT_READ | PROT_WRITE,
                              MAP_SHARED, fd, buf.m.offset);
    if ( buffer == MAP_FAILED) {
        ROS_INFO_STREAM("mmap failed...");
    }
    ROS_INFO_STREAM("Mapped buffer Length: " << buf.length);

    return 0;
}

int captureImage(int fd, struct v4l2_buffer& buf, uint8_t*& buffer)
{
    int retVal = 0;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    //
    // start streaming images to camera buffer
    //
    retVal = ioctl(fd, VIDIOC_STREAMON, &buf.type);
    if (retVal < 0) {
        perror("Stream On");
        return 1;
    }
    //
    //  queue the buffer in camera memory
    //
    retVal = ioctl(fd, VIDIOC_QBUF, &buf);
    if (retVal < 0) {
        perror("Query buffer");
        return 1;
    }
    //
    // check that the camera file descriptor is "ready"
    // before dequeing the buffer
    //
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    struct timeval tv = {0};
    tv.tv_sec = 2;
    int r = select(fd+1, &fds, NULL, NULL, &tv);
    if(-1 == r) {
        perror("Waiting for frame");
        return 1;
    }
    //
    // dequeue the buffer from the camera memory
    //
    retVal = ioctl(fd, VIDIOC_DQBUF, &buf);
    if (retVal < 0) {
        perror("Retrieving frame");
        return 1;
    }

    return 0;
}

int get_image_size(int fd, unsigned int& height, unsigned int& width)
{
    //
    // get the current image parameters
    //
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int retVal = ioctl(fd, VIDIOC_G_FMT, &fmt);
    if (retVal < 0) {
        perror("Getting Pixel Format");
        return EXIT_FAILURE;
    }
    height = fmt.fmt.pix.height;
    width = fmt.fmt.pix.width;
    
    return EXIT_SUCCESS;
}

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
    // calling advertise instantiates a Publiser object
    //
    ros::Publisher pub = nh.advertise<std_msgs::String>(
                "uvc_camera_hello", queue_size);
    //
    // instantiate a publisher for camera images
    //
    //ros::Publisher uvc_image_pub =
    //        nh.advertise<sensor_msgs::Image>("uvc_camera/image", 10);
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
    ros::Rate loop_rate = .50;
    //
    // create a variable to hold the loop count
    //
    int loop_count = 0;
    int final_count = loop_count;
    //
    // publish the messages in a while loop as long as our node is
    // in good standing as a ROS node
    //
    // create a msg object to hold the string
    //
    std_msgs::String msg;
    std::stringstream msg_ss;
    //
    // variables needed to interface to uvc camera
    //  
    uint8_t *buffer;
    struct v4l2_buffer buf = {0};
    int fd;
    ROS_INFO_STREAM("opening file descriptor...");    
    fd = open("/dev/video0", O_RDWR);
    if (fd == -1) {
        perror("Opening video device");
        return EXIT_FAILURE;
    }
    ROS_INFO_STREAM("calling printCaps...");
    if(printCapabilities(fd)) {
        return EXIT_FAILURE;
    }
    ROS_INFO_STREAM("calling initMmap...");
    if(initMmap(fd, buf, buffer)) {
        return EXIT_FAILURE;
    }
    unsigned int image_height;
    unsigned int image_width;
    if (get_image_size(fd, image_height, image_width)) {
        return EXIT_FAILURE;
    }
    ROS_INFO_STREAM("image height = " << image_height 
                    << ", image_width = " << image_width);
    //
    // create vector to hold image data
    //
    std::vector<unsigned char> image_data(buf.length);
    //
    // main ROS loop
    //
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
        // declare an image message object to hold the data
        //
        sensor_msgs::Image image;
        //
        // acquire the live image and assign it to the image block
        //
        ROS_INFO_STREAM("calling capture image...");
        if(captureImage(fd, buf, buffer)) {
            return 1;
        }
        memcpy(image_data.data(), buffer, buf.length);
        //
        // configure the image message
        //
        image.header.stamp = ros::Time::now();
        image.data = image_data;
        image.height = image_height;
        image.width = image_width;
        image.step = buf.length/image_height;
        image.encoding = sensor_msgs::image_encodings::RGB16;
        //
        // publish the image to an image_view data type
        //
        uvc_image_pub.publish(image);

        ROS_INFO_STREAM("command value = " << (int)camera_control.get_control_value());

        if (0 == camera_control.get_control_value()) {
            //
            // close fd and prepare for shutdown
            //
            loop_count = -2;
        }
        else if (1 == camera_control.get_control_value()) {
            //
            // save image to file
            //
        }
        else if (2 == camera_control.get_control_value()) {
            //
            // ???
            //
            ROS_INFO_STREAM("writing " << buf.length << " bytes of data to file");
            FILE *pFile = NULL;
            pFile = fopen ("video0Data.jpg", "wb");
            fwrite (buffer, 1, buf.length, pFile);
            fclose (pFile);
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
    //
    // release and delete held memory and devices
    //
    // un-map memory
    //
    ROS_INFO_STREAM("um-mapping memory...");
    int retVal = 0;
    retVal = munmap(buffer, buf.length);
    if (retVal < 0) {
        perror("munmap");
    }
    //
    // close file descriptor
    //
    close(fd);

    std::exit(EXIT_SUCCESS);
}