#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

//
// create a callback function to process the messages
// received on the subcribed topic
//
void subscriberCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("subscriber received:  [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    //
    // initialize ROS and create unique node name
    //
    ros::init(argc, argv, "sub_node");
    //
    // declare a node handle
    //
    ros::NodeHandle nh;
    //
    // set the maximum number of messages storable in the queue buffer
    //
    int queue_size = 1000;
    //
    // subscribe to the publisher topic and pass to the callback
    //
    ros::Subscriber sub = nh.subscribe("pub_hello", queue_size,
                                       &subscriberCallback);
    //
    // ros::spin() calls all callbacks that have data available and are in
    // the ready queue
    //
    ros::spin();
}
