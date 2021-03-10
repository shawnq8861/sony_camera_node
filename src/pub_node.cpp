#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char **argv)
{
    //
    // initialize ROS and create unique node name
    //
    ros::init(argc, argv, "pub_node");
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
                "pub_hello", queue_size);
    //
    // set the loop rate used by spin to control while loop execution
    // this is an integer that equates to loops/second
    //
    ros::Rate loop_rate = 2;
    //
    // create a variable to hold the loop count
    //
    int count = 0;
    //
    // publish the messages in a while loop as long as our node is
    // in good standing as a ROS node
    //
    while (ros::ok()) {
        //
        // create a msg object to hole the string
        //
        std_msgs::String msg;
        std::stringstream msg_ss;
        ++count;
        msg_ss << "loop count: " << count;
        msg.data = msg_ss.str();
        //
        // publish the message to the topic
        //
        pub.publish(msg);
        //
        // direct to message strin to stdout
        //
        ROS_INFO("%s", msg.data.c_str());
        //
        // spinOnce() allows access to callback functions if we have any
        //
        ros::spinOnce();
        //
        // sleep or block or go inactive until the next loop iteration
        //
        loop_rate.sleep();
    }
}
