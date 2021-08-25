#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <std_msg/Float64.h>

// global joint pub variables
ros::Publisher joint1_pub, joint2_pub;

// main method
int main(int argc, char** argv){

    //initialize the arm_mover node 
    ros::init(argc, argv, "arm_mover");

    ros::NodeHandle n;

    //create two publisher for the joint1 and joint2
    joint1_pub = n.advertise<std_msg::FLoat64>("simple_arm/joint_1_position_controller/command", 10);
    joint2_pub = n.advertise<std_msg::Float64>("simple_arm/joint_2_position_controller/command", 10);

    // create safe_move service with is callback function
    ros::Service service = n.advertiseService("/arm_mover/safe_move", safe_move_request);
    ROS_INFO("Ready to send joint commands");

    // handle ROS communications events
    ros::spin();

    return 0;

}