#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <std_msg/Float64.h>

// global joint pub variables
ros::Publisher joint1_pub, joint2_pub;

//function to check joint angles in a safe position
std::vector<float> clamp_at_boundaries(float requested_j1, float requested_j2){

    // definedclamped joint angles and assign them to the requested
    float clamped_j1 = requested_j1;
    float clamped_j2 = requested_j2;

    //define min and max joint parameters
    float min_j1, min_j2, max_j1, max_j2;

    //create new node handle as we don't have access to the main one
    ros::NodeHandle n2;

    //get node name
    std::string node_name = ros::this_node::getName();

    //get min and max joints angles parameters from master
    
    n2.getParam(node_name + "/min_joint_1_angles",min_j1);
    n2.getParam(node_name + "/min_joint_2_angles",min_j2);
    n2.getParam(node_name + "/max_joint_1_angles",max_j1); 
    n2.getParam(node_name + "/max_joint_2_angles",max_j2);

    //checks if joint 1 is inside the boundaries, in other case clamp it
    if (requested_j1 < min_j1 || requested_j1 > max_j1){
        clamped_j1 = std::min(std::max(requested_j1, min_j1),max_j1);
        ROSWARN("j1 angle is out of boundaries, valid range is (%1.2f,%1.2f) clamped at %1.2f",min_j1,max_j1,clamped_j1);
    }

    //checks if joint 2 is inside the boundaries, in other case clamp it
    if (requested_j2 < min_j2 || requested_j2 > max_j2){
        clamped_j2 = std::min(std::max(requested_j2, min_j2),max_j2);
        ROSWARN("j2 angle is out of boundaries, valid range is (%1.2f,%1.2f) clamped at %1.2f",min_j2,max_j2,clamped_j2);
    }

    //store clamped joint angles
    std::vector<float> clamped_data = {clamped_j1, clamped_j2};

    return clamped_data;
}

// callback function for whenever the safe move service is called
bool safe_move_request(simple_arm::GoToposition::Request& req, simple_arm::GoToPosition::Response& res){

    ROS_INFO("GoToPosition Request recieved - j1:%1.2f, j2:%1.2f", (float)req.joint_1, (float)req.joint_2);

    std::vector<float> joints_angles = clamp_at_boundaries(req.joint1, req.joint_2); // call clamp_at_boundaries

    //define joint angle variables
    std_msg::Float64 joint1_angle, joint2_angle;

    joint1_angle.data = joints_angles[0];
    joint2_angle.data = joints_angles[1];

    joint1_pub.publish(joint1_angle);
    joint2_pub.publish(joint2_angle);

    // wait 3 secs for the arm to settle
    ros::Duration(3).sleep();


    //return a response message
    res.msg_feedback = "Joint angles set - j1: " + std::to_string(joint1_angle) + ", j2: " + std::to_string(joint2_angle);

    ROS_INFO_STREAM(res.msg_feedback);

    return true;

}


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