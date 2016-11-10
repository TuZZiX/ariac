//
// Created by shipei on 10/18/16.
//

#include "RobotPlanner.h"

RobotPlanner::RobotPlanner(ros::NodeHandle &nodeHandle): nh_( nodeHandle ){
    ros::Subscriber joint_state_subscriber = nh_.subscribe(
            "/ariac/joint_states", 10,
            &RobotPlanner::jointStateCallback, this);
    joint_trajectory_publisher = nh_.advertise<trajectory_msgs::JointTrajectory>(
            "/ariac/arm/command", 10);
    gripper = nh_.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
    called = false;
    while(!called) {
        ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.2).sleep();
    }
    if (!gripper.exists()) {
        gripper.waitForExistence();
    }
    attach.request.enable = 1;
    detach.request.enable = 0;
    planningTime = 0.5;
    arrivalTime = 0.5;
}

void RobotPlanner::jointStateCallback(const sensor_msgs::JointState::ConstPtr &jointStateMsg)
{
    current_joint_states = *jointStateMsg;
    called = true;
}

bool RobotPlanner::plan(geometry_msgs::Pose pose, double &executingTime) {

}

bool RobotPlanner::move(geometry_msgs::Pose pose, double &executingTime) {

}

bool RobotPlanner::planToHome(double &executingTime) {

}

bool RobotPlanner::executeLastPlan() {

}

void RobotPlanner::sendJointsValue(vector<double> joints) {
    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    msg.joint_names = current_joint_states.name;
    msg.points.resize(1);
    msg.points[0].positions.resize(current_joint_states.name.size(), 0.0);
    for (int i = 0; i < joints.size(); ++i) {
        msg.points[0].positions[i] = joints[i];
    }
    msg.points[0].time_from_start = ros::Duration(arrivalTime);
    //ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
    waitForFinish(arrivalTime);
}

void RobotPlanner::waitForFinish(double time) {
    ros::Duration(time).sleep();                         // wait for finish
    ros::spinOnce();
}
vector<double> RobotPlanner::getJointsState() {
//    called = false;
//    while(!called) {
//        //ROS_INFO("Waiting for joint feedback...");
//        ros::spinOnce();
//        ros::Duration(0.1).sleep();
//    }
    vector<double> joints;
    joints.resize(current_joint_states.name.size(), 0.0);
    for (int i = 0; i < joints.size(); ++i) {
        joints = current_joint_states.position;
    }
    return joints;
}

void RobotPlanner::grab() {
    gripper.call(attach);
}

void RobotPlanner::release() {
    gripper.call(detach);
}

bool RobotPlanner::getGripperState() {

}