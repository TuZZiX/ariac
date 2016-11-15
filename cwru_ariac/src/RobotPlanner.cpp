//
// Created by shipei on 10/18/16.
//

#include "RobotPlanner.h"

RobotPlanner::RobotPlanner(ros::NodeHandle &nodeHandle): nh_( nodeHandle ){
    joint_state_subscriber = nh_.subscribe(
            "/ariac/joint_states", 10,
            &RobotPlanner::jointStateCallback, this);
    joint_trajectory_publisher = nh_.advertise<trajectory_msgs::JointTrajectory>(
            "/ariac/arm/command", 10);
    gripper = nh_.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
    gripperStateSubscriber = nh_.subscribe("/ariac/gripper/state", 10, &RobotPlanner::gripperStateCallback, this);
    called = false;
    attached = false;
    while(!called && ros::ok()) {
        //ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.05).sleep();
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
void RobotPlanner::gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg) {
    currentGripperState = *msg;
    attached = msg->attached;
}
osrf_gear::VacuumGripperState RobotPlanner::getGripperState() {
    return currentGripperState;
}
bool RobotPlanner::isGripperAttached() {
    return attached;
}
void RobotPlanner::waitForGripperAttach(double timeout) {
    ros::spinOnce();
    timeout = timeout <= 0? FLT_MAX:timeout;
    while((!attached) && timeout > 0 && ros::ok()) {
        //ROS_INFO("Retry grasp");
        release();
        ros::Duration(0.2).sleep();
        grab();
        ros::Duration(0.8).sleep();
        ros::spinOnce();
        timeout -= 0.4;
    }
}
bool RobotPlanner::planPose(geometry_msgs::Pose pose, double &executingTime) {
    executingTime = rand()%6;
    return true;
}
bool RobotPlanner::planPart(Part part, double &executingTime) {
    executingTime = rand()%6;
    return true;
}
bool RobotPlanner::move(geometry_msgs::Pose pose, double &executingTime) {
    executingTime = rand()%6;
    return true;
}

bool RobotPlanner::planToHome(double &executingTime) {
    executingTime = rand()%6;
    return true;
}

bool RobotPlanner::executeLastPlan() {
    return true;
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
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
    waitForFinish(arrivalTime);
}

void RobotPlanner::waitForFinish(double time) {
    ros::Duration(time).sleep();                         // wait for finish
    ros::spinOnce();
}
vector<double> RobotPlanner::getJointsState() {
    called = false;
    while(!called && ros::ok()) {
        //ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    vector<double> joints;
    joints.resize(current_joint_states.name.size(), 0.0);
    for (int i = 0; i < joints.size(); ++i) {
        joints = current_joint_states.position;
    }
    return joints;
}

void RobotPlanner::grab() {
    //ROS_INFO("enable gripper");
    gripper.call(attach);
}

void RobotPlanner::release() {
    //ROS_INFO("release gripper");
    gripper.call(detach);
}