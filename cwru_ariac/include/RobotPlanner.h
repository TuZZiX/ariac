//
// Created by shipei on 10/18/16.
//

#ifndef CWRU_ARIAC_ROBOTPLANNER_H
#define CWRU_ARIAC_ROBOTPLANNER_H

#include <AriacBase.h>

class RobotPlanner: public AriacBase {
public:
    double planningTime;

    RobotPlanner(ros::NodeHandle& nodeHandle);
    bool planToHome(double& executingTime);
    bool plan(geometry_msgs::Pose pose, double& executingTime);
    bool planOffset(Vector3d offset, double& executingTime);
    bool executeLastPlan();
    bool move(geometry_msgs::Pose pose, double& executingTime);
    void sendJointsValue(vector<double> joints);
    void waitForFinish(double time);
    vector<double> getJointsState();
    void grab();
    void release();
    osrf_gear::VacuumGripperState getGripperState();
    bool isGripperAttached();
    void waitForGripperAttach(double timeout);

    void setMaxPlanningTime(double maxPlanningTime) {this->maxPlanningTime = maxPlanningTime;}
    double getMaxPlanningTime() { return maxPlanningTime;}
    Eigen::Vector3d getCurrentBasePosition() { return currentBasePosition;}
    geometry_msgs::Pose getCurrentGripperPose() { return currentGripperPose;}

private:
    ros::NodeHandle nh_;

    trajectory_msgs::JointTrajectory lastPlan;
    double lastPlanningTime;
    double lastExecutingTime;
    double maxPlanningTime;
    Eigen::Vector3d currentBasePosition;
    geometry_msgs::Pose currentGripperPose;

    ros::Publisher joint_trajectory_publisher;
    ros::Subscriber joint_state_subscriber;
    ros::ServiceClient gripper;
    ros::Subscriber gripperStateSubscriber;


    sensor_msgs::JointState current_joint_states;
    osrf_gear::VacuumGripperState currentGripperState;
    bool called;
    bool attached;
    osrf_gear::VacuumGripperControl attach;
    osrf_gear::VacuumGripperControl detach;
    double arrivalTime;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state_msg);
    void gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg);
};


#endif //CWRU_ARIAC_ROBOTPLANNER_H
