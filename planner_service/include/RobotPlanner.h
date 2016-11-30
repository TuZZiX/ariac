//
// Created by shipei on 10/18/16.
//

#ifndef CWRU_ARIAC_ROBOTPLANNER_H
#define CWRU_ARIAC_ROBOTPLANNER_H

#include <AriacBase.h>

class RobotPlanner {
public:
    RobotPlanner(ros::NodeHandle& nodeHandle);
    bool planToHome();
    bool planPose(geometry_msgs::PoseStamped pose);
    bool planPart(Part part);
    bool executeLastPlan();

    bool pick(Part part);
    bool place(Part destination);
    bool move(Part part, Part destination);
    bool estimateMovingPart(Part part, geometry_msgs::PoseStamped &estimatedPose);

    void sendJointsValue(vector<double> joints);
    vector<double> getJointsState();
    void grab();
    void release();
    osrf_gear::VacuumGripperState getGripperState();
    bool isGripperAttached();
    bool waitForGripperAttach(double timeout);

    void setMaxPlanningTime(double maxPlanningTime) {this->maxPlanningTime = maxPlanningTime;}
    double getMaxPlanningTime() { return maxPlanningTime;}
    geometry_msgs::Pose getCurrentBasePose() { return currentBasePose;}
    geometry_msgs::Pose getCurrentGripperPose() { return currentGripperPose;}
    double getLastPlanningTime() { return lastPlanningTime;}
    double getLastExecutingTime() { return lastExecutingTime;}

private:
    ros::NodeHandle nh_;

    trajectory_msgs::JointTrajectory lastPlan;
    double lastPlanningTime;    // must update after each planning
    double lastExecutingTime;   // must update after each planning
    double maxPlanningTime;     // deadline for planning, must return before this time
    geometry_msgs::Pose currentBasePose;
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
    double approachTimes;
    double approachAheadTime;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state_msg);
    void gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg);
};


#endif //CWRU_ARIAC_ROBOTPLANNER_H
