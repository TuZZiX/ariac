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
    void grab();
    void release();

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

};


#endif //CWRU_ARIAC_ROBOTPLANNER_H
