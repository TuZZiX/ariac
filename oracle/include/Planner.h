//
// Created by shipei on 10/18/16.
//

#ifndef CWRU_ARIAC_ROBOTPLANNER_H
#define CWRU_ARIAC_ROBOTPLANNER_H

#include <AriacBase.h>
#include <cwru_ariac/OracleQuery.h>

class Planner {
public:
    Planner(ros::NodeHandle& nodeHandle);
    bool pick(Part part, double &planningTime = fakeDouble, double &executingTime = fakeDouble, int &errorCode = fakeInt, int &planID = fakeInt);
    bool place(Part destination, double &planningTime = fakeDouble, double &executingTime = fakeDouble, int &errorCode = fakeInt, int &planID = fakeInt);
    bool move(Part part, Part destination, double &planningTime = fakeDouble, double &executingTime = fakeDouble, int &errorCode = fakeInt, int &planID = fakeInt);
    bool estimateMovingPart(Part part, geometry_msgs::PoseStamped &estimatedPose);
    vector<double> getJointsState();

    void setMaxPlanningTime(double maxPlanningTime) {this->maxPlanningTime = maxPlanningTime;}
    double getMaxPlanningTime() { return maxPlanningTime;}

private:
    ros::NodeHandle nh_;

    vector<trajectory_msgs::JointTrajectory> plans;
    int assignedID;
    double maxPlanningTime;     // deadline for planning, must return before this time
    ros::Subscriber joint_state_subscriber;
    sensor_msgs::JointState current_joint_states;
    bool called;
    double arrivalTime;
    double approachTimes;
    double approachAheadTime;

    static double fakeDouble;   // declared for default parameter, please ignore
    static int fakeInt;         // declared for default parameter, please ignore

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state_msg);
};


#endif //CWRU_ARIAC_ROBOTPLANNER_H
