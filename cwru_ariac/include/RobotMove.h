//
// Created by tianshipei on 12/4/16.
//

#ifndef CWRU_ARIAC_ROBOTMOVE_H
#define CWRU_ARIAC_ROBOTMOVE_H


#include <AriacBase.h>

class RobotMove {
public:
    RobotMove(ros::NodeHandle& nodeHandle);
    bool planToHome();
    bool pick(Part part, bool move = false);
    bool place(Part destination, bool move = false);
    bool move(Part part, Part destination, bool move = false);
    void sendJointsValue(vector<double> joints);
    vector<double> getJointsState();
    void grab();
    void release();
    bool isGripperAttached();
    bool waitForGripperAttach(double timeout);
    void setMaxPlanningTime(double maxPlanningTime) {this->maxPlanningTime = maxPlanningTime;}
    bool getRobotState(RobotState robotState);

private:
    ros::NodeHandle nh_;
    ros::ServiceClient oracle;
    ros::ServiceClient robot;
    double maxPlanningTime;
    OracleQueryRequest request;
    OracleQueryResponse response;

    geometry_msgs::Pose homePose;
    geometry_msgs::Pose currentBasePose;
    geometry_msgs::Pose currentGripperPose;
};


#endif //CWRU_ARIAC_ROBOTMOVE_H
