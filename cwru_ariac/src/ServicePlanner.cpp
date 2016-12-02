//
// Created by tianshipei on 11/29/16.
//

#include <AriacBase.h>
#include <cwru_ariac/OracleQuery.h>
#include <cwru_ariac/RobotMoveAction.h>

class Oracle {
public:
    Oracle(ros::NodeHandle& nodeHandle): nh_(nodeHandle) {
        oracle = nh_.serviceClient<cwru_ariac::OracleQuery>("/cwru_ariac/oracle");
        robot = nh_.serviceClient<cwru_ariac::RobotMoveAction>("/cwru_ariac/robot_move");
    }
    bool planToHome() {
        OracleQueryRequest request;
    }
    bool pick(Part part, bool move = false) {

    }
    bool place(Part destination, bool move = false) {

    }
    bool move(Part part, Part destination, bool move = false) {

    }

    void sendJointsValue(vector<double> joints) {

    }
    vector<double> getJointsState() {

    }
    void grab() {

    }
    void release() {

    }
    bool isGripperAttached() {

    }
    bool waitForGripperAttach(double timeout) {

    }

    void setMaxPlanningTime(double maxPlanningTime) {this->maxPlanningTime = maxPlanningTime;}
    geometry_msgs::Pose getCurrentBasePose() { return currentBasePose;}
    geometry_msgs::Pose getCurrentGripperPose() { return currentGripperPose;}

private:
    ros::NodeHandle nh_;
    ros::ServiceClient oracle;
    ros::ServiceClient robot;
    double maxPlanningTime;
    OracleQueryRequest request;
    OracleQueryResponse response;

    geometry_msgs::Pose currentBasePose;
    geometry_msgs::Pose currentGripperPose;
};