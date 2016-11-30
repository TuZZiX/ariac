//
// Created by tianshipei on 11/29/16.
//

#include <AriacBase.h>
#include <cwru_ariac/RobotPlanQuery.h>

class ServicePlanner {
public:
    ServicePlanner(ros::NodeHandle& nodeHandle): nh_(nodeHandle) {
        gripper = nh_.serviceClient<cwru_ariac::RobotPlanQuery>("/cwru_ariac/planner_service");
    }
    bool planToHome() {
        RobotPlanQueryRequest request;
    }
    bool planPose(geometry_msgs::PoseStamped pose);
    bool planPart(Part part);
    bool executeLastPlan();

    bool pick(Part part);
    bool place(geometry_msgs::PoseStamped destination);
    bool move(Part part, geometry_msgs::PoseStamped destination);
    bool estimateMovingPart(Part part, geometry_msgs::PoseStamped &estimatedPose);

    void sendJointsValue(vector<double> joints);
    vector<double> getJointsState();
    void grab();
    void release();
    osrf_gear::VacuumGripperState getGripperState();
    bool isGripperAttached();
    bool waitForGripperAttach(double timeout);

    void setMaxPlanningTime(double maxPlanningTime) {this->maxPlanningTime = maxPlanningTime;}
    Eigen::Vector3d getCurrentBasePosition() { return currentBasePosition;}
    geometry_msgs::Pose getCurrentGripperPose() { return currentGripperPose;}
    double getLastPlanningTime() { return lastPlanningTime;}
    double getLastExecutingTime() { return lastExecutingTime;}

private:
    ros::NodeHandle nh_;
};