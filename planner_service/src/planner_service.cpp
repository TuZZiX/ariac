//
// Created by tianshipei on 11/29/16.
//

#include <RobotPlanner.h>
#include <cwru_ariac/RobotPlanQuery.h>

class PlannerService {
public:
    PlannerService(ros::NodeHandle nodeHandle): nh(nodeHandle), planner(nh) {
        service = nh.advertiseService("/cwru_ariac/planner_service", &PlannerService::callback, this);
    }
private:
    ros::ServiceServer service;
    ros::NodeHandle nh;
    RobotPlanner planner;

    bool callback(cwru_ariac::RobotPlanQueryRequest& request, cwru_ariac::RobotPlanQueryResponse& response)
    {
        ROS_INFO("callback activated");
        if (request.type == request.none) {
            response.success = (unsigned char) true;
        } else if (request.type == request.planToHome) {
            response.success = (unsigned char) planner.planToHome();
            response.planningTime = planner.getLastPlanningTime();
            response.executingTime = planner.getLastExecutingTime();
        } else if (request.type == request.planPose) {
            response.success = (unsigned char) planner.planPose(request.source);
            response.planningTime = planner.getLastPlanningTime();
            response.executingTime = planner.getLastExecutingTime();
        } else if (request.type == request.planPart) {
            response.success = (unsigned char) planner.planPart(request.sourcePart);
            response.planningTime = planner.getLastPlanningTime();
            response.executingTime = planner.getLastExecutingTime();
        } else if (request.type == request.pick) {
            response.success = (unsigned char) planner.pick(request.sourcePart);
            response.planningTime = planner.getLastPlanningTime();
            response.executingTime = planner.getLastExecutingTime();
        } else if (request.type == request.place) {
            response.success = (unsigned char) planner.place(request.targetPart);
            response.planningTime = planner.getLastPlanningTime();
            response.executingTime = planner.getLastExecutingTime();
        } else if (request.type == request.move) {
            response.success = (unsigned char) planner.move(request.sourcePart, request.targetPart);
            response.planningTime = planner.getLastPlanningTime();
            response.executingTime = planner.getLastExecutingTime();
        } else if (request.type == request.sendJointsValue) {
            planner.sendJointsValue(request.joints);
            response.planningTime = planner.getLastPlanningTime();
            response.executingTime = planner.getLastExecutingTime();
            response.success = (unsigned char) true;
        } else if (request.type == request.getJointsState) {
            response.jointsState = planner.getJointsState();
            response.success = (unsigned char) true;
        } else if (request.type == request.grab) {
            planner.grab();
            response.success = (unsigned char) true;
        } else if (request.type == request.release) {
            planner.release();
            response.success = (unsigned char) true;
        } else if (request.type == request.getGripperState) {
            response.gripperState = planner.getGripperState();
            response.success = (unsigned char) true;
        } else if (request.type == request.isGripperAttached) {
            response.gripperAttached = (unsigned char) planner.isGripperAttached();
            response.success = (unsigned char) true;
        } else if (request.type == request.waitForGripperAttach) {
            response.success = (unsigned char) planner.waitForGripperAttach(request.timeout);
        } else if (request.type == request.setMaxPlanningTime) {
            planner.setMaxPlanningTime(request.maxPlanningTime);
            response.success = (unsigned char) true;
        } else if (request.type == request.getCurrentBasePose) {
            response.basePose.pose = planner.getCurrentBasePose();
            response.basePose.header.stamp = ros::Time::now();
            response.success = (unsigned char) true;
        } else if (request.type == request.getCurrentGripperPose) {
            response.gripperPose.pose= planner.getCurrentGripperPose();
            response.gripperPose.header.stamp = ros::Time::now();
            response.success = (unsigned char) true;
        } else if (request.type == request.executeLastPlan) {
            response.success = (unsigned char) planner.executeLastPlan();
            response.success = (unsigned char) true;
        } else if (request.type == request.moveClosestParts) {
            // TODO
            response.success = (unsigned char) true;
        }
        // TODO
        if (response.success) {
            response.ERROR_CODE = response.NO_ERROR;
        } else {
            response.ERROR_CODE =response.UNKNOWN_ERROR;
        }
        return true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_service");
    PlannerService plannerService(ros::NodeHandle nh);
    ROS_INFO("Start planner service.");
    ros::spin();
    return 0;
}