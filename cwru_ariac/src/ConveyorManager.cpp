//
// Created by shipei on 11/1/16.
//

#include "ConveyorManager.h"

ConveyorManager::ConveyorManager(ros::NodeHandle nodeHandle, CameraEstimator &estimator, RobotPlanner &planner): nh_(nodeHandle), estimator_(&estimator), planner_(&planner) {
    extendSearchRange = 1.0;
    approachTimes = 5;
    approachAheadTime = 0.5;
}

Part ConveyorManager::getClosestPart() {
    int bestApproachIndex = 0;
    double bestDistance = numeric_limits<double>::infinity();
    double tempDistance;

    for (int i = 0; i < estimator_->onConveyor.size(); ++i) {
        tempDistance = euclidianDistance(planner_->getCurrentGripperPose().position, estimator_->onConveyor[i].pose.pose.position);
        if (tempDistance < bestDistance) {
            bestApproachIndex = i;
            bestDistance = tempDistance;
        }
    }
    return estimator_->onConveyor[bestApproachIndex];

}

Part ConveyorManager::getClosestPartBest() {
    int bestApproachIndex = 0;
    double bestTime = numeric_limits<double>::infinity();
    double tempTime;

    for (int i = 0; i < estimator_->onConveyor.size(); ++i) {
        if(!planner_->plan(estimator_->onConveyor[i].pose.pose, tempTime))
            continue;
        if (tempTime < bestTime) {
            bestApproachIndex = i;
            bestTime = tempTime;
        }
    }
    return estimator_->onConveyor[bestApproachIndex];
}

double ConveyorManager::estimatePickTime(Part part) {
    double planningTime = 0.0;
    if (planner_->plan(part.pose.pose, planningTime)) {
        return planningTime;
    }
    return numeric_limits<double>::infinity();
}

bool ConveyorManager::estimateMovingPart(Part part, double &executingTime, geometry_msgs::PoseStamped &estimatedPose) {
    Part estimatedPart = part;
    double dplan = 0;
    double dt = 0;
    double tempExeTime = 0;
    estimatedPose = part.pose;
    estimatedPose.header.stamp = ros::Time::now();

    // planning time for the first time
    dplan += planner_->planningTime;
    part.pose.pose.position.x += part.linear.x * dt;
    part.pose.pose.position.y += part.linear.y * dt;
    part.pose.pose.position.z += part.linear.z * dt;

    for (int i = 0; i < approachTimes; ++i) {
        if (planner_->plan(estimatedPose.pose, tempExeTime))
            return false;
        // calculate time cost for next plan
        dplan += planner_->planningTime;
        dt = tempExeTime + dplan;
        estimatedPart.pose.pose.position.x = part.pose.pose.position.x + part.linear.x * dt;
        estimatedPart.pose.pose.position.y = part.pose.pose.position.y + part.linear.y * dt;
        estimatedPart.pose.pose.position.z = part.pose.pose.position.z + part.linear.z * dt;
    }
    bool reachable = false;
    while (!reachable) {
        estimatedPart.pose.pose.position.x += part.linear.x * approachAheadTime;
        estimatedPart.pose.pose.position.y += part.linear.y * approachAheadTime;
        estimatedPart.pose.pose.position.z += part.linear.z * approachAheadTime;
        if (planner_->plan(estimatedPose.pose, executingTime))
            return false;
        if (executingTime - tempExeTime + planner_->planningTime < approachAheadTime)
            reachable = true;
        // reach that pose at estimated time
        estimatedPose.header.stamp = ros::Time::now() + ros::Duration(approachAheadTime);
    }
    return true;
}