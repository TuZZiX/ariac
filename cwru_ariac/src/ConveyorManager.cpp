//
// Created by shipei on 11/1/16.
//

#include "ConveyorManager.h"

ConveyorManager::ConveyorManager(ros::NodeHandle nodeHandle, CameraEstimator &estimator, RobotPlanner &planner): nh_(nodeHandle), estimator_(&estimator), planner_(&planner) {
    extendSearchRange = 1.0;
}

Part ConveyorManager::getClosestPart() {
    int bestApproachIndex = 0;
    double bestDistance = numeric_limits<double>::infinity();
    double tempDistance;

    for (int i = 0; i < estimator_->onConveyor.size(); ++i) {
        tempDistance = euclideanDistance(planner_->getCurrentGripperPose().position, estimator_->onConveyor[i].pose.pose.position);
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

    for (int i = 0; i < estimator_->onConveyor.size(); ++i) {
        if(planner_->planPart(estimator_->onConveyor[i])) {
            if (planner_->getLastExecutingTime() < bestTime) {
                bestApproachIndex = i;
                bestTime = planner_->getLastExecutingTime();
            }
        }
    }
    return estimator_->onConveyor[bestApproachIndex];
}