//
// Created by shipei on 10/18/16.
//

#include "RobotPlanner.h"

RobotPlanner::RobotPlanner(ros::NodeHandle &nodeHandle): nh_( nodeHandle ){
    planningTime = 0.5;
}

bool RobotPlanner::plan(geometry_msgs::Pose pose, double &executingTime) {

}

bool RobotPlanner::move(geometry_msgs::Pose pose, double &executingTime) {

}

bool RobotPlanner::planToHome(double &executingTime) {

}

bool RobotPlanner::executeLastPlan() {

}

void RobotPlanner::grab() {

}

void RobotPlanner::release() {

}