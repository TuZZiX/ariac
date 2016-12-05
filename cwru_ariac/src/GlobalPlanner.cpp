//
// Created by tianshipei on 12/3/16.
//

#include "GlobalPlanner.h"

GlobalPlanner::GlobalPlanner(ros::NodeHandle nodeHandle) {
    agvs[0].basePose.pose.position.x = 0.12;
    agvs[0].basePose.pose.position.y = 3.46;
    agvs[0].basePose.pose.position.z = 0.75;
}