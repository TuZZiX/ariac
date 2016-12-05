//
// Created by tianshipei on 11/29/16.
//

#include "OraclePlanner.h"

OraclePlanner::OraclePlanner(ros::NodeHandle &nodeHandle): nh_(nodeHandle) {
    oracle = nh_.serviceClient<cwru_ariac::OracleQuery>("/cwru_ariac/oracle");
}

bool OraclePlanner::pick(Part part, bool move) {

}

bool OraclePlanner::place(Part destination, bool move) {

}

bool OraclePlanner::move(Part part, Part destination, bool move) {

}

bool OraclePlanner::setMaxPlanningTime(double maxPlanningTime) {
    this->maxPlanningTime = maxPlanningTime;

}