//
// Created by tianshipei on 12/4/16.
//

#ifndef CWRU_ARIAC_ORACLEPLANNER_H
#define CWRU_ARIAC_ORACLEPLANNER_H

#include <AriacBase.h>
#include <cwru_ariac/OracleQuery.h>

class OraclePlanner {
public:
    OraclePlanner(ros::NodeHandle& nodeHandle);
    bool pick(Part part, bool move = false);
    bool place(Part destination, bool move = false);
    bool move(Part part, Part destination, bool move = false);
    bool setMaxPlanningTime(double maxPlanningTime);
    double getMaxPlanningTime() {return maxPlanningTime;}

private:
    ros::NodeHandle nh_;
    ros::ServiceClient oracle;
    double maxPlanningTime;
    OracleQueryRequest request;
    OracleQueryResponse response;
};
#endif //CWRU_ARIAC_ORACLEPLANNER_H
