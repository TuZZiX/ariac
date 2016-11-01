//
// Created by shipei on 11/1/16.
//

#ifndef CWRU_ARIAC_ORDERMANAGER_H
#define CWRU_ARIAC_ORDERMANAGER_H

#include <AriacBase.h>

class OrderManager:AriacBase {
public:
    OrderManager(ros::NodeHandle nodeHandle);

    vector<osrf_gear::Goal> orders;

    void startCompetition();
    void submitOrder(osrf_gear::Goal order);
    bool isCompetitionEnd();
    double getCurrentScore();

    double scoreFunction();

private:
    ros::Time startTime;
    
};


#endif //CWRU_ARIAC_ORDERMANAGER_H
