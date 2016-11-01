//
// Created by shipei on 11/1/16.
//

#include "OrderManager.h"

OrderManager::OrderManager(ros::NodeHandle nodeHandle) {

}

void OrderManager::startCompetition() {

}

bool OrderManager::isCompetitionEnd() {

}

double OrderManager::getCurrentScore() {

}

void OrderManager::submitOrder(osrf_gear::Goal order) {

}

double OrderManager::scoreFunction() {
    double TC = 1;
    double AC = 1;
    double TT;
    double AT = 1;
    double CS = getCurrentScore();
    double CF = AC/TC;
    double EF = AT/TT;
    return CF*CS+EF*CS;
}