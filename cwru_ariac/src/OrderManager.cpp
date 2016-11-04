//
// Created by shipei on 11/1/16.
//

#include "OrderManager.h"

OrderManager::OrderManager(ros::NodeHandle nodeHandle): nh_(nodeHandle){
    orderSubscriber = nh_.subscribe(
            "/ariac/orders", 10,
            &OrderManager::orderCallback, this);
    scoreSubscriber = nh_.subscribe(
            "/ariac/current_score", 10,
            &OrderManager::scoreCallback, this);
    competitionStateSubscriber = nh_.subscribe(
            "/ariac/competition_state", 10,
            &OrderManager::competitionStateCallback, this);
    AGV1Client =
            nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
    AGV2Client =
            nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");
    if (!AGV1Client.exists()) {
        AGV1Client.waitForExistence();
    }
    if (!AGV2Client.exists()) {
        AGV2Client.waitForExistence();
    }
}

void OrderManager::orderCallback(const osrf_gear::Goal::ConstPtr &goal_msg) {
    bool found = false;
    for (int i = 0; i < orders.size(); ++i) {
        if (orders[i].goal_id.data == goal_msg->goal_id.data && orders[i].kits[0].kit_type.data == goal_msg->kits[0].kit_type.data) {
            found = true;
            break;
        }
    }
    if (!found) {
        orders.push_back(*goal_msg);
    }
}

void OrderManager::scoreCallback(const std_msgs::Float32::ConstPtr &msg) {
    if (msg->data != score)
    {
        ROS_INFO("Score: %f", msg->data);
    }
    score = msg->data;
}

void OrderManager::competitionStateCallback(const std_msgs::String::ConstPtr &msg) {
    if (msg->data == "done" && competitionState != "done")
    {
        ROS_INFO("Competition ended.");
    }
    competitionState = msg->data;
}

bool OrderManager::startCompetition() {
    // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
    ros::ServiceClient serviceClient =
            nh_.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    // If it's not already ready, wait for it to be ready.
    // Calling the Service using the client before the server is ready would fail.
    if (!serviceClient.exists()) {
        serviceClient.waitForExistence();
    }
    std_srvs::Trigger srv;  // Combination of the "request" and the "response".
    serviceClient.call(srv);  // Call the start Service.
    return !srv.response.success;
}

bool OrderManager::submitOrder(int AGV, osrf_gear::Kit kit) {
    osrf_gear::AGVControl srv;
    srv.request.kit_type = kit.kit_type;
    switch (AGV) {
        case 1:
            AGV1Client.call(srv);  // Call the start Service.
            return !srv.response.success;
        case 2:
            AGV2Client.call(srv);  // Call the start Service.
            return !srv.response.success;
        default:
            break;
    }
    return false;
}

double OrderManager::scoreFunction(double TT) {
    double TC = 1;
    double AC = 1;
    //double TT;
    double AT = 1;
    double CS = getCurrentScore();
    double CF = AC/TC;
    double EF = AT/TT;
    return CF*CS+EF*CS;
}