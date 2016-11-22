//
// Created by shipei on 11/1/16.
//

#include "BinManager.h"

BinManage::BinManage(ros::NodeHandle nodeHandle) {
    bins = vector<Bin>(totalBins, defaultBin);
    bins[0].name = "Bin4";
    bins[0].priority = 5;
    bins[0].pose.pose.position.x = -1.000000;
    bins[0].pose.pose.position.y = 0.995000;
    bins[0].pose.pose.position.z = 0.0;

    bins[1].name = "Bin4";
    bins[1].priority = 5;
    bins[1].pose.pose.position.x = -1.000000;
    bins[1].pose.pose.position.y = 0.995000;
    bins[1].pose.pose.position.z = 0.0;

    bins[2].name = "Bin4";
    bins[2].priority = 5;
    bins[2].pose.pose.position.x = -1.000000;
    bins[2].pose.pose.position.y = 0.995000;
    bins[2].pose.pose.position.z = 0.0;

    bins[3].name = "Bin4";
    bins[3].priority = 5;
    bins[3].pose.pose.position.x = -1.000000;
    bins[3].pose.pose.position.y = 0.995000;
    bins[3].pose.pose.position.z = 0.0;

    for (auto bin: bins) {
        priorityList.push_back(bin.priority);
    }
    sort(priorityList.begin(), priorityList.end());
}

bool BinManage::initBin(vector<Parts> &onBin) {

}

bool BinManage::findPutLocation(PartType part, geometry_msgs::PoseStamped &best) {

}

bool BinManage::findPutLocationList(PartType part, vector<geometry_msgs::PoseStamped> &list) {

}

bool BinManage::findRemoveLocation(PartType part, geometry_msgs::PoseStamped &best) {

}

bool BinManage::findRemoveLocationList(PartType part, vector<geometry_msgs::PoseStamped> &list) {

}

bool BinManage::put(Part part) {

}

bool BinManage::remove(Part part) {

}