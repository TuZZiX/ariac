//
// Created by shipei on 11/1/16.
//

#ifndef CWRU_ARIAC_BINMANAGER_H
#define CWRU_ARIAC_BINMANAGER_H

#include <AriacBase.h>
#include <CameraEstimator.h>


class BinManage: public AriacBase {
public:
    BinManage(ros::NodeHandle nodeHandle);
    bool initBin(vector<Parts>& onBin);
    bool findPutLocation(PartType part, geometry_msgs::PoseStamped& best);
    bool findPutLocationList(PartType part, vector<geometry_msgs::PoseStamped>& list);
    bool findRemoveLocation(PartType part, geometry_msgs::PoseStamped& best);
    bool findRemoveLocationList(PartType part, vector<geometry_msgs::PoseStamped>& list);
    bool put(Part part);
    bool remove(Part part);

    vector<Bin> bins;
private:
    ros::NodeHandle nh_;

    vector<int> priorityList;
};


#endif //CWRU_ARIAC_BINMANAGER_H
