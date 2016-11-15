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
    bool updateBin(Parts parts, int binNum);
    bool getBestLocation(PartType part, geometry_msgs::Pose &bestLocation);
    bool getListLocation(PartType part, vector<geometry_msgs::Pose> &bestLocation);
private:

};


#endif //CWRU_ARIAC_BINMANAGER_H
