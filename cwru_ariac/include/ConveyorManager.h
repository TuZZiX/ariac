//
// Created by shipei on 11/1/16.
//

#ifndef CWRU_ARIAC_CONVEYORMANAGER_H
#define CWRU_ARIAC_CONVEYORMANAGER_H

#include <AriacBase.h>
#include <CameraEstimator.h>
#include <RobotPlanner.h>

class ConveyorManager: public AriacBase {
public:
    ConveyorManager(ros::NodeHandle nodeHandle, CameraEstimator &estimator, RobotPlanner &planner);
    double extendSearchRange;
    double approachTimes;
    double approachAheadTime;

    Part getClosestPart();
    Parts getCloseParts();
    Part getClosestPartBest();
    Parts getClosePartsBest();
    bool estimateMovingPart(Part part, double &executingTime, geometry_msgs::PoseStamped &estimatedPose);

private:
    ros::NodeHandle nh_;
    CameraEstimator *estimator_;
    RobotPlanner *planner_;
};


#endif //CWRU_ARIAC_CONVEYORMANAGER_H
