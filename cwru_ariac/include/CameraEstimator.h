//
// Created by shipei on 10/19/16.
//

#ifndef CWRU_ARIAC_CAMERAESTIMATOR_H
#define CWRU_ARIAC_CAMERAESTIMATOR_H

#include <AriacBase.h>

class CameraEstimator: public AriacBase {
public:
    double distanceTolerance;
    double untraceableTolerance;
    Parts inView;
    Parts onConvey;
    Parts onAGV;
    Parts onBin[totalBins];

    CameraEstimator(ros::NodeHandle nodeHandle);

    geometry_msgs::Pose estimate(Part object);

private:
    ros::NodeHandle nh_;
    ros::Subscriber cameraSubscriber;
    void cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    ros::Time lastTime;

};


#endif //CWRU_ARIAC_CAMERAESTIMATOR_H
