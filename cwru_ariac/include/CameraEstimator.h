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
    Parts onGround;
    Parts onConveyor;
    Parts onAGV[totalAGVs];
    Parts onBin[totalBins];
    string worldFrame;
    string cameraFrame;
    CameraEstimator(ros::NodeHandle nodeHandle, string cameraTopic = "/ariac/logical_camera_1");
    void waitForUpdate();

private:
    ros::NodeHandle nh_;
    ros::Subscriber cameraSubscriber;
    void cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void splitLocation();
    tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    ros::Time lastTime;
    int assigndID;
    bool called;
};


#endif //CWRU_ARIAC_CAMERAESTIMATOR_H
