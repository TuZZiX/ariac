//
// Created by shipei on 10/19/16.
//

#include "CameraEstimator.h"

CameraEstimator::CameraEstimator(ros::NodeHandle nodeHandle, string cameraTopic) : nh_(nodeHandle) {
    cameraSubscriber = nh_.subscribe(cameraTopic, 10,
                                     &CameraEstimator::cameraCallback, this);
    distanceTolerance = 0.01;
    untraceableTolerance = 0.1;
    assigndID = 1;
    called = false;

    worldFrame = "/world";
    cameraFrame = "/logical_camera_1_frame";
}

void CameraEstimator::cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &image_msg) {
    geometry_msgs::PoseStamped inPose;
    geometry_msgs::PoseStamped outPose;
    Part nextPart;
    Parts updateView;
    ros::Time currentTime = ros::Time::now();
    double dt = currentTime .toSec() - lastTime.toSec();

    // update all exist objects
    for (int k = 0; k < inView.size(); ++k) {
        inView[k].pose.pose.position.x += inView[k].linear.x * dt;
        inView[k].pose.pose.position.y += inView[k].linear.y * dt;
        inView[k].pose.pose.position.z += inView[k].linear.z * dt;
        inView[k].pose.header.stamp = ros::Time::now();
    }
    // TODO refactor to: match all exist object first, then all untraceable object then add new object
    // TODO change inView type to hash map
    for (int i = 0; i < image_msg->models.size(); ++i) {
        bool flag = false;
        nextPart.type = defaultParts.find(image_msg->models[i].type)->second;
        inPose.pose = image_msg->models[i].pose;
        inPose.header.frame_id = cameraFrame;
        inPose.header.stamp = ros::Time::now();
        bool tferr = true;
        while (tferr) {
            tferr = false;
            try {
                tf_listener.transformPose(worldFrame, inPose, outPose);
            } catch (tf::TransformException &exception) {
                ROS_ERROR("%s", exception.what());
                tferr = true;
                ros::Duration(0.05).sleep(); // sleep for half a second
                ros::spinOnce();
            }
        }
        nextPart.pose = outPose;
        double distance;
        // find object by estimated distance;
        for (int j = 0; j < inView.size(); ++j) {
            distance = euclidianDistance(nextPart.pose.pose.position, inView[j].pose.pose.position);
            if ((distance < distanceTolerance) && (nextPart.type.name == inView[j].type.name)) {
                nextPart.linear.x = (nextPart.pose.pose.position.x - inView[j].pose.pose.position.x)/dt;
                nextPart.linear.y = (nextPart.pose.pose.position.y - inView[j].pose.pose.position.y)/dt;
                nextPart.linear.z = (nextPart.pose.pose.position.z - inView[j].pose.pose.position.z)/dt;
                nextPart.traceable = true;
                nextPart.id = inView[i].id;
                inView.erase(inView.begin()+j);
                flag = true;
                break;
            }
        }
        // try to find untraceable object by nearest distance
        if (!flag) {
            for (int j = 0; j < inView.size(); ++j) {
                distance = euclidianDistance(nextPart.pose.pose.position, inView[j].pose.pose.position);
                if ((!inView[j].traceable) && (distance < untraceableTolerance) && (nextPart.type.name == inView[j].type.name)) {
                    if ((distance < untraceableTolerance) && (nextPart.type.name == inView[j].type.name)) {
                        nextPart.linear.x = (nextPart.pose.pose.position.x - inView[j].pose.pose.position.x)/dt;
                        nextPart.linear.y = (nextPart.pose.pose.position.y - inView[j].pose.pose.position.y)/dt;
                        nextPart.linear.z = (nextPart.pose.pose.position.z - inView[j].pose.pose.position.z)/dt;
                        nextPart.traceable = true;
                        nextPart.id = inView[i].id;
                        inView.erase(inView.begin()+j);
                        flag = true;
                        break;
                    }
                }
            }
        }
        // make new object untraceable
        if (!flag) {
            nextPart.linear.x = 0;
            nextPart.linear.y = 0;
            nextPart.linear.z = 0;
            nextPart.traceable = false;
            nextPart.id = assigndID++;
        }
        updateView.push_back(nextPart);
    }
    inView.swap(updateView);
    splitLocation();
    called = true;
}
void CameraEstimator::splitLocation() {
    onConveyor.clear();
    for (int j = 0; j < totalAGVs; ++j) {
        onAGV[j].clear();
    }
    for (int k = 0; k < totalBins; ++k) {
        onBin[k].clear();
    }
    onGround.clear();
    for (int i = 0; i < inView.size(); ++i) {
        if (ConveyorBoundBoxXmin<=inView[i].pose.pose.position.x && inView[i].pose.pose.position.x<=ConveyorBoundBoxXmax || ConveyorBoundBoxYmin <= inView[i].pose.pose.position.y && inView[i].pose.pose.position.y <= ConveyorBoundBoxYmax) {
            if (!inView[i].traceable){
                inView[i].traceable = true;
                inView[i].linear.x = 0;
                inView[i].linear.y = 0;
                inView[i].linear.z = 0;
            }
            onConveyor.push_back(inView[i]);
            continue;
        }
        bool jump = false;
        for (int j = 0; j < totalAGVs; ++j) {
            if (AGVBoundBoxXmin[j]<=inView[i].pose.pose.position.x && inView[i].pose.pose.position.x<=AGVBoundBoxXmax[j] || AGVBoundBoxYmin[j] <= inView[i].pose.pose.position.y && inView[i].pose.pose.position.y <= AGVBoundBoxYmax[j]) {
                onAGV[j].push_back(inView[i]);
                jump = true;
                break;
            }
        }
        if (jump)
            continue;
        for (int j = 0; j < totalBins; ++j) {
            if (BinBoundBoxXmin[j]<=inView[i].pose.pose.position.x && inView[i].pose.pose.position.x<=BinBoundBoxXmax[j] || BinBoundBoxYmin[j] <= inView[i].pose.pose.position.y && inView[i].pose.pose.position.y <= BinBoundBoxYmax[j]) {
                onBin[j].push_back(inView[i]);
                jump = true;
                break;
            }
        }
        if (jump)
            continue;
    }
}
