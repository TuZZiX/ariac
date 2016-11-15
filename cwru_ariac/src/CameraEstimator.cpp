//
// Created by shipei on 10/19/16.
//

#include "CameraEstimator.h"

CameraEstimator::CameraEstimator(ros::NodeHandle nodeHandle, string cameraTopic) : nh_(nodeHandle) {
    cameraSubscriber = nh_.subscribe(cameraTopic, 1,
                                     &CameraEstimator::cameraCallback, this);
    distanceTolerance = 0.02;
    untraceableTolerance = 0.1;
    assigndID = 1;
    updateCount = 0;
    checkedCount = 0;
    worldFrame = "/world";
    cameraFrame = "/logical_camera_1_frame";
    //tf_listener.waitForTransform(cameraFrame, worldFrame, ros::Time(0), ros::Duration(2.0));
}

void CameraEstimator::cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &image_msg) {
    Parts updateView;
    ros::Time currentTime = ros::Time::now();
    double dt = currentTime.toSec() - lastTime.toSec();
    // update all exist objects
    for (int k = 0; k < inView.size(); ++k) {
        inView[k].pose.pose.position.x += inView[k].linear.x * dt;
        inView[k].pose.pose.position.y += inView[k].linear.y * dt;
        inView[k].pose.pose.position.z += inView[k].linear.z * dt;
        inView[k].pose.header.stamp = ros::Time::now();
    }
    // TODO refactor to: match all exist object first, then all untraceable object then add new object
    // TODO change inView type to hash map
    //ROS_INFO("inView: %d, size: %d, dt = %f",(int)inView.size(), (int)image_msg->models.size(), (float)dt);
    for (int i = 0; i < image_msg->models.size(); ++i) {
        geometry_msgs::PoseStamped inPose;
        geometry_msgs::PoseStamped outPose;
        inPose.header.frame_id = cameraFrame;
        bool flag = false;
        bool tferr = true;
        double distance;
        Part nextPart;
        nextPart.type = defaultParts.find(image_msg->models[i].type)->second;
        inPose.pose = image_msg->models[i].pose;
        while (tferr && ros::ok()) {
            tferr = false;
            try {
                inPose.header.stamp = ros::Time::now();
                tf_listener.transformPose(worldFrame, inPose, outPose);
            } catch (tf::TransformException &exception) {
                return;
//                ROS_ERROR("%s", exception.what());
//                tferr = true;
//                ros::Duration(0.05).sleep();
//                ros::spinOnce();
            }
        }
        nextPart.pose = outPose;

        // find object by estimated distance;
        for (int j = 0; j < inView.size(); ++j) {
            distance = euclideanDistance(nextPart.pose.pose.position, inView[j].pose.pose.position);
            if ((distance < distanceTolerance) && (nextPart.type.name == inView[j].type.name)) {
                nextPart.linear.x = (nextPart.linear.x + (nextPart.pose.pose.position.x - inView[j].pose.pose.position.x)/dt)/2;
                nextPart.linear.y = (nextPart.linear.y + (nextPart.pose.pose.position.y - inView[j].pose.pose.position.y)/dt)/2;
                nextPart.linear.z = (nextPart.linear.z + (nextPart.pose.pose.position.z - inView[j].pose.pose.position.z)/dt)/2;
                nextPart.traceable = true;
                nextPart.id = inView[j].id;
                inView.erase(inView.begin()+j);
                flag = true;
                break;
            }
        }
        // try to find untraceable object by nearest distance
        if (!flag) {
            for (int j = 0; j < inView.size(); ++j) {
                distance = euclideanDistance(nextPart.pose.pose.position, inView[j].pose.pose.position);
                if ((!inView[j].traceable) && (distance < untraceableTolerance) && (nextPart.type.name == inView[j].type.name)) {
                    if ((distance < untraceableTolerance) && (nextPart.type.name == inView[j].type.name)) {
                        nextPart.linear.x = (nextPart.pose.pose.position.x - inView[j].pose.pose.position.x)/dt;
                        nextPart.linear.y = (nextPart.pose.pose.position.y - inView[j].pose.pose.position.y)/dt;
                        nextPart.linear.z = (nextPart.pose.pose.position.z - inView[j].pose.pose.position.z)/dt;
                        nextPart.traceable = true;
                        nextPart.id = inView[j].id;
                        inView.erase(inView.begin()+j);
                        flag = true;
                        break;
                    }
                }
            }
        }
        // make new object untraceable
        if (!flag) {
            ROS_INFO("add part %d : %s", assigndID, nextPart.type.name.c_str());
            nextPart.linear.x = 0;
            nextPart.linear.y = 0;
            nextPart.linear.z = 0;
            nextPart.traceable = false;
            nextPart.id = assigndID++;
        }
        updateView.push_back(nextPart);
    }
//    for (int k = 0; k < inView.size(); ++k) {
//        ROS_INFO("part %d:%s disappear", inView[k].id, inView[k].type.name.c_str());
//    }
    for (auto part: inView) {
        ROS_INFO("part %d : %s disappeared", part.id, part.type.name.c_str());
    }
    inView.swap(updateView);
    splitLocation();
    lastTime = currentTime;
    updateCount++;
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
        if ((ConveyorBoundBoxXmin<=inView[i].pose.pose.position.x && inView[i].pose.pose.position.x<=ConveyorBoundBoxXmax) && (ConveyorBoundBoxYmin <= inView[i].pose.pose.position.y && inView[i].pose.pose.position.y <= ConveyorBoundBoxYmax)) {
//            if (!inView[i].traceable){
//                inView[i].traceable = true;
//                inView[i].linear.x = 0;
//                inView[i].linear.y = -0.05;
//                inView[i].linear.z = 0;
//            }
            onConveyor.push_back(inView[i]);
            continue;
        }
        bool jump = false;
        for (int j = 0; j < totalAGVs; ++j) {
            if (AGVBoundBoxXmin[j]<=inView[i].pose.pose.position.x && inView[i].pose.pose.position.x<=AGVBoundBoxXmax[j] && AGVBoundBoxYmin[j] <= inView[i].pose.pose.position.y && inView[i].pose.pose.position.y <= AGVBoundBoxYmax[j]) {
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

void CameraEstimator::waitForUpdate() {
    while (updateCount == checkedCount && ros::ok()){
        //ROS_INFO("Waiting for camera callback...");
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    checkedCount = updateCount;
}