//
// Created by shipei on 10/19/16.
//

#include "CameraEstimator.h"

CameraEstimator::CameraEstimator(ros::NodeHandle nodeHandle) : nh_(nodeHandle) {
    cameraSubscriber = nh_.subscribe("/ariac/logical_camera_1", 10,
                                     &CameraEstimator::cameraCallback, this);
    distanceTolerance = 0.01;
    untraceableTolerance = 0.1;
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
    }
    bool tferr = true;
    while (tferr) {
        tferr = false;
        try {
            for (int i = 0; i < image_msg->models.size(); ++i) {
                bool flag = false;
                nextPart.type = defaultParts.find(image_msg->models[i].type)->second;
                inPose.pose = image_msg->models[i].pose;
                inPose.header.frame_id = "/logical_camera_1_frame";
                inPose.header.stamp = ros::Time::now();
                tf_listener.transformPose("/world", inPose, outPose);
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
                        inView.erase(inView.begin()+j);
                        break;
                        flag = true;
                    }
                }
                // try to find untraceable object by nearest distance
                if (!flag) {
                    double lastDistance;
                    for (int j = 0; j < inView.size(); ++j) {
                        distance = euclidianDistance(nextPart.pose.pose.position, inView[j].pose.pose.position);
                        if ((!inView[j].traceable) && (distance < untraceableTolerance) && (nextPart.type.name == inView[j].type.name)) {
                            if (distance < lastDistance) {
                                nextPart.linear.x = (nextPart.pose.pose.position.x - inView[j].pose.pose.position.x)/dt;
                                nextPart.linear.y = (nextPart.pose.pose.position.y - inView[j].pose.pose.position.y)/dt;
                                nextPart.linear.z = (nextPart.pose.pose.position.z - inView[j].pose.pose.position.z)/dt;
                                nextPart.traceable = true;
                                lastDistance = distance;
                            }
                        }
                    }
                }
                updateView.push_back(nextPart);
            }
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.1).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }

}