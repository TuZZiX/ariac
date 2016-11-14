//
// Created by shipei on 11/1/16.
//

#include <ConveyorManager.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_tunner");
    ros::NodeHandle nh;
    CameraEstimator camera(nh);
    RobotPlanner robot(nh);
    ConveyorManager conveyor(nh, camera, robot);
}
