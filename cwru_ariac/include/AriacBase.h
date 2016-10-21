//
// Created by shipei on 10/18/16.
//

#ifndef CWRU_ARIAC_ARIACBASE_H
#define CWRU_ARIAC_ARIACBASE_H


#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <unordered_map>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <osrf_gear/Goal.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Proximity.h>

using namespace std;
using namespace Eigen;

const int gridNumber = 60;

typedef struct Grid {
    int x;
    int y;
} Grid;
typedef struct Dimension {
    double x;
    double y;
} Dimension;

typedef struct PartType {
    string name;
    Dimension size;
    Grid grid;
} PartType;

typedef struct Part {
    PartType type;
    bool traceable = false;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::Vector3 linear;
} Part;

typedef vector<Part> Parts;
typedef Eigen::Matrix<double, gridNumber, gridNumber> GridMap;

typedef struct Bin {
    string name;
    Dimension size;
    Grid grid;
    geometry_msgs::Pose pose;
    Parts container;
    int priority;
} Bin;

const int totalPartsType = 8;
const int totalBins = 8;
const double averageCost = 1000;
const string defaultPartsName[totalPartsType] = {"piston_rod_part", "gear_part", "pulley_part", "gasket_part", "part1", "part2", "part3", "part4"};
const double defaultPartsSize[totalPartsType][2] = {{0.059,0.052}, {0.078425,0.078425}, {0.23392,0.23392}, {0.31442,0.15684}, {0.3,0.1}, {0.06,0.015}, {0.13,0.07}, {0.09,0.06}};

// this class is used to storage some basic information of the competition and common algorithms.
class AriacBase {
public:
    unordered_map<string, PartType> defaultParts;
    Bin defaultBin;

    AriacBase() {
        defaultBin.name = "Bin";
        defaultBin.grid.x = 60;
        defaultBin.grid.y = 60;
        defaultBin.size.x = 0.6;
        defaultBin.size.y = 0.6;

        PartType singlePart;
        for (int i = 0; i < totalPartsType; ++i) {
            singlePart.name = defaultPartsName[i];
            singlePart.size.x = defaultPartsSize[i][0];
            singlePart.size.y = defaultPartsSize[i][1];
            singlePart.grid.x = (int)ceil(defaultBin.size.x / singlePart.size.x);
            singlePart.grid.y = (int)ceil(defaultBin.size.y / singlePart.size.y);
            defaultParts.insert(make_pair(defaultPartsName[i], singlePart));
        }
    }

    inline double euclidianDistance(geometry_msgs::Point positionA, geometry_msgs::Point positionB) {
        return sqrt(pow(positionA.x - positionB.x, 2) + pow(positionA.y - positionB.y, 2) + pow(positionA.z - positionB.z, 2));
    }

    vector<double> quat2euler(geometry_msgs::Quaternion quaternion) {
        double mData[4];
        std::vector<double> euler(3);
        const static double PI_OVER_2 = M_PI * 0.5;
        const static double EPSILON = 1e-10;
        double sqw, sqx, sqy, sqz;

        mData[0] = quaternion.x;
        mData[1] = quaternion.y;
        mData[2] = quaternion.z;
        mData[3] = quaternion.w;
        // quick conversion to Euler angles to give tilt to user
        sqw = mData[3] * mData[3];
        sqx = mData[0] * mData[0];
        sqy = mData[1] * mData[1];
        sqz = mData[2] * mData[2];

        euler[1] = asin(2.0 * (mData[3] * mData[1] - mData[0] * mData[2]));
        if (PI_OVER_2 - fabs(euler[1]) > EPSILON) {
            euler[2] = atan2(2.0 * (mData[0] * mData[1] + mData[3] * mData[2]),
                             sqx - sqy - sqz + sqw);
            euler[0] = atan2(2.0 * (mData[3] * mData[0] + mData[1] * mData[2]),
                             sqw - sqx - sqy + sqz);
        } else {
            // compute heading from local 'down' vector
            euler[2] = atan2(2 * mData[1] * mData[2] - 2 * mData[0] * mData[3],
                             2 * mData[0] * mData[2] + 2 * mData[1] * mData[3]);
            euler[0] = 0.0;

            // If facing down, reverse yaw
            if (euler[1] < 0)
                euler[2] = M_PI - euler[2];
        }
        return euler;
    }

};




#endif //CWRU_ARIAC_ARIACBASE_H
