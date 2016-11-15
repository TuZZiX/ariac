//
// Created by shipei on 10/18/16.
//
// So many hard coded stuffs QAQ


#ifndef CWRU_ARIAC_ARIACBASE_H
#define CWRU_ARIAC_ARIACBASE_H


#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <boost/functional/hash.hpp>

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
#include <osrf_gear/AGVControl.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>

using namespace std;
using namespace Eigen;

const int gridNumber = 60;

// TODO change types to ROS message types

typedef pair<int, int> Grid;
typedef pair<double, double> Dimension;
//typedef map<Grid, bool> GridMap;
enum Direction {
    top, bottom, left, right,
    topLeft, topRight, bottomLeft, bottomRight
};

class GridMap: public unordered_set<Grid, boost::hash<pair<int, int>>> {
public:
    bool checkCollision(Grid grid) {
        return find(grid)==end();
    }

    bool addGrids(Grid start, Grid size, Direction direction) {
        Grid A;
        switch (direction) {
            case topLeft:
                A.first = start.first - size.first;
                A.second = start.second;
                break;
            case topRight:
                A = start;
                break;
            case bottomLeft:
                A.first = start.first - size.first;
                A.second = start.second - size.second;
                break;
            case bottomRight:
                A.first = start.first;
                A.second = start.second - size.second;
                break;
        }
        return addGrids(A, size);
    }

    bool addGrids(Grid start, Grid size) {
        if (start.first >= 0 && start.second >= 0 && size.first >= 0 && size.second >= 0 &&
                start.first < gridNumber && start.second < gridNumber && size.first < gridNumber && size.second < gridNumber) {
            for (int i = start.first; i < size.first; ++i) {
                for (int j = start.second; j < size.second; ++j) {
                    if (!checkCollision(Grid(i, j))) {
                        insert(Grid(i, j));
                    } else {
                        return false;
                    }
                }
            }
        } else {
            return false;
        }
        return true;
    }
};

class PartType {
public:
    string name;
    Dimension size;
    Grid grid;
};

class Part {
public:
    PartType type;
    int id;
    bool traceable = false;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::Vector3 linear;
};

typedef vector<Part> Parts;

class Bin {
public:
    string name;
    Dimension size;
    Grid grid;
    GridMap map;
    geometry_msgs::Pose pose;
    Parts container;
    int priority;
};

const int totalPartsTypes = 8;
const int totalAGVs = 8;
const int totalBins = 8;
const double averageCost = 1000;
const string defaultPartsName[totalPartsTypes] = {"piston_rod_part", "gear_part", "pulley_part", "gasket_part", "part1", "part2", "part3", "part4"};
const double defaultPartsSize[totalPartsTypes][2] = {{0.059,0.052}, {0.078425,0.078425}, {0.23392,0.23392}, {0.31442,0.15684}, {0.3,0.1}, {0.06,0.015}, {0.13,0.07}, {0.09,0.06}};

// this class is used to storage some basic information of the competition and common algorithms.
class AriacBase {
public:
    unordered_map<string, PartType> defaultParts;
    vector<Bin> bins;

    Vector3d AGVBaseCoordinate[totalAGVs];

    double AGVBoundBoxXmin[totalAGVs];
    double AGVBoundBoxYmin[totalAGVs];
    double AGVBoundBoxXmax[totalAGVs];
    double AGVBoundBoxYmax[totalAGVs];

    double ConveyorBoundBoxXmin;
    double ConveyorBoundBoxYmin;
    double ConveyorBoundBoxXmax;
    double ConveyorBoundBoxYmax;

    double BinBoundBoxXmin[totalBins];
    double BinBoundBoxYmin[totalBins];
    double BinBoundBoxXmax[totalBins];
    double BinBoundBoxYmax[totalBins];


    AriacBase() {
        Bin defaultBin;
        AGVBaseCoordinate[0] = {0.12, 3.46,0.75};
        defaultBin.name = "Bin";
        defaultBin.grid.first = 60;
        defaultBin.grid.second = 60;
        defaultBin.size.first = 0.6;
        defaultBin.size.second = 0.6;

        bins = vector<Bin>(totalBins, defaultBin);
        bins[3].name = "bin4";
        bins[3].priority = 5;
        bins[3].pose.position.x = -1.000000;
        bins[3].pose.position.y = 0.995000;
        bins[3].pose.position.z = 0.0;

        AGVBoundBoxXmin[0] = 0.0;
        AGVBoundBoxYmin[0] = 2.7;
        AGVBoundBoxXmax[0] = 0.7;
        AGVBoundBoxYmax[0] = 3.9;

        AGVBoundBoxXmin[1] = 0.0;
        AGVBoundBoxYmin[1] = 0.0;
        AGVBoundBoxXmax[1] = 0.0;
        AGVBoundBoxYmax[1] = 0.0;

        ConveyorBoundBoxXmin = 0.9;
        ConveyorBoundBoxYmin = -4.8;
        ConveyorBoundBoxXmax = 1.6;
        ConveyorBoundBoxYmax = 5.8;

        BinBoundBoxXmin[0] = 0.0;
        BinBoundBoxYmin[0] = 0.0;
        BinBoundBoxXmax[0] = 0.0;
        BinBoundBoxYmax[0] = 0.0;

        for (int j = 0; j < totalBins; ++j) {

        }
        PartType singlePart;
        for (int i = 0; i < totalPartsTypes; ++i) {
            singlePart.name = defaultPartsName[i];
            singlePart.size.first = defaultPartsSize[i][0];
            singlePart.size.second = defaultPartsSize[i][1];
            singlePart.grid.first = (int)ceil(defaultBin.size.first / singlePart.size.first);
            singlePart.grid.second = (int)ceil(defaultBin.size.second / singlePart.size.second);
            defaultParts.insert(make_pair(defaultPartsName[i], singlePart));
        }
    }

    inline double euclideanDistance(geometry_msgs::Point positionA, geometry_msgs::Point positionB) {
        return sqrt(pow(positionA.x - positionB.x, 2) + pow(positionA.y - positionB.y, 2) + pow(positionA.z - positionB.z, 2));
    }
};

#endif //CWRU_ARIAC_ARIACBASE_H
// （╯' - ')╯︵ ┻━┻           ┬─┬ ノ( ' - 'ノ)            (╯°Д°)╯︵ ┻━┻