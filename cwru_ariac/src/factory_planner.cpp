//
// Created by tianshipei on 12/3/16.
//

#include <BinManager.h>
#include <CameraEstimator.h>
#include <Cheater.h>
#include <OrderManager.h>
#include <RobotPlanner.h>

enum State {NONE = 0, INIT, END, FILL_ORDER, WAIT};

int main(int argc, char** argv) {
    ros::init(argc, argv, "factory_planner");
    ros::NodeHandle nh;
    CameraEstimator camera(nh);
    RobotPlanner robot(nh);
    OrderManager orderManager(nh);
    State state = INIT;
    while (ros::ok()) {
        camera.waitForUpdate();
        switch (state) {
            case INIT:
                orderManager.startCompetition();
                state = WAIT;
                break;
            case END:
                ROS_INFO("End Competition!");
                return 0;
            case FILL_ORDER:
                state = orderManager.isCompetitionEnd()? END:state;

                break;
            case WAIT:
                state = orderManager.isCompetitionEnd()? END:state;
                for(auto order: orderManager.orders) {
                    for(auto kit: order.second.kits) {
                        for (auto part: kit.objects) {
                            string required = part.type;
                            PartList conveyorParts = findPart(camera.onConveyor, required);
                            PartList binParts;
                            for(auto bin: camera.onBin){
                                PartList temp = findPart(bin, required);
                                for (auto item: temp) {
                                    binParts.push_back(item);
                                }
                            }
                            state = FILL_ORDER;
                            break;
                        }
                        if (state == FILL_ORDER)
                            break;
                    }
                    if (state == FILL_ORDER)
                        break;
                }
                break;
            default:
                ROS_INFO("State machine error!");
                state = INIT;
        }
    }
    return 0;
}

