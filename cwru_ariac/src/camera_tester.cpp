//
// Created by shipei on 11/1/16.
//

#include <ConveyorManager.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_tester");
    ros::NodeHandle nh;
    CameraEstimator camera(nh);
    RobotPlanner robot(nh);
    ConveyorManager conveyor(nh, camera, robot);
    while (ros::ok()) {
        bool found = false;
        camera.waitForUpdate();
        for (int i = 0; i < camera.inView.size(); ++i) {
            if (camera.inView[i].id == atoi(argv[1])) {
                cout << "Id: " << camera.inView[i].id << endl
                     << "Name: " << camera.inView[i].type.name << endl
                     << "Traceable: " << camera.inView[i].traceable << endl;
                ROS_INFO_STREAM("Pose:\n" << camera.inView[i].pose);
                ROS_INFO_STREAM("Linear:\n" << camera.inView[i].linear);
                found = true;
            }
        }
        if (!found)
            cout << "Id: " << atoi(argv[1]) << " is out of view" << endl;
        cout << "inView size: " << camera.inView.size() << ", onConveyor size: " << camera.onConveyor.size() << ", valid id from: " << camera.onConveyor.begin()->id << " to: " << (camera.onConveyor.end()-1)->id << endl;
    }
}
