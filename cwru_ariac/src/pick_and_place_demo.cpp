//
// Created by shipei on 11/8/16.
//

#include <AriacBase.h>
#include <OrderManager.h>

ros::Publisher joint_trajectory_publisher;
sensor_msgs::JointState current_joint_states;
bool called = false;

/// Create a JointTrajectory with all positions set to zero, and command the arm.
void sendArmCommand(vector<double> joints) {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    // Copy the joint names from the msg off the '/ariac/joint_states' topic.
    msg.joint_names = current_joint_states.name;
    // Create one point in the trajectory.
    msg.points.resize(1);
    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    msg.points[0].positions.resize(current_joint_states.name.size(), 0.0);
    for (int i = 0; i < joints.size(); ++i) {
        msg.points[0].positions[i] = joints[i];
    }
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(1);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
}

void joint_state_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg)
{
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    current_joint_states = *joint_state_msg;
    called = true;
}

void sendCommand(vector<double> my_pose) {
    sendArmCommand(my_pose);
    ros::Duration(1).sleep();                         // wait for finish
    ros::spinOnce();
    called = false;
    while(!called) {
        ros::spinOnce();
        ros::Duration(0.02).sleep();
    }

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_and_place_demo");
    ros::NodeHandle nh;                                 // standard ros node handle
    ros::Subscriber joint_state_subscriber = nh.subscribe(
            "/ariac/joint_states", 10,
            &joint_state_callback);
    joint_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/ariac/arm/command", 10);

    OrderManager comp(nh);
    vector<double> my_pose;
    while(!called) {
        ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.2).sleep();
    }

    comp.startCompetition();
    ROS_INFO("Competition started!");
    ros::Duration(19).sleep();
    my_pose.resize(current_joint_states.name.size(), 0.0);
    for (int i = 0; i < my_pose.size(); ++i) {
        my_pose = current_joint_states.position;
    }
    my_pose[0] = 1.288514;
    my_pose[1] = 1.962313;
    my_pose[2] = -0.623798;
    my_pose[3] = 0.134005;
    my_pose[4] = 4.051464;
    my_pose[5] = -1.527353;
    my_pose[6] = -3.179198;

    sendCommand(my_pose);
    ros::Duration(0.2).sleep();
    return 0;
}