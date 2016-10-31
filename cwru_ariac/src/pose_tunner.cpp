//
// Created by shipei on 10/25/16.
//

#include <AriacBase.h>

ros::Publisher joint_trajectory_publisher;
sensor_msgs::JointState current_joint_states;
bool called = false;

/// Create a JointTrajectory with all positions set to zero, and command the arm.
void send_arm_command(vector<double> joints) {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;
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
    msg.points[0].time_from_start = ros::Duration(0.1);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
}


void joint_state_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg)
{
    ROS_INFO_STREAM_THROTTLE(10,
                             "Joint States (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    current_joint_states = *joint_state_msg;
    called = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_tunner");

    ros::NodeHandle nh;                                 // standard ros node handle
    ros::Subscriber joint_state_subscriber = nh.subscribe(
            "/ariac/joint_states", 10,
            &joint_state_callback);
    joint_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/ariac/arm/command", 10);
    int joint;
    double angle;
    vector<double> my_pose;
    while(!called) {
        ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    my_pose.resize(current_joint_states.name.size(), 0.0);
    for (int i = 0; i < my_pose.size(); ++i) {
        my_pose = current_joint_states.position;
    }

    while (ros::ok()) {
        ROS_WARN("Current joints: [%f, %f, %f, %f, %f, %f, %f]",
                 my_pose[0], my_pose[1], my_pose[2], my_pose[3],
                 my_pose[4], my_pose[5], my_pose[6]);
        std::cout << "Joint number:";
        std::cin >> joint;
        std::cout << "Inc/dec angles:";
        std::cin >> angle;
        my_pose[joint] += angle;
        send_arm_command(my_pose);
        ros::Duration(0.2).sleep();                           // wait for finish
    }
    return 0;
}