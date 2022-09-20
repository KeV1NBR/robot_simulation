#include "sync.h"

#include <vector>

#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
using namespace std;

DTNode::DTNode() : tm(nh) {
    this->jointInfoPublisher = nh.advertise<trajectory_msgs::JointTrajectory>(
        "/arm_controller/command", 10);
}
DTNode::~DTNode() {}

void DTNode::run() {
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        jointInfoPublisher.publish(this->getCurrentInfo());
        ros::spinOnce();
        loop_rate.sleep();
    }
}

trajectory_msgs::JointTrajectory DTNode::getCurrentInfo() {
    trajectory_msgs::JointTrajectory result;
    result.joint_names = {"J1", "J2", "J3", "J4", "J5", "J6"};

    for (auto it : tm.getJointPosition()) {
        cout << it << ", ";
    }
    cout << endl;
    result.points.resize(1);

    result.points[0].positions = tm.getJointPosition();
    result.points[0].time_from_start = ros::Duration(0.1);

    return result;
}
