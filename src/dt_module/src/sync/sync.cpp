#include "sync.h"

#include <vector>

#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
using namespace std;

DTNode::DTNode() : arm("manipulator"), tm(nh) {
    this->jointInfoPublisher = nh.advertise<trajectory_msgs::JointTrajectory>(
        "/arm_controller/command", 10);
    arm.setAccel(50);
    arm.setSpeed(50);
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

    result.points.resize(1);

    result.points[0].positions = tm.getJointPosition();

    return result;
}
