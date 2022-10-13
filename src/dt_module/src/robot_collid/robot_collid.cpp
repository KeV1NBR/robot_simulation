#include "robot_collid.h"

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "gazebo_msgs/LinkState.h"
using namespace std;

RobotCollidNode::RobotCollidNode() {
    robotCollidPublisher =
        nh.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 10);
}
RobotCollidNode::~RobotCollidNode() {}

void RobotCollidNode::run() {
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        moveRobot();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void RobotCollidNode::moveRobot() {
    moveGazebo("LINK2", "robot_collid::Link2");
    moveGazebo("LINK3", "robot_collid::Link3");
    moveGazebo("LINK4", "robot_collid::Link4");
    moveGazebo("LINK5", "robot_collid::Link5");
}

void RobotCollidNode::moveGazebo(std::string tfName, std::string linkName) {
    tf::StampedTransform transform;
    gazebo_msgs::LinkState msg;
    tf::Quaternion q;

    if (listener.canTransform("world", tfName, ros::Time(0))) {
        listener.lookupTransform("world", tfName, ros::Time(0), transform);
        msg.link_name = linkName;
        msg.pose.position.x = transform.getOrigin().x();
        msg.pose.position.y = transform.getOrigin().y();
        msg.pose.position.z = transform.getOrigin().z();
        q = transform.getRotation();
        msg.pose.orientation.w = q.w();
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        robotCollidPublisher.publish(msg);
    }
}
