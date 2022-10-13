#include "tf_publisher.h"

#include <vector>

#include "gazebo_msgs/LinkStates.h"

using namespace std;

TfPublisherNode::TfPublisherNode() {
    this->armSubscriber = nh.subscribe<gazebo_msgs::LinkStates>(
        "/gazebo/link_states", 1, &TfPublisherNode::posCallBack, this);
}
TfPublisherNode::~TfPublisherNode() {}

void TfPublisherNode::run() {
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        if (linkStates.name.size() != 0) {
            vector<float> pos = this->getPos(LINK2);
            setTF(getPos(LINK2), "/J2", "/world");
            setTF(getPos(LINK3), "/J3", "/world");
            setTF(getPos(LINK4), "/J4", "/world");
            setTF(getPos(LINK5), "/J5", "/world");
            setTF({0, -0.167, -0.075, 1, 0, 0, 0}, "/LINK2", "/J2");
            setTF({0.153437, 0, 0.0495, 1, 0, 0, 0}, "/LINK3", "/J3");
            setTF({0, 0, -0.038211, 1, 0, 0, 0}, "/LINK4", "/J4");
            setTF({0, -0.16, 0.074, 1, 0, 0, 0}, "/LINK5", "/J5");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void TfPublisherNode::setTF(std::vector<float> pos, std::string parentFrame,
                            std::string frame) {
    tf::Transform transform;
    tf::Quaternion q;

    transform.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));
    // q.setRPY(pos[3], pos[4], pos[5]);
    q.setW(pos[3]);
    q.setX(pos[4]);
    q.setY(pos[5]);
    q.setZ(pos[6]);
    transform.setRotation(q);
    broadcaster.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), frame, parentFrame));
}
vector<float> TfPublisherNode::getPos(LinkName linkName) {
    vector<float> pos;

    pos.push_back(linkStates.pose[linkName].position.x);
    pos.push_back(linkStates.pose[linkName].position.y);
    pos.push_back(linkStates.pose[linkName].position.z);
    pos.push_back(linkStates.pose[linkName].orientation.w);
    pos.push_back(linkStates.pose[linkName].orientation.x);
    pos.push_back(linkStates.pose[linkName].orientation.y);
    pos.push_back(linkStates.pose[linkName].orientation.z);
    return pos;
}
void TfPublisherNode::posCallBack(
    const gazebo_msgs::LinkStates::ConstPtr& linkStates) {
    this->linkStates = *linkStates;
}
