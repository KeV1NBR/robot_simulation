#include "agv_sync.h"

#include <vector>

#include "amr/amr_info.h"
#include "gazebo_msgs/ModelState.h"
#include "tf/tf.h"

using namespace std;

AgvSyncNode::AgvSyncNode() {
    agvSubscriber = nh.subscribe<amr::amr_info>(
        "/amr/robot_state", 1, &AgvSyncNode::agvCallBack, this);

    gazeboPublisher =
        nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
}
AgvSyncNode::~AgvSyncNode() {}

void AgvSyncNode::run() {
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        moveGazeboAgv();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
void AgvSyncNode::agvCallBack(const amr::amr_info::ConstPtr& info) {
    this->agvXYZ = info->position;
}
void AgvSyncNode::moveGazeboAgv() {
    gazebo_msgs::ModelState msg;
    msg.model_name = "AMR";
    msg.pose.position.x = agvXYZ[0];
    msg.pose.position.y = agvXYZ[1];

    tf::Quaternion q;
    q.setRPY(0, 0, agvXYZ[2]);
    q = q.normalize();

    msg.pose.orientation.z = q.getZ();
    msg.pose.orientation.w = q.getW();
    this->gazeboPublisher.publish(msg);
}
