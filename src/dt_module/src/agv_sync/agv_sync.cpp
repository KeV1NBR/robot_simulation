#include "agv_sync.h"

#include <vector>

using namespace std;

AgvSyncNode::AgvSyncNode() {}
AgvSyncNode::~AgvSyncNode() {}

void AgvSyncNode::run() {
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
