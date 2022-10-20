#pragma once

#include <ros/ros.h>

#include <iostream>

#include "amr/amr_info.h"

class AgvSyncNode {
   public:
    AgvSyncNode();
    ~AgvSyncNode();

    void run();

   private:
    ros::NodeHandle nh;

    ros::Subscriber agvSubscriber;
    void agvCallBack(const amr::amr_info::ConstPtr& info);

    ros::Publisher gazeboPublisher;
    std::vector<float> agvXYZ;

    void moveGazeboAgv();
};
