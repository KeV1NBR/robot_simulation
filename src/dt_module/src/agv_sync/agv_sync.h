#pragma once

#include <ros/ros.h>

#include <iostream>

class AgvSyncNode {
   public:
    AgvSyncNode();
    ~AgvSyncNode();

    void run();

   private:
    ros::NodeHandle nh;
};
