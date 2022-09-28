#pragma once

#include <ros/ros.h>

#include "kinect.h"

class SkeletonSimNode {
   public:
    SkeletonSimNode();
    ~SkeletonSimNode();
    void run();

   private:
    ros::NodeHandle nh;
    Kinect kinect;
};
