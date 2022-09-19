#pragma once

#include <ros/ros.h>

#include <iostream>

#include "arm.h"
#include "tm.h"
#include "trajectory_msgs/JointTrajectory.h"

class DTNode {
   public:
    DTNode();
    ~DTNode();

    void run();

   private:
    ros::Publisher jointInfoPublisher;
    ros::NodeHandle nh;

    trajectory_msgs::JointTrajectory getCurrentInfo();

    Arm arm;
    Tm tm;
};
