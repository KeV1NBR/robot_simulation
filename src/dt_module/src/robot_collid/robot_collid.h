#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <iostream>

class RobotCollidNode {
   public:
    RobotCollidNode();
    ~RobotCollidNode();

    void run();

   private:
    ros::NodeHandle nh;

    ros::Publisher robotCollidPublisher;

    void moveRobot();

    void moveGazebo(std::string tfName, std::string linkName);

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
};
