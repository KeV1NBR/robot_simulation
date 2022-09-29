#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "kinect.h"

class SkeletonSimNode {
   public:
    SkeletonSimNode();
    ~SkeletonSimNode();
    void run();

   private:
    ros::NodeHandle nh;
    Kinect kinect;

    ros::ServiceClient spawnClient;

    std::string humanModel;

    void spawnHuman();
    void moveSkeleton();
    void setTF(std::vector<float> pos, std::string namei);

    int bodyNum;

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
};
