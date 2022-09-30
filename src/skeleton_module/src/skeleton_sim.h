#pragma once

#include <ros/publisher.h>
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
    ros::Publisher skeletonPublisher;

    std::string humanModel;

    void spawnHuman();
    void moveSkeleton(std::vector<cv::Point3f> body);
    void setTF(std::vector<float> pos, std::string name);

    int bodyNum;
    bool isBody;

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
};
