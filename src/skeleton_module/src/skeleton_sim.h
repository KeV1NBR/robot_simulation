#pragma once

#include <ros/publisher.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class SkeletonSimNode {
   public:
    SkeletonSimNode();
    ~SkeletonSimNode();
    void run();

   private:
    ros::NodeHandle nh;

    ros::ServiceClient spawnClient;
    ros::Publisher skeletonPublisher;

    std::string humanModel;

    void spawnHuman();
    void moveSkeleton();
    void moveGazebo(std::vector<float> position, std::string linkName);
    int bodyNum;
    bool isBody;

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
};
