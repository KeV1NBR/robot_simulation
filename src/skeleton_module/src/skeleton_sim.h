#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

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

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;
};
