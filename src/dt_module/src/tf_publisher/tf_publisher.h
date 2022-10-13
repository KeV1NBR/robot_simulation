#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <iostream>

#include "gazebo_msgs/LinkStates.h"
#include "trajectory_msgs/JointTrajectory.h"

class TfPublisherNode {
   public:
    enum LinkName { LINK2 = 7, LINK3 = 8, LINK4 = 9, LINK5 = 10 };
    TfPublisherNode();
    ~TfPublisherNode();

    void run();

   private:
    ros::Subscriber armSubscriber;
    ros::NodeHandle nh;
    tf::TransformBroadcaster broadcaster;
    std::vector<float> getPos(LinkName linkName);
    void setTF(std::vector<float> pos, std::string parentFrame,
               std::string frame);
    void posCallBack(const gazebo_msgs::LinkStates::ConstPtr& linkStates);

    gazebo_msgs::LinkStates linkStates;
};
