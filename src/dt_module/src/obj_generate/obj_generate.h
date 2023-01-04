#pragma once

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <iostream>

#include "dt_module/bboxes.h"
#include "ros/subscriber.h"
#include "sensor_msgs/PointCloud2.h"

class ObjGenerateNode {
   public:
    ObjGenerateNode();
    ~ObjGenerateNode();

    void run();

   private:
    ros::NodeHandle nh;

    ros::Publisher gazeboPublisher;
    ros::Subscriber objSubscriber;
    ros::Subscriber pointCloudSubscriber;

    dt_module::bboxes getObjs();
    void moveObj();

    void moveGazebo(std::string tfName, std::string linkName);

    void objCallback(const dt_module::bboxesConstPtr& data);
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& data);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* listener;

    dt_module::bboxes objs;
    sensor_msgs::PointCloud2 pointCloud;
    std::mutex yolo_mutex;
};
