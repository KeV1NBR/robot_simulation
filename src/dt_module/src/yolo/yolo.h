#pragma once

#include <ros/ros.h>

#include <iostream>
#include <opencv2/core.hpp>

#include "dt_module/bboxes.h"
#include "ros/subscriber.h"
#include "sensor_msgs/Image.h"
#define OPENCV
#include "yolo_v2_class.hpp"

class YoloNode {
   public:
    YoloNode();
    ~YoloNode();

    void run();

   private:
    ros::NodeHandle nh;

    ros::Publisher yoloPublisher;
    ros::Subscriber imgSubscriber;

    Detector* detector;
    cv::Mat img;
    void imgCallback(const sensor_msgs::ImageConstPtr& data);

    dt_module::bboxes bboxMsgConvert(std::vector<bbox_t> predict);
    std::map<int, std::string> cocoNames;
};
