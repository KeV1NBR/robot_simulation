#include "skeleton_sim.h"

#include "opencv2/opencv.hpp"

using namespace cv;

SkeletonSimNode::SkeletonSimNode() {}
SkeletonSimNode::~SkeletonSimNode() {}

void SkeletonSimNode::run() {
    ros::Rate loop_rate(10);
    int key = 0;

    while (ros::ok() && key != 27) {
        cv::Mat color(1920, 1080, CV_8UC3);
        cv::Mat depth(1920, 1080, CV_8UC1);

        kinect.update();
        color = kinect.getRgbImage();
        depth = kinect.getDepthImage();
        imshow("color", color);
        imshow("depth", depth);

        key = waitKey(1);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
