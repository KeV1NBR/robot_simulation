#include "yolo.h"

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "dt_module/bbox.h"
#include "dt_module/bboxes.h"
#include "sensor_msgs/Image.h"
#include "yolo_v2_class.hpp"

using namespace std;

YoloNode::YoloNode() {
    string cfg = "/home/kevin/api/darknet/cfg/yolov4.cfg";
    string weights = "/home/kevin/api/darknet/yolov4.weights";
    detector = new Detector(cfg, weights, 0);
    yoloPublisher =
        nh.advertise<dt_module::bboxes>("/dt_module/yolo_result", 10);

    imgSubscriber = nh.subscribe<sensor_msgs::Image>(
        "/rgb/image_raw", 1, &YoloNode::imgCallback, this);

    cocoNames[0] = "person";
    cocoNames[39] = "bottle";
}
YoloNode::~YoloNode() {}

void YoloNode::run() {
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        if (!img.empty()) {
            vector<bbox_t> predict = detector->detect(this->img, 0.1);
            yoloPublisher.publish(bboxMsgConvert(predict));
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
void YoloNode::imgCallback(const sensor_msgs::ImageConstPtr& data) {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(data, "bgr8");
    cv_ptr->image.copyTo(this->img);
}

dt_module::bboxes YoloNode::bboxMsgConvert(vector<bbox_t> predict) {
    dt_module::bboxes msg;

    for (auto it : predict) {
        dt_module::bbox obj;
        obj.x = it.x;
        obj.y = it.y;
        obj.w = it.w;
        obj.h = it.h;
        obj.probability = it.prob;
        obj.id = it.track_id;

        if (it.obj_id == 0 || it.obj_id == 39) {
            obj.Class = cocoNames[it.obj_id];
            msg.objs.push_back(obj);
        }
    }
    return msg;
}
