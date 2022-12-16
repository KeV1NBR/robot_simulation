#include "obj_generate.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "dt_module/bbox.h"
#include "dt_module/bboxes.h"
#include "gazebo_msgs/LinkState.h"
#include "gazebo_msgs/ModelState.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_listener.h"

using namespace std;

geometry_msgs::Point pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud,
                                    const int u, const int v);

ObjGenerateNode::ObjGenerateNode() {
    gazeboPublisher =
        nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);

    objSubscriber = nh.subscribe<dt_module::bboxes>(
        "/dt_module/yolo_result", 1, &ObjGenerateNode::objCallback, this);

    pointCloudSubscriber = nh.subscribe<sensor_msgs::PointCloud2>(
        "/points2", 1, &ObjGenerateNode::pointCloudCallback, this);

    listener = new tf2_ros::TransformListener(tfBuffer);
}
ObjGenerateNode::~ObjGenerateNode() {}

void ObjGenerateNode::run() {
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        moveObj();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void ObjGenerateNode::moveObj() {
    auto tmp = this->getObjs().objs;
    std::vector<dt_module::bbox> objs(std::begin(tmp), std::end(tmp));

    int count = 0;
    for (auto it : objs) {
        if (pointCloud.fields.size() != 0) {
            geometry_msgs::Point p = pixelTo3DPoint(
                pointCloud, it.x + (it.w / 2), it.y + (it.h / 2));

            geometry_msgs::TransformStamped tf;
            tf.header.frame_id = "depth_camera_link";
            tf.header.stamp = ros::Time::now();
            tf.child_frame_id = "jar" + to_string(count);
            tf.transform.translation.x = p.x;
            tf.transform.translation.y = p.y;
            tf.transform.translation.z = p.z;
            tf.transform.rotation.w = 1;

            tfBuffer.setTransform(tf, "obj_generate_node : moveObj()");

            // cout << it.Class << " : " << p.x << " " << p.y << " " << p.z
            //     << endl;
            moveGazebo("jar" + to_string(count), "jar" + to_string(count));
            count++;
        }
    }
    // cout << endl;
}

void ObjGenerateNode::moveGazebo(std::string tfName, std::string linkName) {
    geometry_msgs::TransformStamped transform;
    gazebo_msgs::ModelState msg;

    if (tfBuffer.canTransform("world", tfName, ros::Time(0))) {
        transform = tfBuffer.lookupTransform("world", tfName, ros::Time(0));
        msg.model_name = linkName;
        msg.pose.position.x = transform.transform.translation.x;
        msg.pose.position.y = transform.transform.translation.y;
        msg.pose.position.z = transform.transform.translation.z - 0.056;
        gazeboPublisher.publish(msg);
    }
}

void ObjGenerateNode::objCallback(const dt_module::bboxesConstPtr& data) {
    this->objs.objs = data->objs;
}

void ObjGenerateNode::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& data) {
    this->pointCloud = *data;
}

dt_module::bboxes ObjGenerateNode::getObjs() {
    std::lock_guard<std::mutex> lock(yolo_mutex);
    auto tmp = this->objs;
    return tmp;
}

geometry_msgs::Point pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud,
                                    const int u, const int v) {
    geometry_msgs::Point p;

    // get width and height of 2D point cloud data
    int width = pCloud.width;
    int height = pCloud.height;

    // cout << width << ", " << height << ", " << pCloud.data.size() << endl;
    // Convert from u (column / width), v (row/height) to position in array
    // where X,Y,Z data starts
    int arrayPosition = (v * 1920 + u) * 32;

    // compute position in array where x,y,z data start
    int arrayPosX =
        arrayPosition + pCloud.fields[0].offset;  // X has an offset of 0
    int arrayPosY =
        arrayPosition + pCloud.fields[1].offset;  // Y has an offset of 4
    int arrayPosZ =
        arrayPosition + pCloud.fields[2].offset;  // Z has an offset of 8

    // cout << arrayPosition << ", " << arrayPosX << ", " << arrayPosY << ", "
    //     << arrayPosZ << endl;
    float x = .0;
    float y = .0;
    float z = .0;
    memcpy(&x, &pCloud.data[arrayPosX], sizeof(float));
    memcpy(&y, &pCloud.data[arrayPosY], sizeof(float));
    memcpy(&z, &pCloud.data[arrayPosZ], sizeof(float));

    while (isnan(x)) {
        arrayPosX += 32;
        arrayPosY += 32;
        arrayPosZ += 32;
        memcpy(&x, &pCloud.data[arrayPosX], sizeof(float));
        memcpy(&y, &pCloud.data[arrayPosY], sizeof(float));
        memcpy(&z, &pCloud.data[arrayPosZ], sizeof(float));
    }

    //// put data into the point p
    p.x = x;
    p.y = y;
    p.z = z;

    return p;
}
