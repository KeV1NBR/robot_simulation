#include "skeleton_sim.h"

#include <chrono>
#include <fstream>
#include <thread>

#include "gazebo_msgs/SpawnModel.h"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

SkeletonSimNode::SkeletonSimNode() {
    std::ifstream humanModelFile(
        "/home/kevin/project/robot_simulation/src/skeleton_module/resouce/"
        "human/model.sdf");
    humanModel.assign((std::istreambuf_iterator<char>(humanModelFile)),
                      (std::istreambuf_iterator<char>()));

    spawnClient =
        nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf");

    bodyNum = 0;
}
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
        spawnHuman();

        imshow("color", color);
        imshow("depth", depth);
        key = waitKey(1);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
void SkeletonSimNode::spawnHuman() {
    std::vector<k4abt_body_t> bodies = kinect.getBodies();
    if (bodies.size() != 0) {
        if (bodyNum < bodies.size()) {
            for (int i = 0; i < bodies.size() - bodyNum; i++) {
                std::vector<Point3f> skeleton =
                    kinect.getConcernPart(bodies[i]);

                std::vector<float> posKinectFrame = {
                    (skeleton[0].x + skeleton[1].x) / 2000,
                    (skeleton[0].y + skeleton[1].y) / 2000,
                    (skeleton[0].z + skeleton[1].z) / 2000};
                setTF(posKinectFrame,
                      "/human" + to_string(bodyNum + 1) + "_body");

                std::this_thread::sleep_for(chrono::milliseconds(100));
                tf::StampedTransform transform;

                listener.lookupTransform(
                    "/human" + to_string(bodyNum + 1) + "_body", "/world",
                    ros::Time(0), transform);

                gazebo_msgs::SpawnModel msg;

                msg.request.initial_pose.position.x = transform.getOrigin().x();
                msg.request.initial_pose.position.y = transform.getOrigin().y();
                msg.request.initial_pose.position.z = transform.getOrigin().z();
                msg.request.model_name = "human" + to_string(bodyNum + 1);
                msg.request.model_xml = humanModel;

                spawnClient.call(msg);
            }
        }
    }
}
void SkeletonSimNode::moveSkeleton() {}

void SkeletonSimNode::setTF(std::vector<float> pos, std::string name) {
    tf::Transform transform;
    tf::Quaternion q;

    transform.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));
    // q.setRPY(pos[3], pos[4], pos[5]);
    transform.setRotation(q);
    broadcaster.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "/kinect", name));
}
