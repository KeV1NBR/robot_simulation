#include "skeleton_sim.h"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <thread>

#include "gazebo_msgs/LinkState.h"
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

    skeletonPublisher =
        nh.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 10);

    bodyNum = 0;
    isBody = false;
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
                if (!isBody) {
                    system(("rosrun gazebo_ros spawn_model -sdf -file "
                            "/home/kevin/model_editor_models/human/model.sdf "
                            "-model "
                            "human1 -x " +
                            to_string(transform.getOrigin().x()) + " -y " +
                            to_string(transform.getOrigin().y()) + " -z " +
                            to_string(transform.getOrigin().z()))
                               .c_str());
                    isBody = true;
                }
                moveSkeleton(skeleton);
            }
        }
    }
}
void SkeletonSimNode::moveSkeleton(std::vector<Point3f> body) {
    std::vector<float> headPos(6);
    tf::StampedTransform transform;
    gazebo_msgs::LinkState msg;

    headPos[0] = (body[EYE_LEFT].x + body[EYE_RIGHT].x) / 2000;
    headPos[1] = (body[EYE_LEFT].y + body[EYE_RIGHT].y) / 2000;
    headPos[2] = (body[EYE_LEFT].z + body[EYE_RIGHT].z) / 2000;

    setTF(headPos, "/human1_head");

    std::this_thread::sleep_for(chrono::milliseconds(100));
    listener.lookupTransform("/human1_head", "/world", ros::Time(0), transform);
    msg.link_name = "human1::head";
    msg.pose.position.x = transform.getOrigin().x();
    msg.pose.position.y = transform.getOrigin().y();
    msg.pose.position.z = transform.getOrigin().z();
    skeletonPublisher.publish(msg);
}

void SkeletonSimNode::setTF(std::vector<float> pos, std::string name) {
    tf::Transform transform;
    tf::Quaternion q;

    transform.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));
    // q.setRPY(pos[3], pos[4], pos[5]);
    transform.setRotation(q);
    broadcaster.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "/kinect", name));
}
