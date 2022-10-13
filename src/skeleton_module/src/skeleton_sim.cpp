#include "skeleton_sim.h"

#include <chrono>
#include <cmath>
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
        "/home/kevin/project/robot_simulation/src/skeleton_module/resource/"
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

                if (!isBody) {
                    std::vector<float> posKinectFrame = {
                        (skeleton[0].x + skeleton[1].x) / 2000,
                        (skeleton[0].y + skeleton[1].y) / 2000,
                        (skeleton[0].z + skeleton[1].z) / 2000};
                    setTF(posKinectFrame,
                          "/human" + to_string(bodyNum + 1) + "_body");

                    std::this_thread::sleep_for(chrono::milliseconds(100));
                    tf::StampedTransform transform;

                    listener.lookupTransform(
                        "/world", "/human" + to_string(bodyNum + 1) + "_body",
                        ros::Time(0), transform);

                    system(("rosrun gazebo_ros spawn_model -sdf -file "
                            "/home/kevin/project/robot_simulation/src/"
                            "skeleton_module/resource/"
                            "human/model.sdf "
                            "-model "
                            "human1 -x " +
                            to_string(transform.getOrigin().x()) + " -y " +
                            to_string(transform.getOrigin().y()))
                               .c_str());
                    isBody = true;
                }
                moveSkeleton(skeleton);
            }
        }
    }
}
double rad2Degree(double rad) { return rad * 180 / M_PI; };

void SkeletonSimNode::moveSkeleton(std::vector<Point3f> body) {
    std::vector<float> headPos(6, .0);
    std::vector<float> bodyPos(6, .0);

    std::vector<float> shoulderLeftPos(6, .0);
    std::vector<float> armLeftPos(6, .0);
    std::vector<float> handLeftPos(6, .0);

    std::vector<float> shoulderRightPos(6, .0);
    std::vector<float> armRightPos(6, .0);
    std::vector<float> handRightPos(6, .0);

    headPos[0] = (body[EYE_LEFT].x + body[EYE_RIGHT].x) / 2000;
    headPos[1] = (body[EYE_LEFT].y + body[EYE_RIGHT].y) / 2000 + .1;
    headPos[2] = (body[EYE_LEFT].z + body[EYE_RIGHT].z) / 2000 + 0.15;

    bodyPos[0] = (body[NECK].x + body[SPINE_NAVEL].x) / 2000;
    bodyPos[1] = (body[NECK].y + body[SPINE_NAVEL].y) / 2000 + .2;
    bodyPos[2] = (body[NECK].z + body[SPINE_NAVEL].z) / 2000;
    // bodyPos[3] = atan2((body[NECK].z - body[SPINE_NAVEL].z),
    //                   (body[NECK].y + body[SPINE_NAVEL].y)) +
    //             M_PI_2;
    // bodyPos[4] = atan2((body[NECK].x - body[SPINE_NAVEL].x),
    //                   (body[NECK].z + body[SPINE_NAVEL].z));
    // bodyPos[5] = atan2((body[NECK].y - body[SPINE_NAVEL].y),
    //                   (body[NECK].x + body[SPINE_NAVEL].x));

    shoulderLeftPos[0] = (body[SHOULDER_LEFT].x + body[ELBOW_LEFT].x) / 2000;
    shoulderLeftPos[1] = (body[SHOULDER_LEFT].y + body[ELBOW_LEFT].y) / 2000;
    shoulderLeftPos[2] = (body[SHOULDER_LEFT].z + body[ELBOW_LEFT].z) / 2000;

    shoulderRightPos[0] = (body[SHOULDER_RIGHT].x + body[ELBOW_RIGHT].x) / 2000;
    shoulderRightPos[1] = (body[SHOULDER_RIGHT].y + body[ELBOW_RIGHT].y) / 2000;
    shoulderRightPos[2] = (body[SHOULDER_RIGHT].z + body[ELBOW_RIGHT].z) / 2000;

    armLeftPos[0] = (body[ELBOW_LEFT].x + body[WRIST_LEFT].x) / 2000;
    armLeftPos[1] = (body[ELBOW_LEFT].y + body[WRIST_LEFT].y) / 2000;
    armLeftPos[2] = (body[ELBOW_LEFT].z + body[WRIST_LEFT].z) / 2000;

    armRightPos[0] = (body[ELBOW_RIGHT].x + body[WRIST_RIGHT].x) / 2000;
    armRightPos[1] = (body[ELBOW_RIGHT].y + body[WRIST_RIGHT].y) / 2000;
    armRightPos[2] = (body[ELBOW_RIGHT].z + body[WRIST_RIGHT].z) / 2000;

    handLeftPos[0] = (body[WRIST_LEFT].x + body[HANDTIP_LEFT].x) / 2000;
    handLeftPos[1] = (body[WRIST_LEFT].y + body[HANDTIP_LEFT].y) / 2000;
    handLeftPos[2] = (body[WRIST_LEFT].z + body[HANDTIP_LEFT].z) / 2000;

    handRightPos[0] = (body[WRIST_RIGHT].x + body[HANDTIP_RIGHT].x) / 2000;
    handRightPos[1] = (body[WRIST_RIGHT].y + body[HANDTIP_RIGHT].y) / 2000;
    handRightPos[2] = (body[WRIST_RIGHT].z + body[HANDTIP_RIGHT].z) / 2000;

    bodyPos[3] = M_PI_2;
    shoulderLeftPos[4] = M_PI_2;
    shoulderRightPos[4] = M_PI_2;
    armLeftPos[4] = M_PI_2;
    armRightPos[4] = M_PI_2;

    setTF(headPos, "/human1_head");
    setTF(bodyPos, "/human1_body");
    setTF(shoulderLeftPos, "/human1_shoulder_left");
    setTF(shoulderRightPos, "/human1_shoulder_right");
    setTF(armLeftPos, "/human1_arm_left");
    setTF(armRightPos, "/human1_arm_right");
    setTF(handLeftPos, "/human1_hand_left");
    setTF(handRightPos, "/human1_hand_right");

    std::this_thread::sleep_for(chrono::milliseconds(100));

    moveGazebo("/human1_head", "human1::head");
    moveGazebo("/human1_body", "human1::body");
    moveGazebo("/human1_shoulder_left", "human1::shoulder_left");
    moveGazebo("/human1_arm_left", "human1::arm_left");
    moveGazebo("/human1_hand_left", "human1::hand_left");
    moveGazebo("/human1_shoulder_right", "human1::shoulder_right");
    moveGazebo("/human1_arm_right", "human1::arm_right");
    moveGazebo("/human1_hand_right", "human1::hand_right");
}

void SkeletonSimNode::setTF(std::vector<float> pos, std::string name) {
    tf::Transform transform;
    tf::Quaternion q;

    transform.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));
    q.setRPY(pos[3], pos[4], pos[5]);
    transform.setRotation(q);
    broadcaster.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "/kinect", name));
}
vector<double> quat2Euler(double rx, double ry, double rz, double rw) {
    tf::Quaternion quaternion(rx, ry, rz, rw);
    tf::Matrix3x3 rpy(quaternion);
    vector<double> result(3, .0);
    rpy.getRPY(result[0], result[1], result[2]);

    return result;
}
void SkeletonSimNode::moveGazebo(std::string tfName, std::string linkName) {
    tf::StampedTransform transform;
    gazebo_msgs::LinkState msg;
    tf::Quaternion q;

    listener.lookupTransform("/world", tfName, ros::Time(0), transform);
    msg.link_name = linkName;
    msg.pose.position.x = transform.getOrigin().x();
    msg.pose.position.y = transform.getOrigin().y();
    msg.pose.position.z = transform.getOrigin().z();
    q = transform.getRotation();
    msg.pose.orientation.w = q.w();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    skeletonPublisher.publish(msg);

    if (linkName == "human1::body") {
        vector<double> tmp = quat2Euler(q.x(), q.y(), q.z(), q.w());
        for (int i = 0; i < tmp.size(); i++) {
            // cout << tmp[i] << ", ";
        }
        //      cout << endl;
    }
}
