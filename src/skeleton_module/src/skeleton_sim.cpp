#include "skeleton_sim.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <thread>

#include "gazebo_msgs/LinkState.h"
#include "gazebo_msgs/SpawnModel.h"
#include "geometry_msgs/TransformStamped.h"
#include "kinect.h"
#include "opencv2/opencv.hpp"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"

using namespace cv;
using namespace std;
vector<double> quat2Euler(double rx, double ry, double rz, double rw) {
    tf::Quaternion quaternion(rx, ry, rz, rw);
    tf::Matrix3x3 rpy(quaternion);
    vector<double> result(3, .0);
    rpy.getRPY(result[0], result[1], result[2]);
    return result;
}
vector<double> euler2Quat(double rx, double ry, double rz) {
    tf2::Quaternion quaternion;
    quaternion.setRPY(rx, ry, rz);
    quaternion = quaternion.normalize();

    return {quaternion.getX(), quaternion.getY(), quaternion.getZ(),
            quaternion.getW()};
}
SkeletonSimNode::SkeletonSimNode() {
    spawnClient =
        nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf");

    skeletonPublisher =
        nh.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 10);

    bodyNum = 0;
    isBody = false;

    tfListener = new tf2_ros::TransformListener(tfBuffer);
}
SkeletonSimNode::~SkeletonSimNode() {}

void SkeletonSimNode::run() {
    ros::Rate loop_rate(10);

    spawnHuman();
    while (ros::ok()) {
        moveSkeleton();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
void SkeletonSimNode::spawnHuman() {
    system(("rosrun gazebo_ros spawn_model -sdf -file "
            "/home/kevin/project/robot_simulation/src/"
            "skeleton_module/resource/"
            "human/model.sdf "
            "-model "
            "human1 -x " +
            to_string(0) + " -y " + to_string(0))
               .c_str());
}
double rad2Degree(double rad) { return rad * 180 / M_PI; };

void SkeletonSimNode::moveSkeleton() {
    if (tfBuffer.canTransform("world", "/Spine_Naval0", ros::Time(0))) {
        vector<geometry_msgs::TransformStamped> transforms(12);
        vector<float> positionTmp(7);
        tf::Quaternion q;
        tf::Quaternion deg90;
        deg90.setRPY(0, M_PI_2, 0);

        transforms[SPINE_NAVEL] =
            tfBuffer.lookupTransform("world", "Spine_Naval0", ros::Time(0));
        transforms[NECK] =
            tfBuffer.lookupTransform("world", "Spine_Chest0", ros::Time(0));
        transforms[EYE_RIGHT] =
            tfBuffer.lookupTransform("world", "Ear_Right0", ros::Time(0));
        transforms[EYE_LEFT] =
            tfBuffer.lookupTransform("world", "Ear_Left0", ros::Time(0));
        transforms[SHOULDER_RIGHT] =
            tfBuffer.lookupTransform("world", "Shoulder_right0", ros::Time(0));
        transforms[ELBOW_RIGHT] =
            tfBuffer.lookupTransform("world", "Elbow_right0", ros::Time(0));
        transforms[WRIST_RIGHT] =
            tfBuffer.lookupTransform("world", "Wrist_right0", ros::Time(0));
        transforms[HANDTIP_RIGHT] =
            tfBuffer.lookupTransform("world", "Handtip_right0", ros::Time(0));
        transforms[SHOULDER_LEFT] =
            tfBuffer.lookupTransform("world", "Shoulder_left0", ros::Time(0));
        transforms[ELBOW_LEFT] =
            tfBuffer.lookupTransform("world", "Elbow_left0", ros::Time(0));
        transforms[WRIST_LEFT] =
            tfBuffer.lookupTransform("world", "Wrist_left0", ros::Time(0));
        transforms[HANDTIP_LEFT] =
            tfBuffer.lookupTransform("world", "Handtip_left0", ros::Time(0));

        // head
        positionTmp[0] = (transforms[EYE_LEFT].transform.translation.x +
                          transforms[EYE_RIGHT].transform.translation.x) /
                         2;
        positionTmp[1] = (transforms[EYE_LEFT].transform.translation.y +
                          transforms[EYE_RIGHT].transform.translation.y) /
                         2;
        positionTmp[2] = (transforms[EYE_LEFT].transform.translation.z +
                          transforms[EYE_RIGHT].transform.translation.z) /
                         2;

        positionTmp[3] = 1;
        positionTmp[4] = 0;
        positionTmp[5] = 0;
        positionTmp[6] = 0;
        moveGazebo(positionTmp, "human1::head");

        // body
        positionTmp[0] = (transforms[NECK].transform.translation.x +
                          transforms[SPINE_NAVEL].transform.translation.x) /
                         2;
        positionTmp[1] = (transforms[NECK].transform.translation.y +
                          transforms[SPINE_NAVEL].transform.translation.y) /
                         2;
        positionTmp[2] = (transforms[NECK].transform.translation.z +
                          transforms[SPINE_NAVEL].transform.translation.z) /
                         2;

        q = tf::Quaternion(transforms[SPINE_NAVEL].transform.rotation.x,
                           transforms[SPINE_NAVEL].transform.rotation.y,
                           transforms[SPINE_NAVEL].transform.rotation.z,
                           transforms[SPINE_NAVEL].transform.rotation.w);
        q *= deg90;
        positionTmp[3] = q.getW();
        positionTmp[4] = q.getX();
        positionTmp[5] = q.getY();
        positionTmp[6] = q.getZ();
        moveGazebo(positionTmp, "human1::body");

        // shoulder_left
        positionTmp[0] = (transforms[SHOULDER_LEFT].transform.translation.x +
                          transforms[ELBOW_LEFT].transform.translation.x) /
                         2;
        positionTmp[1] = (transforms[SHOULDER_LEFT].transform.translation.y +
                          transforms[ELBOW_LEFT].transform.translation.y) /
                         2;
        positionTmp[2] = (transforms[SHOULDER_LEFT].transform.translation.z +
                          transforms[ELBOW_LEFT].transform.translation.z) /
                         2;

        q = tf::Quaternion(transforms[SHOULDER_LEFT].transform.rotation.x,
                           transforms[SHOULDER_LEFT].transform.rotation.y,
                           transforms[SHOULDER_LEFT].transform.rotation.z,
                           transforms[SHOULDER_LEFT].transform.rotation.w);
        q *= deg90;
        positionTmp[3] = q.getW();
        positionTmp[4] = q.getX();
        positionTmp[5] = q.getY();
        positionTmp[6] = q.getZ();
        moveGazebo(positionTmp, "human1::shoulder_left");

        // arm_left
        positionTmp[0] = (transforms[ELBOW_LEFT].transform.translation.x +
                          transforms[WRIST_LEFT].transform.translation.x) /
                         2;
        positionTmp[1] = (transforms[ELBOW_LEFT].transform.translation.y +
                          transforms[WRIST_LEFT].transform.translation.y) /
                         2;
        positionTmp[2] = (transforms[ELBOW_LEFT].transform.translation.z +
                          transforms[WRIST_LEFT].transform.translation.z) /
                         2;
        q = tf::Quaternion(transforms[ELBOW_LEFT].transform.rotation.x,
                           transforms[ELBOW_LEFT].transform.rotation.y,
                           transforms[ELBOW_LEFT].transform.rotation.z,
                           transforms[ELBOW_LEFT].transform.rotation.w);

        q *= deg90;
        positionTmp[3] = q.getW();
        positionTmp[4] = q.getX();
        positionTmp[5] = q.getY();
        positionTmp[6] = q.getZ();
        moveGazebo(positionTmp, "human1::arm_left");

        // hand_left
        positionTmp[0] = (transforms[WRIST_LEFT].transform.translation.x +
                          transforms[HANDTIP_LEFT].transform.translation.x) /
                         2;
        positionTmp[1] = (transforms[WRIST_LEFT].transform.translation.y +
                          transforms[HANDTIP_LEFT].transform.translation.y) /
                         2;
        positionTmp[2] = (transforms[WRIST_LEFT].transform.translation.z +
                          transforms[HANDTIP_LEFT].transform.translation.z) /
                         2;

        positionTmp[3] = 1;
        positionTmp[4] = 0;
        positionTmp[5] = 0;
        positionTmp[6] = 0;
        moveGazebo(positionTmp, "human1::hand_left");

        // shoulder_right
        positionTmp[0] = (transforms[SHOULDER_RIGHT].transform.translation.x +
                          transforms[ELBOW_RIGHT].transform.translation.x) /
                         2;
        positionTmp[1] = (transforms[SHOULDER_RIGHT].transform.translation.y +
                          transforms[ELBOW_RIGHT].transform.translation.y) /
                         2;
        positionTmp[2] = (transforms[SHOULDER_RIGHT].transform.translation.z +
                          transforms[ELBOW_RIGHT].transform.translation.z) /
                         2;

        q = tf::Quaternion(transforms[SHOULDER_RIGHT].transform.rotation.x,
                           transforms[SHOULDER_RIGHT].transform.rotation.y,
                           transforms[SHOULDER_RIGHT].transform.rotation.z,
                           transforms[SHOULDER_RIGHT].transform.rotation.w);

        q *= deg90;
        positionTmp[3] = q.getW();
        positionTmp[4] = q.getX();
        positionTmp[5] = q.getY();
        positionTmp[6] = q.getZ();
        moveGazebo(positionTmp, "human1::shoulder_right");

        // arm_right
        positionTmp[0] = (transforms[ELBOW_RIGHT].transform.translation.x +
                          transforms[WRIST_RIGHT].transform.translation.x) /
                         2;
        positionTmp[1] = (transforms[ELBOW_RIGHT].transform.translation.y +
                          transforms[WRIST_RIGHT].transform.translation.y) /
                         2;
        positionTmp[2] = (transforms[ELBOW_RIGHT].transform.translation.z +
                          transforms[WRIST_RIGHT].transform.translation.z) /
                         2;
        q = tf::Quaternion(transforms[ELBOW_RIGHT].transform.rotation.x,
                           transforms[ELBOW_RIGHT].transform.rotation.y,
                           transforms[ELBOW_RIGHT].transform.rotation.z,
                           transforms[ELBOW_RIGHT].transform.rotation.w);

        q *= deg90;
        positionTmp[3] = q.getW();
        positionTmp[4] = q.getX();
        positionTmp[5] = q.getY();
        positionTmp[6] = q.getZ();
        moveGazebo(positionTmp, "human1::arm_right");

        // hand_right
        positionTmp[0] = (transforms[WRIST_RIGHT].transform.translation.x +
                          transforms[HANDTIP_RIGHT].transform.translation.x) /
                         2;
        positionTmp[1] = (transforms[WRIST_RIGHT].transform.translation.y +
                          transforms[HANDTIP_RIGHT].transform.translation.y) /
                         2;
        positionTmp[2] = (transforms[WRIST_RIGHT].transform.translation.z +
                          transforms[HANDTIP_RIGHT].transform.translation.z) /
                         2;

        positionTmp[3] = 1;
        positionTmp[4] = 0;
        positionTmp[5] = 0;
        positionTmp[6] = 0;
        moveGazebo(positionTmp, "human1::hand_right");
    }
}
void SkeletonSimNode::moveGazebo(vector<float> position, std::string linkName) {
    gazebo_msgs::LinkState msg;
    tf::Quaternion q;

    msg.link_name = linkName;
    msg.pose.position.x = position[0];
    msg.pose.position.y = position[1];
    msg.pose.position.z = position[2];
    msg.pose.orientation.w = position[3];
    msg.pose.orientation.x = position[4];
    msg.pose.orientation.y = position[5];
    msg.pose.orientation.z = position[6];

    skeletonPublisher.publish(msg);
}
