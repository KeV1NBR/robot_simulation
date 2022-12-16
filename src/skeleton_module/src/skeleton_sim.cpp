#include "skeleton_sim.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <thread>

#include "gazebo_msgs/LinkState.h"
#include "gazebo_msgs/SpawnModel.h"
#include "kinect.h"
#include "opencv2/opencv.hpp"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Transform.h"
#include "tf/transform_datatypes.h"

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
    if (listener.canTransform("world", "/Spine_Naval0", ros::Time(0))) {
        vector<tf::StampedTransform> transforms(12);
        vector<float> positionTmp(7);
        tf::Quaternion q;
        tf::Quaternion deg90;
        deg90.setRPY(0, M_PI_2, 0);

        listener.lookupTransform("/world", "/Spine_Naval0", ros::Time(0),
                                 transforms[SPINE_NAVEL]);
        listener.lookupTransform("/world", "/Spine_Chest0", ros::Time(0),
                                 transforms[NECK]);
        listener.lookupTransform("/world", "/Ear_Right0", ros::Time(0),
                                 transforms[EYE_RIGHT]);
        listener.lookupTransform("/world", "/Ear_Left0", ros::Time(0),
                                 transforms[EYE_LEFT]);
        listener.lookupTransform("/world", "/Shoulder_right0", ros::Time(0),
                                 transforms[SHOULDER_RIGHT]);
        listener.lookupTransform("/world", "/Elbow_right0", ros::Time(0),
                                 transforms[ELBOW_RIGHT]);
        listener.lookupTransform("/world", "/Wrist_right0", ros::Time(0),
                                 transforms[WRIST_RIGHT]);
        listener.lookupTransform("/world", "/Handtip_right0", ros::Time(0),
                                 transforms[HANDTIP_RIGHT]);
        listener.lookupTransform("/world", "Shoulder_left0", ros::Time(0),
                                 transforms[SHOULDER_LEFT]);
        listener.lookupTransform("/world", "Elbow_left0", ros::Time(0),
                                 transforms[ELBOW_LEFT]);
        listener.lookupTransform("/world", "Wrist_left0", ros::Time(0),
                                 transforms[WRIST_LEFT]);
        listener.lookupTransform("/world", "Handtip_left0", ros::Time(0),
                                 transforms[HANDTIP_LEFT]);

        // head
        positionTmp[0] = (transforms[EYE_LEFT].getOrigin().x() +
                          transforms[EYE_RIGHT].getOrigin().x()) /
                         2;
        positionTmp[1] = (transforms[EYE_LEFT].getOrigin().y() +
                          transforms[EYE_RIGHT].getOrigin().y()) /
                         2;
        positionTmp[2] = (transforms[EYE_LEFT].getOrigin().z() +
                          transforms[EYE_RIGHT].getOrigin().z()) /
                         2;

        positionTmp[3] = 1;
        positionTmp[4] = 0;
        positionTmp[5] = 0;
        positionTmp[6] = 0;
        moveGazebo(positionTmp, "human1::head");

        // body
        positionTmp[0] = (transforms[NECK].getOrigin().x() +
                          transforms[SPINE_NAVEL].getOrigin().x()) /
                         2;
        positionTmp[1] = (transforms[NECK].getOrigin().y() +
                          transforms[SPINE_NAVEL].getOrigin().y()) /
                         2;
        positionTmp[2] = (transforms[NECK].getOrigin().z() +
                          transforms[SPINE_NAVEL].getOrigin().z()) /
                         2;

        q = transforms[SPINE_NAVEL].getRotation();
        q *= deg90;
        positionTmp[3] = q.getW();
        positionTmp[4] = q.getX();
        positionTmp[5] = q.getY();
        positionTmp[6] = q.getZ();
        moveGazebo(positionTmp, "human1::body");

        // shoulder_left
        positionTmp[0] = (transforms[SHOULDER_LEFT].getOrigin().x() +
                          transforms[ELBOW_LEFT].getOrigin().x()) /
                         2;
        positionTmp[1] = (transforms[SHOULDER_LEFT].getOrigin().y() +
                          transforms[ELBOW_LEFT].getOrigin().y()) /
                         2;
        positionTmp[2] = (transforms[SHOULDER_LEFT].getOrigin().z() +
                          transforms[ELBOW_LEFT].getOrigin().z()) /
                         2;

        q = transforms[SHOULDER_LEFT].getRotation();
        q *= deg90;
        positionTmp[3] = q.getW();
        positionTmp[4] = q.getX();
        positionTmp[5] = q.getY();
        positionTmp[6] = q.getZ();
        moveGazebo(positionTmp, "human1::shoulder_left");

        // arm_left
        positionTmp[0] = (transforms[ELBOW_LEFT].getOrigin().x() +
                          transforms[WRIST_LEFT].getOrigin().x()) /
                         2;
        positionTmp[1] = (transforms[ELBOW_LEFT].getOrigin().y() +
                          transforms[WRIST_LEFT].getOrigin().y()) /
                         2;
        positionTmp[2] = (transforms[ELBOW_LEFT].getOrigin().z() +
                          transforms[WRIST_LEFT].getOrigin().z()) /
                         2;

        q = transforms[ELBOW_LEFT].getRotation();
        q *= deg90;
        positionTmp[3] = q.getW();
        positionTmp[4] = q.getX();
        positionTmp[5] = q.getY();
        positionTmp[6] = q.getZ();
        moveGazebo(positionTmp, "human1::arm_left");

        // hand_left
        positionTmp[0] = (transforms[WRIST_LEFT].getOrigin().x() +
                          transforms[HANDTIP_LEFT].getOrigin().x()) /
                         2;
        positionTmp[1] = (transforms[WRIST_LEFT].getOrigin().y() +
                          transforms[HANDTIP_LEFT].getOrigin().y()) /
                         2;
        positionTmp[2] = (transforms[WRIST_LEFT].getOrigin().z() +
                          transforms[HANDTIP_LEFT].getOrigin().z()) /
                         2;

        positionTmp[3] = 1;
        positionTmp[4] = 0;
        positionTmp[5] = 0;
        positionTmp[6] = 0;
        moveGazebo(positionTmp, "human1::hand_left");

        // shoulder_right
        positionTmp[0] = (transforms[SHOULDER_RIGHT].getOrigin().x() +
                          transforms[ELBOW_RIGHT].getOrigin().x()) /
                         2;
        positionTmp[1] = (transforms[SHOULDER_RIGHT].getOrigin().y() +
                          transforms[ELBOW_RIGHT].getOrigin().y()) /
                         2;
        positionTmp[2] = (transforms[SHOULDER_RIGHT].getOrigin().z() +
                          transforms[ELBOW_RIGHT].getOrigin().z()) /
                         2;

        q = transforms[SHOULDER_RIGHT].getRotation();
        q *= deg90;
        positionTmp[3] = q.getW();
        positionTmp[4] = q.getX();
        positionTmp[5] = q.getY();
        positionTmp[6] = q.getZ();
        moveGazebo(positionTmp, "human1::shoulder_right");

        // arm_right
        positionTmp[0] = (transforms[ELBOW_RIGHT].getOrigin().x() +
                          transforms[WRIST_RIGHT].getOrigin().x()) /
                         2;
        positionTmp[1] = (transforms[ELBOW_RIGHT].getOrigin().y() +
                          transforms[WRIST_RIGHT].getOrigin().y()) /
                         2;
        positionTmp[2] = (transforms[ELBOW_RIGHT].getOrigin().z() +
                          transforms[WRIST_RIGHT].getOrigin().z()) /
                         2;

        q = transforms[ELBOW_RIGHT].getRotation();
        q *= deg90;
        positionTmp[3] = q.getW();
        positionTmp[4] = q.getX();
        positionTmp[5] = q.getY();
        positionTmp[6] = q.getZ();
        moveGazebo(positionTmp, "human1::arm_right");

        // hand_right
        positionTmp[0] = (transforms[WRIST_RIGHT].getOrigin().x() +
                          transforms[HANDTIP_RIGHT].getOrigin().x()) /
                         2;
        positionTmp[1] = (transforms[WRIST_RIGHT].getOrigin().y() +
                          transforms[HANDTIP_RIGHT].getOrigin().y()) /
                         2;
        positionTmp[2] = (transforms[WRIST_RIGHT].getOrigin().z() +
                          transforms[HANDTIP_RIGHT].getOrigin().z()) /
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
