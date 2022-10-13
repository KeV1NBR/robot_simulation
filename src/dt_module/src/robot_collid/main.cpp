#include "robot_collid.h"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_collid");

    RobotCollidNode node;
    node.run();

    return 0;
}
