#include "yolo.h"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "kinect_yolo");

    YoloNode node;
    node.run();

    return 0;
}
