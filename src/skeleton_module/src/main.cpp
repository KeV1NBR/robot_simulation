#include "skeleton_sim.h"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "skeleton_sim");

    SkeletonSimNode node;
    node.run();
    return 0;
}
