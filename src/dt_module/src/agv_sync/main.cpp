#include "agv_sync.h"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "agv_sync");

    AgvSyncNode node;
    node.run();

    return 0;
}
