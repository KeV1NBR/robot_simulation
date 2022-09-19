#include "sync.h"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "digital_twins");

    DTNode node;
    node.run();

    return 0;
}
