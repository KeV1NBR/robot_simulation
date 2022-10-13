#include "tf_publisher.h"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_publisher");

    TfPublisherNode node;
    node.run();

    return 0;
}
