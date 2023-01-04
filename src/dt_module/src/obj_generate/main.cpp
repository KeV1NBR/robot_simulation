#include "obj_generate.h"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "obj_generate");

    ObjGenerateNode node;
    node.run();

    return 0;
}
