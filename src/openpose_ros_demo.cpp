#include <openpose_ros/openpose_ros.hpp>
#include <image_transport/image_transport.h>

int main(int argc, char** argv) {

    OpenPoseROS op_ros(argc, argv);
    op_ros.init();
    op_ros.start();
    while (op_ros.ok()) {
        op_ros.loop();
    }
}