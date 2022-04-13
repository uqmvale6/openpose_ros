#ifndef OPENPOSE_ROS_HPP
#define OPENPOSE_ROS_HPP

#include <ros/ros.h>

class OpenPoseROS {
private:
public:
    OpenPoseROS(int argc, char** argv);
    void init();
};



#endif