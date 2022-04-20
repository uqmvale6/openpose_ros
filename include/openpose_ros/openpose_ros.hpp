#ifndef OPENPOSE_ROS_HPP
#define OPENPOSE_ROS_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <openpose/headers.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda.inl.hpp>
#include <opencv2/core/cuda_types.hpp>
#include <opencv2/core/cuda_types.hpp>

class OpenPoseROS {
private:
    cv::Mat cvMat;
    op::Array<float> poseKeypoints;
    
    std::string image_topic;
    bool compressed;
    std::unique_ptr<image_transport::ImageTransport> it_image;
    image_transport::Subscriber sub_image;
public:
    OpenPoseROS(int argc, char** argv);
    void init();
    void loop();
    void start();
    bool ok();
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
};



#endif