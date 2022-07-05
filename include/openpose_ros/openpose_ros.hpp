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
    
    std::string image_color_topic;
    std::string image_depth_topic;
    std::string serial;
    bool compressed;
    std::unique_ptr<image_transport::ImageTransport> it_image_color;
    std::unique_ptr<image_transport::ImageTransport> it_image_depth;
    image_transport::Subscriber sub_image_color;
    image_transport::Subscriber sub_image_depth;
    ros::Publisher pub_pointstamped_head;
    ros::Publisher pub_pointstamped_wrist_l;
    ros::Publisher pub_pointstamped_elbow_l;
    ros::Publisher pub_pointstamped_shoulder_l;
    ros::Publisher pub_pointstamped_wrist_r;
    ros::Publisher pub_pointstamped_elbow_r;
    ros::Publisher pub_pointstamped_shoulder_r;
    ros::Publisher pub_pointstamped_chest;
    float x_arr[8];
    float y_arr[8];
    float z_arr[8];
public:
    OpenPoseROS(int argc, char** argv);
    void init();
    void loop();
    void start();
    bool ok();
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
};



#endif