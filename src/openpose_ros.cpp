#include <openpose_ros/openpose_ros.hpp>

#include <openpose/flags.hpp>
#include <openpose/headers.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda.inl.hpp>
#include <opencv2/core/cuda_types.hpp>
#include <opencv2/core/cuda_types.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


cv::Mat img;

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
    img = cv_bridge::toCvShare(msg, "bgr8")->image;
}

OpenPoseROS::OpenPoseROS(int argc, char** argv) {
    ros::init(argc, argv, "openpose_node");
}

void OpenPoseROS::init() {
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
    ros::NodeHandle nh_image;
    image_transport::ImageTransport it_image(nh_image);
    image_transport::Subscriber sub_image = it_image.subscribe("/color_097377233947/image_raw", 1, image_callback);
    opWrapper.start();
    while (ros::ok()) {
        ros::spinOnce();
        if (img.data == NULL) {
            continue;
        }
        const cv::Mat cvImageToProcess = img.clone();
        const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(cvImageToProcess);
        auto datumProcessed = opWrapper.emplaceAndPop(imageToProcess);
        const cv::Mat cvMat = OP_OP2CVCONSTMAT(datumProcessed->at(0)->cvOutputData);
        auto poseKeypoints = datumProcessed->at(0)->poseKeypoints;
        const auto numberPeopleDetected = poseKeypoints.getSize(0);
        const auto numberBodyParts = poseKeypoints.getSize(1);
        cv::imshow("Output", cvMat);
        cv::waitKey(1);
    }
}

