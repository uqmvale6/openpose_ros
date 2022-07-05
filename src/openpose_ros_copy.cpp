#include <signal.h>
#include <cstdio>
#include <cstring>


#include <argparse/argparse.hpp>


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


struct OpenPoseROSArgs : public argparse::Args {
    std::string &image_topic = arg("Image Topic");
    bool& compressed = kwarg("c,compressed", "Compressed Image Topic?", "true").set_default("false");
};


OpenPoseROS::OpenPoseROS(int argc, char** argv) {
    OpenPoseROSArgs args = argparse::parse<OpenPoseROSArgs>(argc, argv);
    image_topic = args.image_topic;
    compressed = args.compressed;
    args.print();
    ros::init(argc, argv, "openpose_node");
}


bool OpenPoseROS::ok() {
    return ros::ok();
}

void OpenPoseROS::loop() {
    ros::spinOnce();
    if (img.data == NULL) {
        return;
    }
    const cv::Mat cvImageToProcess = img.clone();
    const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(cvImageToProcess);
    auto datumProcessed = opWrapper.emplaceAndPop(imageToProcess);

    cvMat = OP_OP2CVCONSTMAT(datumProcessed->at(0)->cvOutputData);
    poseKeypoints = datumProcessed->at(0)->poseKeypoints;
    cv::imshow("Output", cvMat);
    cv::waitKey(1);
}

void OpenPoseROS::start() {
    opWrapper.start();
}

void OpenPoseROS::init() {
    
    const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
    const auto netInputSize = op::flagsToPoint(op::String("-1x160"), "-1x160");
    const auto faceNetInputSize = op::flagsToPoint(op::String(FLAGS_face_net_resolution), "368x368 (multiples of 16)");
    const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
    const auto poseMode = op::flagsToPoseMode(FLAGS_body);
    const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
    
    const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
    const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg, FLAGS_heatmaps_add_PAFs);
    const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
    const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1);
    const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
    const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
    const bool enableGoogleLogging = true;

    const op::WrapperStructPose wrapperStructPose{
        poseMode,
        netInputSize,
        FLAGS_net_resolution_dynamic,
        outputSize,
        keypointScaleMode,
        FLAGS_num_gpu,
        FLAGS_num_gpu_start,
        FLAGS_scale_number,
        (float)FLAGS_scale_gap,
        op::flagsToRenderMode(FLAGS_render_pose, multipleView),
        poseModel,
        !FLAGS_disable_blending,
        (float)FLAGS_alpha_pose,
        (float)FLAGS_alpha_heatmap,
        FLAGS_part_to_show,
        op::String("/home/medrobotics/openpose/models"),
        heatMapTypes, heatMapScaleMode,
        FLAGS_part_candidates,
        (float)FLAGS_render_threshold,
        FLAGS_number_people_max,
        FLAGS_maximize_positives,
        FLAGS_fps_max,
        op::String(FLAGS_prototxt_path),
        op::String(FLAGS_caffemodel_path),
        (float)FLAGS_upsampling_ratio,
        enableGoogleLogging
    };

    opWrapper.configure(wrapperStructPose);

    it_image = std::make_unique<image_transport::ImageTransport>(ros::NodeHandle());
    image_transport::TransportHints hints;
    if (compressed) {
        hints = image_transport::TransportHints("compressed");
    } else {
        hints = image_transport::TransportHints("raw");
    }
    sub_image = it_image->subscribe(image_topic, 1, image_callback, ros::VoidPtr(), hints);
}