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
    
    // Applying user defined configuration - GFlags to program variables
    
    // outputSize
    const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
    // netInputSize
    const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x160");
    // faceNetInputSize
    const auto faceNetInputSize = op::flagsToPoint(op::String(FLAGS_face_net_resolution), "368x368 (multiples of 16)");
    // handNetInputSize
    const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
    // poseMode
    const auto poseMode = op::flagsToPoseMode(FLAGS_body);
    // poseModel
    const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
    // JSON saving
    if (!FLAGS_write_keypoint.empty())
        op::opLog(
            "Flag `write_keypoint` is deprecated and will eventually be removed. Please, use `write_json`"
            " instead.", op::Priority::Max);
    // keypointScaleMode
    const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
    // heatmaps to add
    const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
                                                    FLAGS_heatmaps_add_PAFs);
    const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
    // >1 camera view?
    const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1);
    // Face and hand detectors
    const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
    const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
    // Enabling Google Logging
    const bool enableGoogleLogging = true;

    // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
    const op::WrapperStructPose wrapperStructPose{
        poseMode, netInputSize, FLAGS_net_resolution_dynamic, outputSize, keypointScaleMode, FLAGS_num_gpu,
        FLAGS_num_gpu_start, FLAGS_scale_number, (float)FLAGS_scale_gap,
        op::flagsToRenderMode(FLAGS_render_pose, multipleView), poseModel, !FLAGS_disable_blending,
        (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap, FLAGS_part_to_show, op::String(FLAGS_model_folder),
        heatMapTypes, heatMapScaleMode, FLAGS_part_candidates, (float)FLAGS_render_threshold,
        FLAGS_number_people_max, FLAGS_maximize_positives, FLAGS_fps_max, op::String(FLAGS_prototxt_path),
        op::String(FLAGS_caffemodel_path), (float)FLAGS_upsampling_ratio, enableGoogleLogging};
    opWrapper.configure(wrapperStructPose);

    opWrapper.configure(wrapperStructPose);
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

