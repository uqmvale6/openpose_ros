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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <kinect_streamer/kinect_streamer.hpp>

#include <fstream>

#include <chrono>

#define HEAD    0
#define CHST    1
#define SHDR_R  2
#define ELBW_R  3
#define WRST_R  4
#define SHDR_L  5
#define ELBW_L  6
#define WRST_L  7



cv::Mat img_color;
cv::Mat img_depth;
cv::Mat img_draw;
bool flag_color = false;
bool flag_depth = false;
int last_row = -1;
int last_col = -1;
//std::string base_frame = "kinect2_ir_optical_frame";
std::string base_frame = "base";

void image_color_callback(const sensor_msgs::ImageConstPtr& msg) {
    (cv_bridge::toCvShare(msg, "bgr8")->image).copyTo(img_color);
    flag_color = true;
}


void image_depth_callback(const sensor_msgs::ImageConstPtr& msg) {
    (cv_bridge::toCvShare(msg, "16UC1")->image).copyTo(img_depth);
    flag_depth = true;
}



struct OpenPoseROSArgs : public argparse::Args {
    std::string &image_color_topic = arg("Image Color Topic");
    std::string &image_depth_topic = arg("Image Depth Topic");
    std::string &serial = arg("Serial");
    bool& compressed = kwarg("c,compressed", "Compressed Image Topic?", "true").set_default("false");
};


OpenPoseROS::OpenPoseROS(int argc, char** argv) {
    OpenPoseROSArgs args = argparse::parse<OpenPoseROSArgs>(argc, argv);
    image_color_topic = args.image_color_topic;
    image_depth_topic = args.image_depth_topic;
    serial = args.serial;
    compressed = args.compressed;
    args.print();
    ros::init(argc, argv, "openpose_node");
}


bool OpenPoseROS::ok() {
    return ros::ok();
}


void get_point(const float row, const float col, const float depth, float& x, float& y, float& z) {

    const float cx = 256.7421875; //ir_params.cx;
    const float cy = 203.66299438476562; //ir_params.cy;
    const float fx = 1 / 366.2200012207031; //1 / ir_params.fx;
    const float fy = 1 / 366.2200012207031; //1 / ir_params.fy;

    const float depth_val = depth / 1000.0f;

    if (!isnan(depth_val) && depth_val > 0.001) {

        x = -(512-col + 0.5 - cx) * fx * depth_val;
        y = (row + 0.5 - cy) * fy * depth_val;
        z = depth_val;
    } else {

        x = 0;
        y = 0;
        z = 0;
    }
}

void print_xyz(float x, float y, float z) {
    printf("(%f, %f, %f)\n\r", x, y, z);
}


geometry_msgs::Point p_prev_head;
geometry_msgs::Point p_prev_wrist_l;
geometry_msgs::Point p_prev_wrist_r;
geometry_msgs::Point p_prev_elbow_l;
geometry_msgs::Point p_prev_elbow_r;
geometry_msgs::Point p_prev_shoulder_l;
geometry_msgs::Point p_prev_shoulder_r;
geometry_msgs::Point p_prev_chest;
bool flag_init = false;
float diff_x = 0;
float diff_y = 0;
float diff_z = 0;
float threshold = 0.8;

void OpenPoseROS::loop() {
    ros::spinOnce();
    if (img_color.data == NULL || img_depth.data == NULL) {
        return;
    }
    cv::Mat img_thresh1;
    cv::Mat img_thresh2;
    cv::Mat img_thresh;
    
    cv::threshold(img_depth, img_thresh1, 2000, 65535, cv::THRESH_BINARY_INV);
    cv::threshold(img_depth, img_thresh2, 1, 65535, cv::THRESH_BINARY);
    cv::bitwise_and(img_thresh1, img_thresh2, img_thresh);
    cv::Mat mask;
    img_thresh.convertTo(mask, CV_8UC1);

    cv::Mat img_seg;
    cv::copyTo(img_color, img_seg, mask);
    
    const cv::Mat cvImageToProcess = img_color;

    const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(cvImageToProcess);
    auto datumProcessed = opWrapper.emplaceAndPop(imageToProcess);

    cvMat = OP_OP2CVCONSTMAT(datumProcessed->at(0)->cvOutputData);
    poseKeypoints = datumProcessed->at(0)->poseKeypoints;


    float row_arr[8];
    float col_arr[8];
    float depth_arr[8];
    float x_arr[8];
    float y_arr[8];
    float z_arr[8];

    const auto numberPeopleDetected = poseKeypoints.getSize(0);
    const auto numberBodyParts = poseKeypoints.getSize(1);
    if (numberPeopleDetected > 0 && numberBodyParts > 0) {
        for (int i = 0; i <= 7; i++) {
            row_arr[i] = poseKeypoints[{0, i, 1}];
            col_arr[i] = poseKeypoints[{0, i, 0}];
            int row = (int)row_arr[i];
            int col = (int)col_arr[i];
            uint16_t depth = img_depth.at<uint16_t>(row, col);
            depth_arr[i] = depth;
        }
        for (int i = 0; i <= 7; i++) {
            float x;
            float y;
            float z;
            get_point(row_arr[i], col_arr[i], depth_arr[i], x_arr[i], y_arr[i], z_arr[i]);
            if (poseKeypoints[{0, i, 2}] <= 0.3) {
                x_arr[i] = 0;
                y_arr[i] = 0;
                z_arr[i] = 0;
            }
        }
        
        ros::Time time_ros = ros::Time::now();

        geometry_msgs::Point p_head;
        p_head.x = x_arr[HEAD];
        p_head.y = y_arr[HEAD];
        p_head.z = z_arr[HEAD];
        geometry_msgs::PointStamped ps_head;
        ps_head.header.frame_id = "kinect2_ir_optical_frame";
        ps_head.header.stamp = time_ros;
        ps_head.point = p_head;
        if (ps_head.point.x != 0 || ps_head.point.y != 0 || ps_head.point.z != 0) {
            pub_pointstamped_head.publish(ps_head);
        }
        
        geometry_msgs::Point p_wrist_l;
        p_wrist_l.x = x_arr[WRST_L];
        p_wrist_l.y = y_arr[WRST_L];
        p_wrist_l.z = z_arr[WRST_L];
        geometry_msgs::PointStamped ps_wrist_l;
        ps_wrist_l.header.frame_id = base_frame;
        ps_wrist_l.header.stamp = time_ros;
        ps_wrist_l.point = p_wrist_l;
        if (ps_wrist_l.point.x != 0 || ps_wrist_l.point.y != 0 || ps_wrist_l.point.z != 0) {
            pub_pointstamped_wrist_l.publish(ps_wrist_l);
        }

        

        geometry_msgs::Point p_elbow_l;
        p_elbow_l.x = x_arr[ELBW_L];
        p_elbow_l.y = y_arr[ELBW_L];
        p_elbow_l.z = z_arr[ELBW_L];
        geometry_msgs::PointStamped ps_elbow_l;
        ps_elbow_l.header.frame_id = base_frame;
        ps_elbow_l.header.stamp = time_ros;
        ps_elbow_l.point = p_elbow_l;
        if (ps_elbow_l.point.x != 0 || ps_elbow_l.point.y != 0 || ps_elbow_l.point.z != 0) {
            pub_pointstamped_elbow_l.publish(ps_elbow_l);
        }


        geometry_msgs::Point p_shoulder_l;
        p_shoulder_l.x = x_arr[SHDR_L];
        p_shoulder_l.y = y_arr[SHDR_L];
        p_shoulder_l.z = z_arr[SHDR_L];
        geometry_msgs::PointStamped ps_shoulder_l;
        ps_shoulder_l.header.frame_id = base_frame;
        ps_shoulder_l.header.stamp = time_ros;
        ps_shoulder_l.point = p_shoulder_l;
        if (ps_shoulder_l.point.x != 0 || ps_shoulder_l.point.y != 0 || ps_shoulder_l.point.z != 0) {
            pub_pointstamped_shoulder_l.publish(ps_shoulder_l);
        }

        geometry_msgs::Point p_wrist_r;
        p_wrist_r.x = x_arr[WRST_R];
        p_wrist_r.y = y_arr[WRST_R];
        p_wrist_r.z = z_arr[WRST_R];
        geometry_msgs::PointStamped ps_wrist_r;
        ps_wrist_r.header.frame_id = base_frame;
        ps_wrist_r.header.stamp = time_ros;
        ps_wrist_r.point = p_wrist_r;
        if (ps_wrist_r.point.x != 0 || ps_wrist_r.point.y != 0 || ps_wrist_r.point.z != 0) {
            pub_pointstamped_wrist_r.publish(ps_wrist_r);
        }

        geometry_msgs::Point p_elbow_r;
        p_elbow_r.x = x_arr[ELBW_R];
        p_elbow_r.y = y_arr[ELBW_R];
        p_elbow_r.z = z_arr[ELBW_R];
        geometry_msgs::PointStamped ps_elbow_r;
        ps_elbow_r.header.frame_id = base_frame;
        ps_elbow_r.header.stamp = time_ros;
        ps_elbow_r.point = p_elbow_r;
        if (ps_elbow_r.point.x != 0 || ps_elbow_r.point.y != 0 || ps_elbow_r.point.z != 0) {
            pub_pointstamped_elbow_r.publish(ps_elbow_r);
        }


        geometry_msgs::Point p_shoulder_r;
        p_shoulder_r.x = x_arr[SHDR_R];
        p_shoulder_r.y = y_arr[SHDR_R];
        p_shoulder_r.z = z_arr[SHDR_R];
        geometry_msgs::PointStamped ps_shoulder_r;
        ps_shoulder_r.header.frame_id = base_frame;
        ps_shoulder_r.header.stamp = time_ros;
        ps_shoulder_r.point = p_shoulder_r;
        if (ps_shoulder_r.point.x != 0 || ps_shoulder_r.point.y != 0 || ps_shoulder_r.point.z != 0) {
            pub_pointstamped_shoulder_r.publish(ps_shoulder_r);
        }


        geometry_msgs::Point p_chest;
        p_chest.x = x_arr[CHST];
        p_chest.y = y_arr[CHST];
        p_chest.z = z_arr[CHST];
        geometry_msgs::PointStamped ps_chest;
        ps_chest.header.frame_id = base_frame;
        ps_chest.header.stamp = time_ros;
        ps_chest.point = p_chest;
        if (ps_chest.point.x != 0 || ps_chest.point.y != 0 || ps_chest.point.z != 0) {
                pub_pointstamped_chest.publish(ps_chest);
        }

    }
    cv::imshow("OpenPose", cvMat);
    char c = cv::waitKey(1);
    flag_color = false;
    flag_depth = false;
    
}

void OpenPoseROS::start() {
    opWrapper.start();
}

void OpenPoseROS::init() {
    
    const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
    const auto netInputSize = op::flagsToPoint(op::String("-1x320"), "-1x320");
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
    ros::NodeHandle nh_pointstamped_head = ros::NodeHandle();
    
    ros::NodeHandle nh_pointstamped_wrist_l = ros::NodeHandle();
    ros::NodeHandle nh_pointstamped_elbow_l = ros::NodeHandle();
    ros::NodeHandle nh_pointstamped_shoulder_l = ros::NodeHandle();

    ros::NodeHandle nh_pointstamped_wrist_r = ros::NodeHandle();
    ros::NodeHandle nh_pointstamped_elbow_r = ros::NodeHandle();
    ros::NodeHandle nh_pointstamped_shoulder_r = ros::NodeHandle();

    ros::NodeHandle nh_pointstamped_chest = ros::NodeHandle();

    ros::NodeHandle nh_image_color = ros::NodeHandle("color_image");
    ros::NodeHandle nh_image_depth = ros::NodeHandle("color_depth");
    it_image_color = std::make_unique<image_transport::ImageTransport>(nh_image_color);
    it_image_depth = std::make_unique<image_transport::ImageTransport>(nh_image_depth);
    image_transport::TransportHints hints_color;
    image_transport::TransportHints hints_depth;
    if (compressed) {
        hints_color = image_transport::TransportHints("compressed");
        hints_depth = image_transport::TransportHints("compressed");
    } else {
        hints_color = image_transport::TransportHints("raw");
        hints_depth = image_transport::TransportHints("raw");
    }
    sub_image_color = it_image_color->subscribe(image_color_topic, 1, image_color_callback, ros::VoidPtr(), hints_color);
    sub_image_depth = it_image_depth->subscribe(image_depth_topic, 1, image_depth_callback, ros::VoidPtr(), hints_depth);

    pub_pointstamped_head = nh_pointstamped_head.advertise<geometry_msgs::PointStamped>("mrn_vision/openpose/body/head", 1);
    
    pub_pointstamped_wrist_l = nh_pointstamped_wrist_l.advertise<geometry_msgs::PointStamped>("mrn_vision/openpose/body/wrist_left", 1);
    pub_pointstamped_elbow_l = nh_pointstamped_elbow_l.advertise<geometry_msgs::PointStamped>("mrn_vision/openpose/body/elbow_left", 1);
    pub_pointstamped_shoulder_l = nh_pointstamped_shoulder_l.advertise<geometry_msgs::PointStamped>("mrn_vision/openpose/body/shoulder_left", 1);

    pub_pointstamped_wrist_r = nh_pointstamped_wrist_r.advertise<geometry_msgs::PointStamped>("mrn_vision/openpose/body/wrist_right", 1);
    pub_pointstamped_elbow_r = nh_pointstamped_elbow_r.advertise<geometry_msgs::PointStamped>("mrn_vision/openpose/body/elbow_right", 1);
    pub_pointstamped_shoulder_r = nh_pointstamped_shoulder_r.advertise<geometry_msgs::PointStamped>("mrn_vision/openpose/body/shoulder_right", 1);

    pub_pointstamped_chest = nh_pointstamped_chest.advertise<geometry_msgs::PointStamped>("mrn_vision/openpose/body/chest", 1);

    img_draw = cv::Mat::zeros(cv::Size(512, 424), CV_8UC3);

}
