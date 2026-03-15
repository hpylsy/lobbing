#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>

#include "detector/lob_vision_node.hpp"

namespace rm_auto_aim {

LobVisionNode::LobVisionNode(const rclcpp::NodeOptions& options)
    : Node("lob_vision_node", options) {
    RCLCPP_INFO(this->get_logger(), "Starting Lob Vision Node!");

    initializeParameters();

    lob_tracker_ = std::make_unique<LobTracker>();
    lob_tracker_->setParameters(tracker_params_);

    guide_light_detector_ = std::make_unique<GuideLightDetector>(light_params_);

    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&LobVisionNode::onParametersSet, this, std::placeholders::_1));

    binary_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("lob/light_binary", rclcpp::SensorDataQoS());
    fused_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("lob/fused_image", rclcpp::SensorDataQoS());
    // 【新增】：初始化候选图的话题
    candidates_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("lob/filtered_candidates", rclcpp::SensorDataQoS());

    // 【新增】：订阅来自 rm_serial_driver 的任务模式
    task_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/task_mode", 10,
        std::bind(&LobVisionNode::taskCallback, this, std::placeholders::_1));

    initializeVideoSource();
    shared_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "shared_video_raw", rclcpp::SensorDataQoS()); 
    if (enable_source_alarm_) {
        source_alarm_pub_ = this->create_publisher<std_msgs::msg::Bool>("source_alarm", 10);
    }
}

void LobVisionNode::initializeParameters() {
    this->declare_parameter("binary_threshold", 25);
    this->declare_parameter("morph_kernel_size", 5);
    this->declare_parameter("min_area", 20.0);
    this->declare_parameter("max_area", 200.0);
    this->declare_parameter("min_circularity", 0.3);
    this->declare_parameter("min_aspect_ratio", 0.5);
    this->declare_parameter("max_aspect_ratio", 2.0);
    this->declare_parameter("tracklet_confirm_age", 3);
    this->declare_parameter("tracklet_min_down_speed", 2.0);
    this->declare_parameter("max_prediction_error", 50.0);
    this->declare_parameter("max_lost_frames", 5);
    this->declare_parameter("max_trajectory_len", 30);
    this->declare_parameter("max_finished_track_age", 10);
    this->declare_parameter("reid_threshold", 30.0);
    this->declare_parameter("max_angle_change", 45.0);
    this->declare_parameter("min_points_for_smooth_check", 5);
    this->declare_parameter("min_trajectory_length_ratio", 0.05);

    // 【修改点 1】：声明三个独立的窗口开关
    this->declare_parameter("show_fused_window", true);
    this->declare_parameter("show_lob_debug", false);
    light_params_.debug = this->declare_parameter("green.debug", true);

    this->declare_parameter("enable_motion_detection", true);
    this->declare_parameter("video_source", "video");
    this->declare_parameter("video_path", "");
    this->declare_parameter("video_fps", 120.0);
    this->declare_parameter("enable_source_alarm", true);

    light_params_.binaryThreshold = this->declare_parameter("green.binaryThreshold", 60);
    light_params_.epsilon = this->declare_parameter("green.epsilon", 100);
    light_params_.contoursRatioMin = this->declare_parameter("green.contoursRatioMin", 50);
    light_params_.contoursRatioMax = this->declare_parameter("green.contoursRatioMax", 150);
    light_params_.contoursAreaMin = this->declare_parameter("green.contoursAreaMin", 100);
    light_params_.contoursAreaMax = this->declare_parameter("green.contoursAreaMax", 20000);
    light_params_.filledRatioMin = this->declare_parameter("green.filledRatioMin", 50);
    light_params_.filledRatioMax = this->declare_parameter("green.filledRatioMax", 100);
    light_params_.ellipseAreaMin = this->declare_parameter("green.ellipseAreaMin", 100);
    light_params_.ellipseAreaMax = this->declare_parameter("green.ellipseAreaMax", 20000);
    light_params_.ellipseRatioMin = this->declare_parameter("green.ellipseRatioMin", 50);
    light_params_.ellipseRatioMax = this->declare_parameter("green.ellipseRatioMax", 150);

    tracker_params_.binary_threshold = this->get_parameter("binary_threshold").as_int();
    tracker_params_.morph_kernel_size = this->get_parameter("morph_kernel_size").as_int();
    tracker_params_.min_area = this->get_parameter("min_area").as_double();
    tracker_params_.max_area = this->get_parameter("max_area").as_double();
    tracker_params_.min_circularity = this->get_parameter("min_circularity").as_double();
    tracker_params_.min_aspect_ratio = this->get_parameter("min_aspect_ratio").as_double();
    tracker_params_.max_aspect_ratio = this->get_parameter("max_aspect_ratio").as_double();
    tracker_params_.tracklet_confirm_age = this->get_parameter("tracklet_confirm_age").as_int();
    tracker_params_.tracklet_min_down_speed = this->get_parameter("tracklet_min_down_speed").as_double();
    tracker_params_.max_prediction_error = this->get_parameter("max_prediction_error").as_double();
    tracker_params_.max_lost_frames = this->get_parameter("max_lost_frames").as_int();
    tracker_params_.max_trajectory_len = this->get_parameter("max_trajectory_len").as_int();
    tracker_params_.max_finished_track_age = this->get_parameter("max_finished_track_age").as_int();
    tracker_params_.reid_threshold = this->get_parameter("reid_threshold").as_double();
    tracker_params_.max_angle_change = this->get_parameter("max_angle_change").as_double();
    tracker_params_.min_points_for_smooth_check = this->get_parameter("min_points_for_smooth_check").as_int();
    tracker_params_.min_trajectory_length_ratio = this->get_parameter("min_trajectory_length_ratio").as_double();

    // 【修改点 2】：读取这三个开关的值（注意如果你在 .hpp 里没把 show_debug_windows_ 改名，就继续用原来的变量名存 show_fused_window）
    show_fused_window_ = this->get_parameter("show_fused_window").as_bool();
    tracker_params_.show_debug = this->get_parameter("show_lob_debug").as_bool();
    
    enable_motion_detection_ = this->get_parameter("enable_motion_detection").as_bool();
    video_source_type_ = this->get_parameter("video_source").as_string();
    video_path_ = this->get_parameter("video_path").as_string();
    video_fps_ = this->get_parameter("video_fps").as_double();
    enable_source_alarm_ = this->get_parameter("enable_source_alarm").as_bool();
}

rcl_interfaces::msg::SetParametersResult LobVisionNode::onParametersSet(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto& param : parameters) {
        if (param.get_name() == "binary_threshold") tracker_params_.binary_threshold = param.as_int();
        else if (param.get_name() == "max_prediction_error") tracker_params_.max_prediction_error = param.as_double();
        else if (param.get_name() == "reid_threshold") tracker_params_.reid_threshold = param.as_double();
        else if (param.get_name() == "green.binaryThreshold") light_params_.binaryThreshold = param.as_int();
        
        // 【新增】：动态监听绝对位移占比过滤参数！
        else if (param.get_name() == "min_trajectory_length_ratio") {
            tracker_params_.min_trajectory_length_ratio = param.as_double();
        }

        // 【修改点 3】：动态监听这三个开关
        else if (param.get_name() == "show_fused_window") show_fused_window_ = param.as_bool();
        else if (param.get_name() == "show_lob_debug") tracker_params_.show_debug = param.as_bool();
        else if (param.get_name() == "green.debug") light_params_.debug = param.as_bool();
    }

    cv::destroyAllWindows(); // 一旦开关状态改变，立刻清理多余的旧窗口

    if (lob_tracker_) lob_tracker_->setParameters(tracker_params_);
    if (guide_light_detector_) guide_light_detector_->setParameters(light_params_);
    
    return result;
}

void LobVisionNode::taskCallback(const std_msgs::msg::String::SharedPtr task_msg)
{
  std::string task_mode = task_msg->data;
  
  // 只有当任务模式是 "lob" (吊射) 时，才将标志位置为 true
  if (task_mode == "lob") {
    is_lob_task_ = true;
  } else {
    // 收到 "aim"、"base" 或其他任何指令，统统关闭吊射计算
    is_lob_task_ = false;
  }
}

void LobVisionNode::initializeVideoSource() {
    if (video_source_type_ == "camera") {
        
        // 【核心修复】：直接使用 create_subscription，彻底绕开 ImageTransport 对象的指针生命周期坑！
        // 并且把订阅话题改成了 hik_camera 默认的 "/image_raw"
        img_tele_sub_ = image_transport::create_subscription(
            this, 
            "/image_raw",  // <--- 【关键】去掉了 /camera 前缀
            std::bind(&LobVisionNode::imageTeleCallback, this, std::placeholders::_1),
            "raw", 
            rmw_qos_profile_sensor_data);
            
    } else {
        video_cap_.open(video_path_);
        int timer_period_ms = static_cast<int>(1000.0 / video_fps_);
        video_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms),
            std::bind(&LobVisionNode::videoTimerCallback, this));
    }
}

void LobVisionNode::imageTeleCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    try {
        cv::Mat frame = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;
        handleImage(frame);
    } catch (const cv_bridge::Exception& e) {}
}

void LobVisionNode::videoTimerCallback() {
    cv::Mat frame;
    if (video_cap_.read(frame)) {
        handleImage(frame); 
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_optical_frame"; 
        auto img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        shared_img_pub_->publish(*img_msg);
    } else {
        video_cap_.set(cv::CAP_PROP_POS_FRAMES, 0); 
    }
}

void LobVisionNode::handleImage(const cv::Mat& frame) {
    if (!is_lob_task_) {
        return; 
    }

    if (frame.empty()) return;
    
    if (current_task_mode_ != "lob" && !show_fused_window_) {
        return; 
    }
    // 1. 调用原版的绿灯检测算法
    auto lights = guide_light_detector_->detect(frame);
    
    // 2. 【核心修改】：根据引导灯尺寸，等比例推算装甲板的物理范围
    std::vector<cv::Rect2f> target_armors;
    for (const auto& l : lights) {
        float light_diameter = l.rect.size.width; // 因为我们在底层改成了圆，宽和高就是直径
        float armor_size = light_diameter * 3.5f; // 假设装甲板是引导灯直径的 3.5 倍大小
        
        // 计算装甲板的左上角坐标，使其与引导灯同心
        cv::Point2f top_left(l.center.x - armor_size / 2.0f, l.center.y - armor_size / 2.0f);
        target_armors.emplace_back(top_left.x, top_left.y, armor_size, armor_size);
    }

   // 3. 执行弹丸检测 (把刚才算出来的 target_armors 传进去)
    cv::Mat final_vis;
    if (enable_motion_detection_ && lob_tracker_) {
        final_vis = lob_tracker_->track(frame, target_armors);

        // 【新增】：提取并发布候选框过滤画面 (只有在开启 show_lob_debug 时才执行)
        if (tracker_params_.show_debug) {
            cv::Mat filter_vis = lob_tracker_->getFilterVis();
            if (!filter_vis.empty()) {
                std_msgs::msg::Header header;
                header.stamp = this->now();
                header.frame_id = "camera_optical_frame";
                // 这是带彩框的图，所以用 bgr8
                auto filter_msg = cv_bridge::CvImage(header, "bgr8", filter_vis).toImageMsg();
                candidates_img_pub_->publish(*filter_msg);
            }
        }
    } else {
        final_vis = frame.clone();
    }

    // 4. 进行绘制
    guide_light_detector_->drawResults(final_vis);

    if (show_fused_window_) {
        //cv::imshow("Lob Vision System (Fused)", final_vis);
        cv::waitKey(1);
    }
}

LobVisionNode::~LobVisionNode() {
    if (video_cap_.isOpened()) video_cap_.release();
    cv::destroyAllWindows();
}

} // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::LobVisionNode)