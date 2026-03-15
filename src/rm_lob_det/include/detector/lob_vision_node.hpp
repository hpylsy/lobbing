#ifndef LOB_VISION_NODE_HPP_
#define LOB_VISION_NODE_HPP_

#include <memory>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <image_transport/image_transport.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/string.hpp> // 确保包含了 String 消息类型
#include "detector/lob_tracker.hpp"
#include "detector/guide_light_detector.hpp"
#include <std_msgs/msg/string.hpp>

namespace rm_auto_aim {

class LobVisionNode : public rclcpp::Node {
public:
    explicit LobVisionNode(const rclcpp::NodeOptions& options);
    ~LobVisionNode();

private:
    void imageTeleCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
    void handleImage(const cv::Mat& frame);
    void videoTimerCallback();

    void initializeParameters();
    void initializeVideoSource();

    rcl_interfaces::msg::SetParametersResult onParametersSet(const std::vector<rclcpp::Parameter>& parameters);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    std::unique_ptr<LobTracker> lob_tracker_;
    std::unique_ptr<GuideLightDetector> guide_light_detector_; 

    TrackerParams tracker_params_;
    GuideLightDetector::LightParams light_params_;
    
    bool enable_motion_detection_;
    bool show_fused_window_;
    bool enable_source_alarm_;
    bool is_lob_task_ = true;

    image_transport::Subscriber img_tele_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr source_alarm_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr shared_img_pub_;  
    // 【新增】：专门用于 rqt 调试的图像话题
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr binary_img_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fused_img_pub_;  
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr candidates_img_pub_; // <--- 新增这行

    std::string video_source_type_;
    std::string video_path_;
    double video_fps_;
    cv::VideoCapture video_cap_;
    rclcpp::TimerBase::SharedPtr video_timer_;

        // 【新增】：任务模式订阅者和状态变量
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
    std::string current_task_mode_ = "lob"; // 默认随便设一个，等电控下发
    
    void taskCallback(const std_msgs::msg::String::SharedPtr task_msg);
};

}  // namespace rm_auto_aim

#endif  // LOB_VISION_NODE_HPP_