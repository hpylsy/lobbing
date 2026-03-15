#ifndef LOB_TRACKER_HPP_
#define LOB_TRACKER_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <deque>
#include <vector>
#include <memory>

namespace rm_auto_aim {

// 1. 更新候选轨迹结构体
struct Tracklet {
    std::deque<cv::Point2f> points;
    int age;
    double avg_speed_y;
    cv::Point2f first_point; // 【新增】：记录轨迹的绝对出生点
    cv::Point2f last_center;
    bool confirmed;
    int lost_count;

    // 构造函数初始化 first_point
    Tracklet(const cv::Point2f& p) : age(1), avg_speed_y(0), first_point(p), last_center(p), confirmed(false), lost_count(0) {
        points.push_back(p);
    }

    void addPoint(const cv::Point2f& p) {
        points.push_back(p);
        if (points.size() > 10) points.pop_front(); // 稍微增加一点队列长度
        last_center = p;
        age++;
        if (points.size() >= 2) {
            double sum_dy = 0;
            for (size_t i = 1; i < points.size(); i++) {
                sum_dy += (points[i].y - points[i-1].y);
            }
            avg_speed_y = sum_dy / (points.size() - 1);
        }
    }

    bool isDownward(double min_speed = 2.0) const {
        return avg_speed_y > min_speed;
    }

    // 【新增】：计算当前点到出生点的绝对物理位移
    double getDisplacement() const {
        return cv::norm(last_center - first_point);
    }
};

struct FinishedTrack {
    cv::KalmanFilter kf;
    cv::Mat_<float> state;
    int last_frame_id;
    FinishedTrack(const cv::KalmanFilter& kf_, const cv::Mat_<float>& state_, int frame)
        : kf(kf_), state(state_), last_frame_id(frame) {}
};

// 2. 更新参数结构体
struct TrackerParams {
    int binary_threshold = 20;
    int morph_kernel_size = 5;
    double min_area = 20.0;
    double max_area = 500.0;
    double min_circularity = 0.3;
    double min_aspect_ratio = 0.5;
    double max_aspect_ratio = 2.0;
    
    double max_prediction_error = 50.0;
    int tracklet_confirm_age = 3;
    double tracklet_min_down_speed = 2.0;
    
    // 【新增】：轨迹生效的最短位移比例（默认 0.05 代表必须移动画面高度的 5%）
    double min_trajectory_length_ratio = 0.05; 
    
    int max_finished_track_age = 10;
    double reid_threshold = 30.0;
    
    double max_angle_change = 45.0; 
    int min_points_for_smooth_check = 5;
    int max_lost_frames = 5;
    int max_trajectory_len = 30;
    
    bool show_debug = false; 
};

class LobTracker {
public:
    LobTracker();
    void setParameters(const TrackerParams& params);
    // 【核心修改】：接收装甲板矩形，而不是单一的中心点
    cv::Mat track(const cv::Mat& frame, const std::vector<cv::Rect2f>& target_armors = {});
    cv::Mat getFilterVis() const { return filter_vis_; }

private:
    void initKalman(const cv::Point2f& initPos);

    TrackerParams params_;
    
    cv::Mat background_;
    cv::Mat background_gray_;
    std::vector<cv::Point2f> prev_pts_;
    
    int frame_id_;
    cv::Point2f prev_center_;
    bool has_prev_;

    cv::KalmanFilter kf_;
    cv::Mat_<float> state_;
    cv::Mat_<float> meas_;
    cv::Mat filter_vis_;
    bool kalman_initialized_;

    std::deque<cv::Point2f> current_trajectory_;
    bool trajectory_active_;
    int lost_frames_;

    std::vector<Tracklet> tracklets_;
    std::vector<FinishedTrack> finished_tracks_;

    // 【新增】：幽灵落点标记结构体和队列
    struct ImpactMark {
        cv::Point2f pos;
        int age; 
    };
    // 【新增】：死亡轨迹保留队列
    struct DeadTrajectory {
        std::deque<cv::Point2f> points;
        int age;
    };
    std::vector<DeadTrajectory> dead_trajectories_;
    std::vector<ImpactMark> impact_marks_;
    bool current_lob_marked_ = false; // 防止同一发飞镖被记录多次
};

} // namespace rm_auto_aim

#endif // LOB_TRACKER_HPP_