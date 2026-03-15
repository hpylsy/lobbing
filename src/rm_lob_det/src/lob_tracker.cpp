#include "detector/lob_tracker.hpp"

namespace rm_auto_aim {

LobTracker::LobTracker() 
    : frame_id_(0), prev_center_(-1, -1), has_prev_(false), 
      state_(4, 1), meas_(2, 1), kalman_initialized_(false), 
      trajectory_active_(false), lost_frames_(0) {
}

void LobTracker::setParameters(const TrackerParams& params) {
    params_ = params;
}

void LobTracker::initKalman(const cv::Point2f& initPos) {
    kf_.init(4, 2, 0, CV_32F);
    cv::setIdentity(kf_.transitionMatrix);
    kf_.transitionMatrix.at<float>(0, 2) = 1;
    kf_.transitionMatrix.at<float>(1, 3) = 1;
    cv::setIdentity(kf_.measurementMatrix);
    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-3));
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-1));
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1));
    state_ = cv::Mat_<float>::zeros(4, 1);
    state_.at<float>(0) = initPos.x;
    state_.at<float>(1) = initPos.y;
    kf_.statePost = state_;
    kalman_initialized_ = true;
}

cv::Mat LobTracker::track(const cv::Mat& frame, const std::vector<cv::Rect2f>& target_armors) {
    if (frame.empty()) return frame;
    frame_id_++;
    cv::Mat result = frame.clone();
    cv::Mat frame_gray;
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

    if (background_.empty()) {
        background_ = frame.clone();
        background_gray_ = frame_gray.clone();
        cv::goodFeaturesToTrack(background_gray_, prev_pts_, 200, 0.01, 10);
        return frame;
    }

    if (prev_pts_.empty()) {
        cv::goodFeaturesToTrack(frame_gray, prev_pts_, 200, 0.01, 10);
        return frame;
    }

    std::vector<cv::Point2f> next_pts;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(background_gray_, frame_gray, prev_pts_, next_pts, status, err);

    std::vector<cv::Point2f> prev_good, next_good;
    for (size_t i = 0; i < status.size(); i++) {
        if (status[i]) {
            prev_good.push_back(prev_pts_[i]);
            next_good.push_back(next_pts[i]);
        }
    }

    if (prev_good.size() < 4) {
        cv::goodFeaturesToTrack(frame_gray, prev_pts_, 200, 0.01, 10);
        return frame;
    }

    cv::Mat H = cv::findHomography(prev_good, next_good, cv::RANSAC);
    if (H.empty()) return frame;

    // ==========================================
    // 【弹丸探针 1：光流法追踪】
    // ==========================================
    if (params_.show_debug) {
        cv::Mat flow_vis = frame.clone();
        for (size_t i = 0; i < prev_good.size(); i++) {
            cv::circle(flow_vis, next_good[i], 3, cv::Scalar(0, 255, 0), -1);
            cv::line(flow_vis, prev_good[i], next_good[i], cv::Scalar(0, 0, 255), 2);
        }
        //cv::imshow("Lob_1_Optical_Flow", flow_vis);
    }

    cv::Mat aligned, diff, thresholded;
    cv::warpPerspective(frame, aligned, H, background_.size());
    if (aligned.empty()) return frame;

    cv::Mat aligned_gray;
    cv::cvtColor(aligned, aligned_gray, cv::COLOR_BGR2GRAY);
    
    // 关键修复：方向性相减避免双重残影
    cv::subtract(aligned_gray, background_gray_, diff);

    // ==========================================
    // 【弹丸探针 2：对齐与纯差分图】
    // ==========================================
    if (params_.show_debug) {
        //cv::imshow("Lob_2_Aligned", aligned);
        cv::Mat diff_vis;
        diff.convertTo(diff_vis, -1, 2.0); // 亮度 x2，方便肉眼看暗部
        //cv::imshow("Lob_3_Abs_Diff", diff_vis);
    }

    cv::threshold(diff, thresholded, params_.binary_threshold, 255, cv::THRESH_BINARY);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(params_.morph_kernel_size, params_.morph_kernel_size));
    cv::morphologyEx(thresholded, thresholded, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(thresholded, thresholded, cv::MORPH_CLOSE, kernel);

    // ==========================================
    // 【弹丸探针 3：二值化形态学】
    // ==========================================
    if (params_.show_debug) {
        //cv::imshow("Lob_4_Threshold_Morph", thresholded);
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresholded, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    struct Candidate { int index; cv::Point2f center; double area; };
    std::vector<Candidate> candidates;

    // ==========================================
    // 【弹丸探针 4：筛选淘汰过程 (仅作图像保存)】
    // ==========================================
    if (params_.show_debug) {
        filter_vis_ = frame.clone(); // 开启调试时，克隆原图准备画框
    } else {
        filter_vis_ = cv::Mat(); // 关闭调试时清空，不占内存
    }

    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        cv::Rect bound = cv::boundingRect(contours[i]);
        
        if (area < params_.min_area || area > params_.max_area) {
            if (params_.show_debug) cv::rectangle(filter_vis_, bound, cv::Scalar(0, 0, 255), 1); // 面积淘汰画红框
            continue;
        }

        double perimeter = cv::arcLength(contours[i], true);
        double circularity = (perimeter > 0) ? (4 * CV_PI * area / (perimeter * perimeter)) : 0;
        if (circularity < params_.min_circularity) {
            if (params_.show_debug) cv::rectangle(filter_vis_, bound, cv::Scalar(0, 165, 255), 1); // 圆度淘汰画橙框
            continue;
        }

        double aspect_ratio = (double)bound.width / bound.height;
        if (aspect_ratio < params_.min_aspect_ratio || aspect_ratio > params_.max_aspect_ratio) {
            if (params_.show_debug) cv::rectangle(filter_vis_, bound, cv::Scalar(255, 0, 255), 1); // 长宽比淘汰画紫框
            continue;
        }

        cv::Point2f center(bound.x + bound.width/2.0, bound.y + bound.height/2.0);

        if (params_.show_debug) {
            cv::rectangle(filter_vis_, bound, cv::Scalar(0, 255, 0), 2); // 活下来的 Candidate 画绿框
        }

        candidates.push_back({static_cast<int>(i), center, area});
    }

    // ==========================================
    // 候选帧匹配与方向角度过滤
    // ==========================================
    cv::Point2f detected_center(-1, -1);
    bool detected = false;

    if (!candidates.empty()) {
        int best_idx = -1;
        if (!has_prev_) {
            auto max_it = std::max_element(candidates.begin(), candidates.end(),
                                           [](const Candidate& a, const Candidate& b) { return a.area < b.area; });
            best_idx = max_it - candidates.begin();
            detected_center = candidates[best_idx].center;
            detected = true;
            has_prev_ = true;
            prev_center_ = detected_center;
        } else {
            const double min_shift = 5.0;
            const double max_shift = background_.cols * 0.3;
            const double max_angle = 50.0 * CV_PI / 180.0;

            double min_dist = 1e9;
            for (size_t k = 0; k < candidates.size(); ++k) {
                cv::Point2f delta = candidates[k].center - prev_center_;
                double shift = cv::norm(delta);
                if (shift < min_shift || shift > max_shift) continue;
                if (delta.y <= 0) continue;
                double angle = std::atan2(std::abs(delta.x), delta.y);
                if (angle > max_angle) continue;

                if (shift < min_dist) {
                    min_dist = shift;
                    best_idx = static_cast<int>(k);
                }
            }
            if (best_idx != -1) {
                detected_center = candidates[best_idx].center;
                detected = true;
                prev_center_ = detected_center;
            }
        }
    } else {
        has_prev_ = false;
    }

    std::vector<char> matched_tracklets(tracklets_.size(), 0);

    if (detected) {
        bool reconnected = false;
    // ========== 将合格的候选轨迹转正 (加入画面占比过滤与 Debug 提示) ==========
        for (auto it = tracklets_.begin(); it != tracklets_.end(); ) {
            
            double min_required_disp = frame.rows * params_.min_trajectory_length_ratio;
            
            // 拆解三大转正条件
            bool age_ok = it->age >= params_.tracklet_confirm_age;
            bool dir_ok = it->isDownward(params_.tracklet_min_down_speed);
            bool disp_ok = it->getDisplacement() >= min_required_disp;

            if (age_ok && dir_ok && disp_ok && !it->confirmed) {
                // 条件全部满足，正式转正激活卡尔曼滤波！
                if (!trajectory_active_) {
                    initKalman(it->last_center);
                    trajectory_active_ = true;
                    current_lob_marked_ = false; // (如果改名了这里是 current_lob_marked_)
                    // 【新增】：在清空前，把当前轨迹存入“死亡队列”
                if (!current_trajectory_.empty()) {
                    dead_trajectories_.push_back({current_trajectory_, 0});
                }
                    current_trajectory_.clear();
                    current_trajectory_.push_back(it->last_center);
                    it->confirmed = true;
                    ++it;
                } else {
                    ++it;
                }
            } else if (!it->confirmed) {
                // 【核心新增】：在屏幕上标识未转正的“死因” (仅在打开调试开关时显示)
                if (params_.show_debug) {
                    std::string reason;
                    if (!age_ok) {
                        reason = "Wait Age: " + std::to_string(it->age) + "/" + std::to_string(params_.tracklet_confirm_age);
                    } else if (!dir_ok) {
                        reason = "Not Downward";
                    } else if (!disp_ok) {
                        reason = "Short Disp";
                    }
                    
                    // 将死因用橙色小字画在候选点的右侧
                    cv::putText(result, reason, cv::Point(it->last_center.x + 15, it->last_center.y),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 165, 255), 2);
                }
                ++it;
            } else {
                ++it;
            }
        }

        if (!reconnected) {
            double best_match_dist = 1e9;
            int best_tracklet_idx = -1;
            for (size_t t = 0; t < tracklets_.size(); t++) {
                double dist = cv::norm(detected_center - tracklets_[t].last_center);
                // 使用宽容的ReID逻辑
                if (dist < params_.reid_threshold && dist < best_match_dist) {
                    best_match_dist = dist;
                    best_tracklet_idx = t;
                }
            }
            if (best_tracklet_idx != -1) {
                tracklets_[best_tracklet_idx].addPoint(detected_center);
                matched_tracklets[best_tracklet_idx] = 1;
            } else {
                tracklets_.emplace_back(detected_center);
                matched_tracklets.push_back(0); 
            }
        }
    }

    for (size_t t = 0; t < tracklets_.size(); ) {
        if (!matched_tracklets[t]) {
            tracklets_[t].lost_count++;
            if (tracklets_[t].lost_count > 2) {
                tracklets_.erase(tracklets_.begin() + t);
                matched_tracklets.erase(matched_tracklets.begin() + t);
                continue;
            }
        } else {
            tracklets_[t].lost_count = 0;
        }
        t++;
    }

    // ========== 将合格的候选轨迹转正 (加入画面占比过滤) ==========
    for (auto it = tracklets_.begin(); it != tracklets_.end(); ) {
        // 【核心过滤逻辑】：计算当前画面高度的比例要求
        // 假设 frame 垂直分辨率为 1080，ratio 为 0.05，则必须移动超过 54 个像素
        double min_required_disp = frame.rows * params_.min_trajectory_length_ratio;

        if (it->age >= params_.tracklet_confirm_age && 
            it->isDownward(params_.tracklet_min_down_speed) && 
            it->getDisplacement() >= min_required_disp &&  // <--- 【新增】：绝对位移门槛
            !it->confirmed) {
            
            if (!trajectory_active_) {
                initKalman(it->last_center);
                trajectory_active_ = true;
                current_lob_marked_ = false; // 【新增】：新飞镖出现了，解开记录锁
                // 【新增】：在清空前，把当前轨迹存入“死亡队列”
                if (!current_trajectory_.empty()) {
                    dead_trajectories_.push_back({current_trajectory_, 0});
                }
                
                current_trajectory_.clear(); // 原本的清空代码保持不变
                current_trajectory_.push_back(it->last_center);
                it->confirmed = true;
                ++it;
            } else {
                ++it;
            }
        } else {
            ++it;
        }
    }

    tracklets_.erase(std::remove_if(tracklets_.begin(), tracklets_.end(),
                               [](const Tracklet& t) { return t.confirmed; }),
                     tracklets_.end());

    if (trajectory_active_) {
        if (!kalman_initialized_) {
            trajectory_active_ = false;
        } else {
            kf_.predict();
            state_ = kf_.statePost;

            if (detected) {
                cv::Point2f predicted(state_.at<float>(0), state_.at<float>(1));
                double error = cv::norm(detected_center - predicted);
                if (error < params_.max_prediction_error) {
                    meas_.at<float>(0) = detected_center.x;
                    meas_.at<float>(1) = detected_center.y;
                    kf_.correct(meas_);
                    state_ = kf_.statePost;
                    current_trajectory_.push_back(cv::Point2f(state_.at<float>(0), state_.at<float>(1)));
                    lost_frames_ = 0;
                } else { lost_frames_++; }
            } else { lost_frames_++; }

            if (lost_frames_ > params_.max_lost_frames) {
                if (kalman_initialized_) finished_tracks_.emplace_back(kf_, state_, frame_id_);
                trajectory_active_ = false;
                kalman_initialized_ = false;
                // 【新增】：在清空前，把当前轨迹存入“死亡队列”
                    if (!current_trajectory_.empty()) {
                        dead_trajectories_.push_back({current_trajectory_, 0});
                    }
                    
                    current_trajectory_.clear(); // 原本的清空代码保持不变
            } else {
                if (!detected || (detected && cv::norm(detected_center - cv::Point2f(state_.at<float>(0), state_.at<float>(1))) > params_.max_prediction_error)) {
                    cv::Point2f pred_center(state_.at<float>(0), state_.at<float>(1));
                    if (current_trajectory_.empty() || cv::norm(pred_center - current_trajectory_.back()) > 0.1) {
                        current_trajectory_.push_back(pred_center);
                    }
                }
            }

            if (static_cast<int>(current_trajectory_.size()) >= params_.min_points_for_smooth_check) {
                bool is_smooth = true;
                int check_count = std::min(3, (int)current_trajectory_.size() - 1);
                double max_angle_rad = params_.max_angle_change * CV_PI / 180.0;
                int traj_size = static_cast<int>(current_trajectory_.size());
                for (int i = traj_size - 2; i >= traj_size - 1 - check_count && i > 0; i--) {
                    cv::Point2f vec1 = current_trajectory_[i] - current_trajectory_[i-1];
                    cv::Point2f vec2 = current_trajectory_[i+1] - current_trajectory_[i];
                    if (cv::norm(vec1) < 1e-6 || cv::norm(vec2) < 1e-6) continue;
                    double dot = vec1.x * vec2.x + vec1.y * vec2.y;
                    double cos_angle = dot / (cv::norm(vec1) * cv::norm(vec2));
                    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
                    double angle = std::acos(cos_angle);
                    if (angle > max_angle_rad) {
                        is_smooth = false;
                        break;
                    }
                }
                    if (!is_smooth) {
                    // 【触发器 A：命中反弹】
                    if (!current_lob_marked_ && !target_armors.empty()) {
                        impact_marks_.push_back({current_trajectory_.back(), 0});
                        current_lob_marked_ = true;
                    }
                    
                    if (kalman_initialized_) finished_tracks_.emplace_back(kf_, state_, frame_id_);
                    trajectory_active_ = false;
                    kalman_initialized_ = false;
                    // 【新增】：在清空前，把当前轨迹存入“死亡队列”
                            if (!current_trajectory_.empty()) {
                                dead_trajectories_.push_back({current_trajectory_, 0});
                            }
                            
                            current_trajectory_.clear(); // 原本的清空代码保持不变
                }
            }
                    // 【触发器 B：脱靶飞越 (Miss)】
                    if (trajectory_active_ && !current_lob_marked_ && !target_armors.empty()) {
                        // 计算装甲板的水平中心线
                        float target_center_y = target_armors[0].y + target_armors[0].height / 2.0f;
                        // 当飞镖掉落穿过装甲板水平中心线时，立刻截取落点
                        if (current_trajectory_.back().y >= target_center_y) {
                            impact_marks_.push_back({current_trajectory_.back(), 0});
                            current_lob_marked_ = true;
                        }
                    }

            while (static_cast<int>(current_trajectory_.size()) > params_.max_trajectory_len) {
                current_trajectory_.pop_front();
            }
        }
    }

    for (size_t i = 1; i < current_trajectory_.size(); i++) {
        cv::line(result, current_trajectory_[i-1], current_trajectory_[i], cv::Scalar(0, 255, 255), 2);
    }
    
    if (trajectory_active_) {
        cv::Point kf_pos(state_.at<float>(0), state_.at<float>(1));
        cv::circle(result, kf_pos, 5, cv::Scalar(0, 0, 255), -1);
        cv::putText(result, "Lob", cv::Point(kf_pos.x+5, kf_pos.y-5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
    }

    cv::putText(result, detected ? "Lob: 1" : "Lob: 0", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

    prev_pts_ = next_pts;
    background_gray_ = frame_gray.clone();

// ==========================================
    // 绘制目标与弹丸的连线距离
    // ==========================================
    if (trajectory_active_ && !target_armors.empty()) {
        cv::Point2f bullet_pred(state_.at<float>(0), state_.at<float>(1));
        cv::Rect2f armor = target_armors[0];
        cv::Point2f target_center(armor.x + armor.width / 2.0f, armor.y + armor.height / 2.0f); 
        
        cv::line(result, bullet_pred, target_center, cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
        double dist_to_target = cv::norm(bullet_pred - target_center);
        cv::putText(result, "Dist: " + std::to_string((int)dist_to_target), 
                    cv::Point(bullet_pred.x + 10, bullet_pred.y + 20), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 2);
    }

    // ==========================================
    // 渲染幽灵落点、虚拟装甲板与动态偏差提示
    // ==========================================
    for (auto it = impact_marks_.begin(); it != impact_marks_.end(); ) {
        it->age++;
        
        // 1. 绘制幽灵落点
        cv::drawMarker(result, it->pos, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 20, 3);
        
        if (!target_armors.empty()) {
            cv::Rect2f armor = target_armors[0];
            
            // 2. 在画面上画出黄色的虚拟装甲板框，让操作手心里有数
            cv::rectangle(result, armor, cv::Scalar(0, 255, 255), 2);
            
            // 3. 严格利用物理边框进行判定！
            std::string offset_text;
            cv::Scalar text_color;
            
            if (it->pos.x < armor.x) { 
                // 落在装甲板左边界之外
                offset_text = "<<< LEFT";
                text_color = cv::Scalar(0, 0, 255); // 红色警告
            } else if (it->pos.x > armor.x + armor.width) { 
                // 落在装甲板右边界之外
                offset_text = "RIGHT >>>";
                text_color = cv::Scalar(0, 0, 255); // 红色警告
            } else {
                // 完美落在装甲板边界内部！
                offset_text = "HIT !";
                text_color = cv::Scalar(0, 255, 0); // 绿色命中
            }
            
            // 将判定结果写在落点旁边
            cv::putText(result, offset_text, cv::Point(it->pos.x - 40, it->pos.y - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2, cv::LINE_AA);
        }

        // 驻留 120 帧 (1秒) 后自动抹除
        if (it->age > 120) {
            it = impact_marks_.erase(it);
        } else {
            ++it;
        }
    }

    // ==========================================
    // 【新增】：渲染撞击后驻留的死亡轨迹 (保留 0.5 秒)
    // ==========================================
    for (auto it = dead_trajectories_.begin(); it != dead_trajectories_.end(); ) {
        it->age++;
        
        // 把整条遗留的线画出来 (颜色可以用暗一点的黄色区分，比如 0, 200, 200)
        for (size_t i = 1; i < it->points.size(); i++) {
            cv::line(result, it->points[i-1], it->points[i], cv::Scalar(0, 200, 200), 2);
        }

        // 驻留 60 帧 (对于 120fps 就是半秒钟) 后自动抹除，防止屏幕太乱
        if (it->age > 60) {
            it = dead_trajectories_.erase(it);
        } else {
            ++it;
        }
    }

    return result;
}

} // namespace rm_auto_aim