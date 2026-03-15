#include "detector/guide_light_detector.hpp"

namespace rm_auto_aim
{
GuideLightDetector::GuideLightDetector(const LightParams & params) : l_(params) {}

void GuideLightDetector::setParameters(const LightParams & params) {
    l_ = params;
}

std::vector<Light> GuideLightDetector::detect(const cv::Mat & input) {
    cv::Mat binary_img = preprocessImage(input);
    lights_ = findLights(input, binary_img);
    return lights_; 
}

cv::Mat GuideLightDetector::preprocessImage(const cv::Mat & rgb_img) {
    cv::Mat binary_img;
    std::vector<cv::Mat> channels;
    cv::split(rgb_img, channels);
    
    // 核心提取绿色光源：绿通道减去红通道
    binary_img = channels[1] - channels[2];
    cv::threshold(binary_img, binary_img, l_.binaryThreshold, 255, cv::THRESH_BINARY);
    // 【新增调试探针 1：绿灯二值化图】
    if (l_.debug) {
        cv::Mat vis_binary;
        cv::resize(binary_img, vis_binary, cv::Size(640, 480)); // 缩小一点防止占满屏幕
        //cv::imshow("1_Light_Binary", vis_binary);
        cv::waitKey(1);
    }

    return binary_img;
}


std::vector<Light> GuideLightDetector::findLights(const cv::Mat & rgb_img, const cv::Mat & binary_img) {
    std::vector<Light> lights;
    
    std::vector<std::vector<cv::Point>> contours = adaptiveFindInnerContours(binary_img, l_.contoursAreaMin, true);
    std::vector<std::vector<cv::Point>> approxContours;

    for (size_t i = 0; i < contours.size(); i++) {
        std::vector<cv::Point> approx;
        double epsilon_val = l_.epsilon == 0 ? 1.0 : (1.0 / l_.epsilon);
        cv::approxPolyDP(contours[i], approx, cv::arcLength(contours[i], true) * epsilon_val, true);
        cv::Rect tempRetangle = cv::boundingRect(approx);

        if (tempRetangle.area() > l_.contoursAreaMin && tempRetangle.area() < l_.contoursAreaMax * 100 &&
            tempRetangle.tl().y > 0 && tempRetangle.br().y < binary_img.rows && 
            tempRetangle.tl().x > 0 && tempRetangle.br().x < binary_img.cols) {
            approxContours.emplace_back(approx);
        }
    }

    // 1. 收集当前帧所有合格的候选圆
    std::vector<Light> candidates;
    for (size_t i = 0; i < approxContours.size(); i++) {
        if (approxContours[i].size() < 3) continue;

        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(approxContours[i], center, radius);

        cv::RotatedRect tempCircle(center, cv::Size2f(radius * 2.0f, radius * 2.0f), 0.0f);
        float filled = filledOfEllipse(binary_img, tempCircle);
        float circle_area = CV_PI * radius * radius;

        if (filled < l_.filledRatioMax / 100.0 && circle_area > l_.ellipseAreaMin) {
            candidates.emplace_back(Light(tempCircle, approxContours[i]));
        }
    }

    // 2. 时序滤波 (EMA Filter) 与抗闪烁逻辑
    if (!candidates.empty()) {
        // 在所有候选项中，优先选择面积最大的（最突出的主引导灯），天然过滤掉周围的噪点绿光
        auto best_it = std::max_element(candidates.begin(), candidates.end(),
            [](const Light& a, const Light& b) {
                return a.rect.size.area() < b.rect.size.area();
            });

        cv::Point2f current_center = best_it->center;
        float current_radius = best_it->rect.size.width / 2.0f;

        if (!has_prev_light_) {
            // 第一次检测到，初始化滤波器
            smoothed_center_ = current_center;
            smoothed_radius_ = current_radius;
            has_prev_light_ = true;
        } else {
            // 【核心 EMA 滤波算法】
            // alpha 值越小，抗抖动能力越强（越滞后）；alpha 值越大，跟随越紧密。
            const float alpha_pos = 0.1f;  // 位置平滑系数 (极其稳定)
            const float alpha_size = 0.02f; // 半径平滑系数更小，死死锁住圆的大小

            // 突变检测：如果云台大幅度甩动导致位置瞬间突变超过 100 像素，强制重置滤波器避免强拉扯
            if (cv::norm(current_center - smoothed_center_) > 100.0) {
                smoothed_center_ = current_center;
                smoothed_radius_ = current_radius;
            } else {
                // 平滑融合：新位置 = 当前测量值 * alpha + 历史记忆值 * (1 - alpha)
                smoothed_center_ = current_center * alpha_pos + smoothed_center_ * (1.0f - alpha_pos);
                smoothed_radius_ = current_radius * alpha_size + smoothed_radius_ * (1.0f - alpha_size);
            }
        }
        lost_light_frames_ = 0; // 重置丢帧计数器

        // 使用平滑后的数据构造最终的绿灯输出
        cv::RotatedRect smoothed_rect(smoothed_center_, cv::Size2f(smoothed_radius_ * 2.0f, smoothed_radius_ * 2.0f), 0.0f);
        lights.emplace_back(Light(smoothed_rect, std::vector<cv::Point>())); 

    } else {
        // 3. 护城河逻辑：如果当前帧没检测到绿灯（闪烁/被飞镖瞬间遮挡）
        lost_light_frames_++;
        
        if (lost_light_frames_ < 8 && has_prev_light_) {
            // 短暂丢帧（例如低于8帧），利用滤波器的记忆继续输出上一帧的绿灯！
            cv::RotatedRect smoothed_rect(smoothed_center_, cv::Size2f(smoothed_radius_ * 2.0f, smoothed_radius_ * 2.0f), 0.0f);
            lights.emplace_back(Light(smoothed_rect, std::vector<cv::Point>()));
        } else if (lost_light_frames_ >= 8) {
            // 彻底丢失超过8帧，清除历史记忆，等待下次重新捕捉
            has_prev_light_ = false;
        }
    }

    return lights;
}

void GuideLightDetector::drawResults(cv::Mat & img) {
    for (const auto & light : lights_) {
        cv::ellipse(img, light.rect, cv::Scalar(0, 255, 0), 3); 
        cv::circle(img, light.center, 5, cv::Scalar(0, 0, 255), -1);
        // cv::putText(img, "Guide_Light", cv::Point(light.center.x + 10, light.center.y), 
        //             cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }
}

float GuideLightDetector::filledOfEllipse(const cv::Mat & input, const cv::RotatedRect & ellipse_input) {
    CV_Assert(input.type() == CV_8UC1);
    if (ellipse_input.center.x < 0 || ellipse_input.center.x >= input.cols ||
        ellipse_input.center.y < 0 || ellipse_input.center.y >= input.rows) {
        return -1;
    }

    if (ellipse_input.size.width > 10 || ellipse_input.size.height > 10) {
        cv::Mat mask = cv::Mat::zeros(input.size(), CV_8UC1);
        cv::ellipse(mask, ellipse_input, cv::Scalar(255), -1);
        cv::bitwise_and(input, mask, mask);
        cv::Scalar gray_sum = cv::sum(mask) / 255.0;
        float ellipse_area = CV_PI * (ellipse_input.size.width / 2.0) * (ellipse_input.size.height / 2.0);
        return ellipse_area > 0 ? (gray_sum[0] / ellipse_area) : -1;
    }
    return -1;
}

std::vector<std::vector<cv::Point>> GuideLightDetector::adaptiveFindInnerContours(
    const cv::Mat & imgBinary, int areaThreshold, bool returnOutmost) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> inner_contours;
    std::vector<cv::Vec4i> hierarchy;
    bool haveInnerContours = false;
    cv::findContours(imgBinary, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < contours.size(); i++) {
        if (hierarchy[i][3] != -1 && cv::boundingRect(contours[i]).area() > areaThreshold) {
            inner_contours.emplace_back(contours[i]);
            haveInnerContours = true;
        }
    }
    if (!haveInnerContours && returnOutmost) return contours;
    else return inner_contours;
}

}  // namespace rm_auto_aim