#ifndef GUIDE_LIGHT_DETECTOR_HPP_
#define GUIDE_LIGHT_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

namespace rm_auto_aim
{
// 补全原版缺失的 Light 结构体
struct Light {
    cv::RotatedRect rect;
    cv::Point2f center;
    Light() = default;
    Light(cv::RotatedRect r, const std::vector<cv::Point>& pts) : rect(r), center(r.center) {}
};

class GuideLightDetector
{
public:
  struct LightParams
  {
    int binaryThreshold = 60;
    int epsilon = 100;
    int contoursRatioMin = 50;  // 100x
    int contoursRatioMax = 150; // 100x
    int contoursAreaMin = 100;
    int contoursAreaMax = 20000;
    int filledRatioMin = 50;    // 100x
    int filledRatioMax = 100;   // 100x
    int ellipseAreaMin = 100;
    int ellipseAreaMax = 20000;
    int ellipseRatioMin = 50;   // 100x
    int ellipseRatioMax = 150;  // 100x
    bool debug = false; // 【新增】绿灯独立调试开关
  };

  explicit GuideLightDetector(const LightParams & params);
  void setParameters(const LightParams & params);

  std::vector<Light> detect(const cv::Mat & input);
  void drawResults(cv::Mat & img);

private:
  cv::Mat preprocessImage(const cv::Mat & rgb_img);
  std::vector<Light> findLights(const cv::Mat & rgb_img, const cv::Mat & binary_img);
  float filledOfEllipse(const cv::Mat & input, const cv::RotatedRect & ellipse_input);
  std::vector<std::vector<cv::Point>> adaptiveFindInnerContours(
    const cv::Mat & imgBinary, int areaThreshold = 100, bool returnOutmost = false);

  LightParams l_;
  std::vector<Light> lights_;
  // 【新增】：滤波状态记忆变量
  cv::Point2f smoothed_center_{-1, -1};
  float smoothed_radius_{0.0f};
  bool has_prev_light_{false};
  int lost_light_frames_{0};
};

}  // namespace rm_auto_aim

#endif  // GUIDE_LIGHT_DETECTOR_HPP_