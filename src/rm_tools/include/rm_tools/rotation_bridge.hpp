#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>  // 放在Eigen后面

#ifndef CV_PI
#define CV_PI 3.1415926535897932384626433832795
#endif

// using namespace rm_tools::tf; 是安全的
// 当然更推荐 using rm_tools::tf::rot_bridge;


// clang-format off

namespace rm_tools::tf
{

class rot_bridge {
    public:
    // ----------------------构造函数--------------
    //! 添加构造函数时务必确保得到的四元数必须是归一化的

        /**
         * @brief 支持很多种构造方法，比如四元数，旋转矩阵，旋转向量，欧拉角等
         */
        rot_bridge(const Eigen::Quaterniond q) { q_ = q; q_.normalize();};
        rot_bridge(const geometry_msgs::msg::Quaternion q);
        rot_bridge(const tf2::Quaternion q);
        rot_bridge(const Eigen::Matrix3d R) { q_ = Eigen::Quaterniond(R); }

        /**
         * @brief 支持很多种构造方法，比如四元数，旋转矩阵，旋转向量，欧拉角等
         * 
         * @param R 同时支持旋转向量与旋转矩阵作为输入
         */
        rot_bridge(const cv::Mat &R);

        /**
         * @brief 支持很多种构造方法，比如四元数，旋转矩阵，旋转向量，欧拉角等
         * 
         * @param eulers  欧拉角（以3维向量形式）
         * @param axis  x=0, y=1, z=2
         * @param extrinsic  是否启用外旋组合
         */
        rot_bridge(const Eigen::Vector3d eulers, int axis0, int axis1, int axis2, bool extrinsic = false);

        // ------------------输出结果-------------------------
        
        /**
         * @brief 输出对应四元数
         * @return 输出四元数类型：函数前不加前缀为Eigen，加ros_为ros消息类型，加tf2_为tf2类型
         */
        Eigen::Quaterniond quaternion() { return q_; };

        /**
         * @brief 输出对应四元数
         * @return 输出四元数类型：函数前不加前缀为Eigen，加ros_为ros消息类型，加tf2_为tf2类型
         */
        geometry_msgs::msg::Quaternion ros_quaternion();

        /**
         * @brief 输出对应四元数
         * @return 输出四元数类型：函数前不加前缀为Eigen，加ros_为ros消息类型，加tf2_为tf2类型
         */
        tf2::Quaternion tf2_quaternion();

        /**
         * @brief 输出旋转矩阵，类型取决于参数
         * @param dst 加了cv::Mat类型的参数时，结果会输入到dst参数里，不加就返回一个Eigen类型矩阵
         */
        void rmat(cv::Mat &dst);

        /**
         * @brief 输出旋转矩阵，类型取决于参数
         * @param dst 加了cv::Mat类型的参数时，结果会输入到dst参数里，不加就返回一个Eigen类型矩阵
         */
        Eigen::Matrix3d rmat() { return Eigen::Matrix3d(q_); }
        
        /**
         * @brief 输出旋转向量，opencv的
         */
        void rvec(cv::Mat &dst);

        /**
         * @brief 输出欧拉角
         * @param axis  x=0, y=1, z=2
         * @param extrinsic  是否启用外旋组合
         * 
         * 参考：rm_tools/math.hpp
         */
        Eigen::Vector3d eulers(int axis0, int axis1, int axis2, bool extrinsic = false);

        /**
         * @brief 输出欧拉角(角度版)
         * @param axis  x=0, y=1, z=2
         * @param extrinsic  是否启用外旋组合
         * 
         * 参考：rm_tools/math.hpp
         */
        Eigen::Vector3d eulers_degree(int axis0, int axis1, int axis2, bool extrinsic = false) 
            { return (this->eulers(axis0, axis1, axis2, extrinsic) * (180.0 / CV_PI)); };

    // 工具函数
        void inv() { q_.conjugate(); };
        double limit_rad(double angle);

        Eigen::Vector3d getAxisVector(int axis)
        {
          switch (axis) {
            case 0:
              return Eigen::Vector3d::UnitX();  // x-axis
            case 1:
              return Eigen::Vector3d::UnitY();  // y-axis
            case 2:
              return Eigen::Vector3d::UnitZ();  // z-axis
            default:
              throw std::invalid_argument("Invalid axis index: must be 0 (x), 1 (y), or 2 (z)");
          }
        }

      private:
        Eigen::Quaterniond q_;
};


}  // namespace rm_tools

namespace rm_tools
{
    bool is_valid_rotation_matrix(const cv::Mat & R);
} // namespace rm_tools

