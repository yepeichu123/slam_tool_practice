#ifndef QUAT_TRANSFORM_H
#define QUAT_TRANSFORM_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class RotateTransform {
    public:
        // 默认构造函数
        RotateTransform();

        // 默认析构函数
        ~RotateTransform();

        // 运行四元数变换
        // 从四元数转换为旋转矩阵，欧式旋转，轴角
        void RunQuatTransform(const Eigen::Quaterniond &quat);

        // 运行旋转矩阵变换
        // 从旋转矩阵转换为四元数，欧式旋转，轴角
        void RunRotMatTransform(const Eigen::Matrix3d &rot);

        // 运行欧式旋转变换
        // 从欧式旋转转换为四元数，旋转矩阵，轴角
        void RunEulerAngleTransform(const Eigen::Vector3d &euler_angle);

        // 运行轴角变换
        // 从轴角转换为四元数，旋转矩阵，欧式变换
        void RunAngleAxisTransform(const Eigen::AngleAxisd &rot_vec);

    private:

};

#endif // !QUAT_TRANSFORM_H
