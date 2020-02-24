#include "RotateTransform.h"
#include <iostream>
using namespace std;

RotateTransform::RotateTransform() {

}

RotateTransform::~RotateTransform() {

}

void RotateTransform::RunQuatTransform(const Eigen::Quaterniond &quat) {
    cout << "运行四元数转换为其他各种旋转表示方法！" << endl;
    Eigen::Quaterniond new_quat(quat);
    new_quat.normalize();

    cout << "\n四元数：" << endl;
    cout << "Quat = \n" << new_quat.coeffs().transpose() << endl;

    cout << "\n从四元数转换为轴角：" << endl;
    Eigen::AngleAxisd rot_vec;
    rot_vec = new_quat;
    cout << "Angle asix = \n" << rot_vec.axis().transpose() << ", 角度为 " << rot_vec.angle() * (180/M_PI)  << endl;

    cout << "\n从四元数转换为旋转矩阵：" << endl;
    Eigen::Matrix3d rot_mat_1, rot_mat_2;
    rot_mat_1 = new_quat.matrix();
    rot_mat_2 = new_quat.toRotationMatrix();
    cout << "rot_mat_1 = \n" << rot_mat_1 << endl;
    cout << "rot_mat_2 = \n" << rot_mat_2 << endl;

    cout << "\n从四元数转换为欧式旋转：" << endl;
    Eigen::Vector3d euler_angle;
    euler_angle = new_quat.matrix().eulerAngles(2, 1, 0);
    cout << "euler_anle = \n" << euler_angle.transpose() << endl;
}

void RotateTransform::RunRotMatTransform(const Eigen::Matrix3d &rot) {
    cout << "运行旋转矩阵转换为其他各种旋转表示方法！" << endl;

    cout << "\n旋转矩阵：" << endl;
    cout << "Rot = \n" << rot << endl;

    cout << "\n从旋转矩阵转换为四元数：" << endl;
    Eigen::Quaterniond quat;
    quat = rot;
    cout << "quat = \n" << quat.coeffs().transpose() << endl;

    cout << "\n从旋转矩阵转换为轴角：" << endl;
    Eigen::AngleAxisd rot_vec;
    rot_vec = rot;
    cout << "rot_vec = \n" << rot_vec.axis().transpose() << ", 角度为 " << rot_vec.angle() * (180/M_PI)  << endl;

    cout << "\n从旋转矩阵转换为欧式旋转：" << endl;
    Eigen::Vector3d euler_angle;
    euler_angle = rot.eulerAngles(2, 1, 0);
    cout << "euler_angle = \n" << euler_angle.transpose() << endl;
}

void RotateTransform::RunEulerAngleTransform(const Eigen::Vector3d &euler_angle) {
    cout << "运行欧式变换转换为其他各种旋转表示方法！" << endl;

    cout << "\n欧式旋转：" << endl;
    cout << "euler_angle = \n" << euler_angle.transpose() << endl;

    cout << "\n从欧式旋转转换为四元数：" << endl;
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler_angle(2), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler_angle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler_angle(0), Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quat;
    quat = yawAngle * pitchAngle * rollAngle;
    cout << "quat = \n" << quat.coeffs().transpose() << endl;

    cout << "\n从欧式旋转转换为旋转矩阵：" << endl;
    Eigen::Matrix3d rot_mat;
    rot_mat = yawAngle * pitchAngle * rollAngle;
    cout << "rot_mat = \n" << rot_mat << endl;

    cout << "\n从欧式旋转转换为轴角变换：" << endl;
    Eigen::AngleAxisd rot_vec;
    rot_vec = yawAngle * pitchAngle * rollAngle;
    cout << "rot_vec = \n" << rot_vec.axis().transpose() << ", 角度为 " << rot_vec.angle() * (180/M_PI)  << endl;
}

void RotateTransform::RunAngleAxisTransform(const Eigen::AngleAxisd &rot_vec) {
    cout << "运行轴角旋转转换为其他各种旋转表示方法！" << endl;

    cout << "\n轴角变换：" << endl;
    cout << "rot_vec = \n" << rot_vec.axis().transpose() << ", 角度为 " << rot_vec.angle() * (180/M_PI) << endl;

    cout << "\n从轴角旋转转换为旋转矩阵：" << endl;
    Eigen::Matrix3d rot_mat_1, rot_mat_2;
    rot_mat_1 = rot_vec.matrix();
    rot_mat_2 = rot_vec.toRotationMatrix();
    cout << "rot_mat_1 = \n" << rot_mat_1 << endl;
    cout << "rot_mat_2 = \n" << rot_mat_2 << endl;

    cout << "\n从轴角旋转转换为欧式旋转：" << endl;
    Eigen::Vector3d euler_angle;
    euler_angle = rot_vec.matrix().eulerAngles(2, 1, 0);
    cout << "euler_angle = \n" << euler_angle.transpose() << endl;

    cout << "\n从轴角旋转转换为四元数：" << endl;
    Eigen::Quaterniond quat;
    quat = rot_vec;
    cout << "quat = \n" << quat.coeffs().transpose() << endl;
}
