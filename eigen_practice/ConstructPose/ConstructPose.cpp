// c++
#include <iostream>
#include <cmath>

// eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {

    // 旋转描述
    // 构建轴角旋转：绕z轴旋转45度
    AngleAxisd rot_vec(M_PI/4, Vector3d(0, 0, 1));
    // 轴角转换为四元数
    Quaterniond quat;
    quat = rot_vec;
    quat.normalize();
    // 轴角转换为旋转矩阵
    Matrix3d rot_mat = rot_vec.matrix();
    // 轴角转换为欧式旋转
    Vector3d euler_angle = rot_vec.matrix().eulerAngles(2, 1, 0);

    // 位移描述
    Vector3d trans(1, 2, 3);

    // 转换成欧式变换T
    Isometry3d T_1 = Isometry3d::Identity();
    T_1.rotate(quat);
    T_1.pretranslate(trans);
    cout << "T_1 = \n" << T_1.matrix() << endl;

    Isometry3d T_2 = Isometry3d::Identity();
    T_2.linear() = rot_mat;
    T_2.translation() = trans;
    cout << "T_2 = \n" << T_2.matrix() << endl;

    Isometry3d T_3 = Isometry3d::Identity();
    T_3.prerotate(rot_vec);
    T_3.pretranslate(trans);
    cout << "T_3 = \n" << T_3.matrix() << endl;

    Matrix<double, 4, 4> T_4 = Matrix<double, 4, 4>::Identity();
    T_4.block<3, 3>(0, 0) = rot_mat;
    T_4.block<3, 1>(0, 3) = trans;
    cout << "T_4 = \n" << T_4 << endl;

    cout << "Rotation = \n" << T_1.rotation().matrix()
    << "\n translation = \n" << T_1.translation() << endl;

    cout << "Rotation = \n" << T_2.linear().matrix() 
    << "\n translation = \n" << T_2.translation() << endl;

    cout << "Rotation = \n" << T_4.topLeftCorner<3,3>() 
    << "\n translation = \n" << T_4.topRightCorner<3,1>() << endl;
    return 0; 
}