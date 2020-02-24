#include "ReadTrajectory.h"
#include <iostream>
#include <memory>

using namespace std;

int main(int argc, char** argv) {

    if (argc != 2) {
        cout << "请输入 应用程序，输入文件名称！" << endl;
        return -1;
    }

    string file = argv[1];
    ReadTrajectory my_rt;

    // 时间戳，位移，四元数
    vector<double> timestamp;
    vector<Eigen::Vector3d> trans;
    vector<Eigen::Quaterniond> quat;
    my_rt.RunReadTrajectory(file, timestamp, trans, quat);

    // 相机姿态结构体
    vector<CamPose> cam_pose;
    my_rt.RunReadTrajectory(file, cam_pose);

    return 0;
}