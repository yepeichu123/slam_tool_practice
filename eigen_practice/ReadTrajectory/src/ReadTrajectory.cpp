#include "ReadTrajectory.h"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

// 默认构造函数
ReadTrajectory::ReadTrajectory() {

}

// 默认析构函数
ReadTrajectory::~ReadTrajectory() {

}

// 指定轨迹文件
// 读取的轨迹分别存入时间戳，位移和四元数
void ReadTrajectory::RunReadTrajectory(const std::string &traj_file, 
                        std::vector<double> &timestamp,
                        std::vector<Eigen::Vector3d> &trans,
                        std::vector<Eigen::Quaterniond> &quat) {
    cout << "将轨迹读入各自的时间戳，位移和四元数容器中！" << endl;

    ifstream fin;
    fin.open(traj_file);
    if (!fin.is_open()) {
        cout << "轨迹文件路径有误！" << endl;
        return;
    }

    string line;
    getline(fin, line);
    getline(fin, line);
    getline(fin, line);
    while ( getline(fin, line) ) {
        stringstream ss;
        ss << line;
        double time;
        double tx, ty, tz;
        double qx, qy, qz, qw;
        ss >> time;
        ss >> tx >> ty >> tz;
        ss >> qx >> qy >> qz >> qw;

        timestamp.push_back(time);
        trans.push_back(Eigen::Vector3d(tx, ty, tz));
        quat.push_back(Eigen::Quaterniond(qw, qx, qy, qz));
    }

    cout << "总共读取到 " << timestamp.size() << " 条轨迹！" << endl;
}

// 制定轨迹文件
// 读取的轨迹存入pose结构体中
void ReadTrajectory::RunReadTrajectory(const std::string &traj_file,
                        std::vector<CamPose> &pose) {
    cout << "\n将轨迹读入pose结构体中！" << endl;
    ifstream fin;
    fin.open(traj_file);
    if (!fin.is_open()) {
        cout << "轨迹文件路径有误！" << endl;
        return;
    }

    string line;
    getline(fin, line);
    getline(fin, line);
    getline(fin, line);
    while (getline(fin, line)) {
        stringstream ss;
        ss << line;
        double time;
        double tx, ty, tz;
        double qx, qy, qz, qw;
        ss >> time;
        ss >> tx >> ty >> tz;
        ss >> qx >> qy >> qz >> qw;
        Eigen::Vector3d trans(tx, ty, tz);
        Eigen::Quaterniond quat(qw, qx, qy, qz);
        CamPose cam_pose(time, trans, quat);
        pose.push_back(cam_pose);
    }

    cout << "总共读取到 " << pose.size() << " 条轨迹！" << endl;
}
