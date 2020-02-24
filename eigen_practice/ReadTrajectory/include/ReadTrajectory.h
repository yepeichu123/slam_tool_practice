#ifndef READ_TRAJECTORY_H
#define READ_TRAJECTORY_H

#include <string>
#include <vector>
#include <memory>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "CamPose.h"

class ReadTrajectory {
    public:
        // 默认构造函数
        ReadTrajectory();

        // 默认析构函数
        ~ReadTrajectory();

        // 指定轨迹文件
        // 读取的轨迹分别存入时间戳，位移和四元数
        void RunReadTrajectory(const std::string &traj_file, 
                                std::vector<double> &timestamp,
                                std::vector<Eigen::Vector3d> &trans,
                                std::vector<Eigen::Quaterniond> &quat);

        // 制定轨迹文件
        // 读取的轨迹存入pose结构体中
        void RunReadTrajectory(const std::string &traj_file,
                                std::vector<CamPose> &pose);

};

#endif // !READ_TRAJECTORY_H
