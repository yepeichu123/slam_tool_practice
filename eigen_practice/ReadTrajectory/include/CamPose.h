#ifndef CAM_POSE_H
#define CAM_POSE_H

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>

class CamPose {
    public:
    CamPose(double time, Eigen::Vector3d trans, Eigen::Quaterniond quat);

    double timestamp_;
    Eigen::Vector3d trans_;
    Eigen::Quaterniond quat_;
};

#endif // !CAM_POSE_H
