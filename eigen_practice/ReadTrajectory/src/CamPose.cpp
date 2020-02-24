#include "CamPose.h"

CamPose::CamPose(double time, Eigen::Vector3d trans, Eigen::Quaterniond quat):
        timestamp_(time), trans_(trans), quat_(quat) { }