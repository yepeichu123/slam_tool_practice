#include <iostream>
#include <cmath>

#include "RotateTransform.h"

using namespace std;

int main(int argc, char** argv) {

	RotateTransform my_rt;
	Eigen::AngleAxisd rot_vec(M_PI/4, Eigen::Vector3d(0, 0, 1));
	Eigen::Matrix3d rot_mat = rot_vec.matrix();
	Eigen::Vector3d euler_angle = rot_vec.matrix().eulerAngles(2, 1, 0);
	Eigen::Quaterniond quat;
	quat = rot_vec;

	my_rt.RunQuatTransform(quat);
	// my_rt.RunRotMatTransform(rot_mat);
	// my_rt.RunEulerAngleTransform(euler_angle);
	// my_rt.RunAngleAxisTransform(rot_vec);

	return 0;
}
