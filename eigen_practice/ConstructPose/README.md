# ConstructPose——Designed by 叶培楚

1. 尝试构建四种旋转表示方法;
2. 利用四种方法构建变换矩阵T;
3. Eigen::Isometry 变量需要初始化才能使用（即Eigen::Isometry::Identity()），否则会出错;
4. 同样，用Eigen::Matrix<double, 4, 4>时，也需要初始化Eigen::Matrix<double, 4, 4>::Identity().
