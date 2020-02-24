# DestortionImg——Designed by 叶培楚

### 提供三种去畸变方法：
1. undistort 函数，非常方便，输出图像与原图尺寸、类型一样；
2. initUndistortRectifyMap 函数先根据畸变参数和相机内参生成映射关系map1, map2, 再利用remap函数对图像进行去畸变。（PS：利用getOptimalNewCameraMatrix函数来获得新的相机矩阵）;
3. 利用畸变函数与去畸变函数之间的理论推导关系，对图像每一个像素值进行去畸变，并利用插值法将结果赋值给输出图像。