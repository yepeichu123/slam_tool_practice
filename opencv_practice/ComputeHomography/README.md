# ComputeHomography——Designed by 叶培楚

程序目标：
    通过鼠标按键在目标图像中指定位置（四个点），将源文件图像贴到目标图像中。

程序流程：
1. 读取图像;
2. 设置源文件图像src_img要贴到目标图像dst_img的部分;
3. 利用鼠标事件回调记录鼠标在目标图像中的贴图位置;
4. 计算单应矩阵H；
5. 利用单应矩阵H将源文件图像映射成指定大小，但图像大小保持与目标文件一致warped_1;
6. 将步骤5中的图拷贝后阈值化warped_2；
7. 将阈值化后的图像warped_2与目标dst_img相与，再和warped_1相加，得到最终的图像。
    即：final_img = warped_1 + (warped_2 & dst_img);