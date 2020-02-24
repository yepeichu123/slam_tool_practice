// c++
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {

    string input_img = "../data/test.png";
    string output_path = "../data/";

    Mat in_img = imread(input_img, 0);
    if (in_img.empty()) {
        cout << "未读取到图像，请重新确认文件路径！" << endl;
    }
    imshow("in_img", in_img);
    waitKey(0);

    // 该图像的内参数
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05; // 畸变参数
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;
    Mat K = (Mat_<double>(3,3) << fx, 0, cx, 
                                    0, fy, cy,
                                    0, 0, 1);
    Mat destort = (Mat_<double>(4, 1) << k1, k2, p1, p2);
    int row = in_img.rows, col = in_img.cols;
    Size imgSize = in_img.size();

    // 第一种去畸变方法
    Mat out_img = Mat(row, col, CV_8UC1);
    undistort(in_img, out_img, K, destort);
    string img_out1_file = output_path + "out_img_1.png";
    imwrite(img_out1_file, out_img);
    imshow("out_img", out_img);
    waitKey(0);

    // 第二种去畸变方法
    Mat out_img_2 = Mat(row, col, CV_8UC1);
    Mat map1, map2;
    initUndistortRectifyMap(K, destort, Mat(),
        getOptimalNewCameraMatrix(K, destort, imgSize, 1, imgSize, 0),
        imgSize, CV_8UC1, map1, map2);
    remap(in_img, out_img_2, map1, map2, INTER_LINEAR);
    string img_out2_file = output_path + "out_img_2.png";
    imwrite(img_out2_file, out_img_2);
    imshow("out_img_2", out_img_2);
    waitKey(0);

    // 第三种去畸变方法：根据公式推导，对每个点都推导一遍
    Mat out_img_3 = Mat(row, col, CV_8UC1);
    for (int v = 0; v < row; ++v) {
        for (int u = 0; u < col; ++u) {
            double u_distort = 0, v_distort = 0;
            double x1, y1, x2, y2;
            x1 = (u - cx) / fx;
            y1 = (v - cy) / fy;
            double r;
            r = pow(x1, 2) + pow(y1, 2);
            x2 = x1 * (1 + k1*r + k2*pow(r, 2));
            y2 = y1 * (1 + k1*r + k2*pow(r, 2));
            u_distort = fx * x2 + cx;
            v_distort = fy * y2 + cy;

            // 最近邻插值
            if (u_distort >= 0 && u_distort < col &&
                v_distort >= 0 && v_distort < row) {
                out_img_3.at<uchar>(v, u) = in_img.at<uchar>((int)v_distort, (int)u_distort);
            }
            else {
                out_img_3.at<uchar>(v, u) = in_img.at<uchar>((int)v_distort, (int)u_distort);
            }
        }
    }
    string img_out3_file = output_path + "out_img_3.png";
    imwrite(img_out3_file, out_img_3);
    imshow("out_img_3", out_img_3);
    waitKey(0);

    return 0;
}