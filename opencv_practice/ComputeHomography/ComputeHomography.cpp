#include <iostream>
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

vector<Point2i> dst_points;
int my_count = 0;

// 鼠标时间触发后调用的程序
void onMouse(int event, int x, int y, int flags, void *param) {
    Mat *img = reinterpret_cast<Mat*>(param);
    if (event == EVENT_LBUTTONDOWN) {
        Point2i p = Point2i(x, y);
        circle(*img, p, 5, Scalar(0,125,125), 2);
        dst_points.push_back(p);
        ++my_count;
    }
}

int main(int argc, char** argv) {

    string src_img_path = "../data/src_img.jpg";
    string dst_img_path = "../data/dst_img.png";
    Mat src_img = imread(src_img_path, 1);
    Mat dst_img = imread(dst_img_path, 1);
    if (src_img.empty() || dst_img.empty()) {
        cout << "输入图像路径有误！请重新确认输入路径！" << endl;
        return -1;
    }

    // 选择源图像中待投影平面顶点（4个）
    vector<Point2i> src_points;
    src_points.push_back(Point2i(0, 0));
    src_points.push_back(Point2i(0, src_img.cols));
    src_points.push_back(Point2i(src_img.rows, 0));
    src_points.push_back(Point2i(src_img.rows, src_img.cols));

    // 利用回调事件，捕获鼠标选取的4个点
    namedWindow("dst_img");
    setMouseCallback("dst_img", onMouse, reinterpret_cast<void*>(&dst_img));
    bool flag = true;
    while (flag) {
        imshow("dst_img", dst_img);
        waitKey(1);

        if (my_count == 4) {
            flag = false;
        }
    }

    // 计算单应矩阵
    Mat H = findHomography(src_points, dst_points, CV_RANSAC);

    // 对源图像进行单应变换
    Mat warped_1;
    warpPerspective(src_img, warped_1, H, dst_img.size());
    imshow("透视变换结果", warped_1);
    waitKey(0);

    // 阈值化
    Mat warped_2;
    threshold(warped_1, warped_2, 0, 255, 1);
    imshow("阈值化后", warped_2);
    waitKey(0);

    // 合并两张图
    Mat final_img;
    final_img = warped_1.clone();
    dst_img.copyTo(final_img, warped_2);
    // Mat test = dst_img & warped_2;
    // final_img = final_img + test;
    imshow("映射后的图像", final_img);
    waitKey(0);
    imwrite("../data/final_img.png", final_img);

    return 0;
}