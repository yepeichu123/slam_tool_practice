#include <iostream>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    string in_img1 = "../data/1.png";
    string in_img2 = "../data/2.png";

    // 读取图像
    Mat img1 = imread(in_img1, 1);
    Mat img2 = imread(in_img2, 1);
    if (img1.empty() || img2.empty()) {
        cout << "输入图像有误，请重新确认图像路径！" << endl;
        return -1;
    }

    // 提取特征点
    Ptr<ORB> orb = ORB::create();
    vector<KeyPoint> kpt1, kpt2;
    Mat desp1, desp2;
    orb->detectAndCompute(img1, Mat(), kpt1, desp1);
    orb->detectAndCompute(img2, Mat(), kpt2, desp2);

    // 显示特征点图像
    Mat out_img1;
    drawKeypoints(img1, kpt1, out_img1);
    imshow("out_img1", out_img1);
    waitKey(0);

    // 特征匹配
    vector<DMatch> matches, good_matches;
    Ptr<BFMatcher> matcher = BFMatcher::create();
    matcher->match(desp1, desp2, matches);
    float min_dist = min_element(matches.begin(), matches.end())->distance;
    cout << "匹配点间的最小距离是：" << min_dist << endl;
    for (auto m : matches) {
        if (m.distance < 10*min_dist) {
            good_matches.push_back(m);
        }
    }
    if(good_matches.size() < 10) {
        cout << "匹配对数目不足10，无法计算本质矩阵。请调整匹配阈值！" << endl;
        return -1;
    }
    cout << "匹配对数目为：" << good_matches.size() << endl;

    // 显示匹配后的图像
    Mat matching_img;
    drawMatches(img1, kpt1, img2, kpt2, good_matches, matching_img);
    imshow("matching_img", matching_img);
    waitKey(0);

    // 相机内参数
    float fx = 517.3, fy = 516.5;
    float cx = 318.6, cy = 255.3;
    Mat K = (Mat_<float>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    // 提取匹配对二维点
    vector<Point2d> p1, p2;
    for (auto m : good_matches) {
        Point2f p1_temp = kpt1[m.queryIdx].pt;
        Point2f p2_temp = kpt2[m.trainIdx].pt;
        p1.push_back(
            Point2f((p1_temp.x - cx) / fx, (p1_temp.y - cy) / fy)
        );
        p2.push_back(
            Point2f((p2_temp.x - cx) / fx, (p2_temp.y - cy) / fy)
        );
    }

    // 计算本质矩阵
    Mat E = findEssentialMat(p1, p2, K);
    cout << "本质矩阵：\n E = " << E << endl;

    // 计算极线
    vector<Vec<double,3> > epilines_1, epilines_2;
    computeCorrespondEpilines(p1, 1, E, epilines_1);
    computeCorrespondEpilines(p2, 2, E, epilines_2);
    cout << "极线1有：" << epilines_1.size() << endl;
    cout << "极线2有：" << epilines_2.size() << endl;

    RNG &rng = theRNG(); 
    for (size_t i = 0; i < epilines_1.size(); ++i) {
        Mat p1_norm = (Mat_<double>(3,1) << p1[i].x, p1[i].y, 1);
        Mat p2_norm = (Mat_<double>(3,1) << p2[i].x, p2[i].y, 1);
        cout << "验证对极几何：" << endl;
        cout << "p1.t() * E * p2 = " << p1_norm.t() * E * p2_norm << endl;

        // 极线方程参数
        double a_1 = epilines_1[i][0], b_1 = epilines_1[i][1], c_1 = epilines_1[i][2];
        double a_2 = epilines_2[i][0], b_2 = epilines_2[i][1], c_2 = epilines_2[i][2];

        // 绘制极线
        Scalar color = Scalar(rng(255), rng(255), rng(255));
        // 图一中的点和极线
        circle(img1, p1[i], 5, color, 3);
        line(img1, Point2f(0, -c_1/b_1), Point2f(img1.cols, -(a_1*img1.cols + c_1)/b_1), color);
        // 图二中的点和极线
        circle(img2, p2[i], 5, color, 3);
        line(img2, Point2f(0, -c_2/b_2), Point2f(img2.cols, -(a_2*img2.cols + c_2)/b_2), color);
    }
    imshow("image_1 epipolar line", img1);
    imshow("image_2 epipolar line", img2);
    waitKey(0);

    return 0;
}