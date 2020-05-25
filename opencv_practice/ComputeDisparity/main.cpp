#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

bool SBMStereoMatching(const Mat& left, const Mat& right, const float& bf, Mat& disparity, Mat& depth);

bool SGBMStereoMatching(const Mat& left, const Mat& right, const float& bf, Mat& disparity, Mat& depth);

void DisparityToDepth(const Mat& disparity, const float& bf, Mat& depth);

int main(int argc, char** argv) {

    if (argc != 3) {
        cout << "Please input ./bin/ComputeDisparity ./data/left.png ./data/right.png." << endl;
        return -1;
    }

    // read images and camera intrinsics
    Mat left = imread(argv[1], IMREAD_GRAYSCALE);
    Mat right = imread(argv[2], IMREAD_GRAYSCALE);
    // kitti
    // Mat K = (Mat_<float>(3, 3) << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1);
    // float baseline = 0.537150653267924;
    // eurco
    Mat K = (Mat_<float>(3, 3) << 458.654, 0, 367.215, 0, 457.296, 248.375, 0, 0, 1);
    float baseline = 0.11007;
    float bf = baseline * K.at<float>(0, 0);
    cout << "bf = " << bf << endl;
    if (left.empty() || right.empty()) {
        cout << "empty images!" << endl;
        return -1;
    }

    // stereo BM
    Mat sbm_disparity, sbm_depth;
    SBMStereoMatching(left, right, bf, sbm_disparity, sbm_depth);
    imwrite("./data/sbm_disparity.png", sbm_disparity);
    imwrite("./data/sbm_depth.png", sbm_depth);
    imshow("sbm_depth", sbm_depth);
    waitKey(0);

    Mat sgbm_disparity, sgbm_depth;
    SGBMStereoMatching(left, right, bf, sgbm_disparity, sgbm_depth);
    imwrite("./data/sgbm_disparity.png", sgbm_disparity);
    imwrite("./data/sgbm_depth.png", sgbm_depth);
    imshow("sgbm_depth", sgbm_depth);
    waitKey(0);

    return 0;
}

bool SBMStereoMatching(const Mat& left, const Mat& right, const float& bf, Mat& disparity, Mat& depth) {

    if (left.empty() || right.empty() || bf == 0) {
        return false;
    }
    
    // parameters for kitti:
    // (16,9), 9, 31, 9, 0, 64, 10, 15, 100, 32, 1, 

    // stereoBM 构造函数
    Ptr<StereoBM> sbm = StereoBM::create(16, 9);

    // 左右视图有效像素区域
    Rect roi1, roi2;
    sbm->setROI1(roi1);
    sbm->setROI2(roi2);

    // 预处理滤波的设置：
    // 滤波器的类型:归一化相应/水平方向sobel算子,目的是降低亮度失真,消除噪声和增强纹理等；
    // 滤波器窗口：一般应该在5*5-21-21之间，且必须为奇数
    // 滤波器截断值：将滤波器输出值保留在一定范围内
    sbm->setPreFilterType(CV_STEREO_BM_NORMALIZED_RESPONSE);
    sbm->setPreFilterSize(9);
    sbm->setPreFilterCap(31);

    // SAD窗口，一般在5*5-21*21间
    sbm->setBlockSize(9);

    // 最小视差值
    sbm->setMinDisparity(0);

    // 设置最大视差值和最小视差值之差,窗口必须是16的整数倍
    Size imgSize = left.size();
    int numOfDisparities = 64;
    sbm->setNumDisparities(numOfDisparities);

    // 低纹理区域的判断阈值
    sbm->setTextureThreshold(10);

    // 类似于最近邻比例法的阈值
    sbm->setUniquenessRatio(15);

    // 检查视差连通区域变化度的窗口大小
    sbm->setSpeckleWindowSize(100);

    // 视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零
    sbm->setSpeckleRange(32);

    // 左视差图（直接计算得出）和右视差图（通过cvValidateDisparity计算得出）之间的最大容许差异
    sbm->setDisp12MaxDiff(1);

    // 视差图
    Mat imgDisparity16S = Mat(left.rows, left.cols, CV_16S);
    // 计算视差图
    sbm->compute(left, right, imgDisparity16S);
    // 将计算得到的视差图转换为输出
    // imgDisparity16S.convertTo(disparity, CV_8U, 255/(numOfDisparities*16.));
    imgDisparity16S.convertTo(disparity, CV_32F, 1.0/16.);

    // 深度图
    depth = Mat(left.rows, left.cols, 0);
    DisparityToDepth(disparity, bf, depth);

    return true;
}

bool SGBMStereoMatching(const Mat& left, const Mat& right, const float& bf, Mat& disparity, Mat& depth) {

    if (left.empty() || right.empty() || bf == 0) {
        return false;
    }
    
    // parameters for kitti:
    // 0, 64, 5, 31, 8, 32, 15, 100, 32, 1, 1

    int minDisparity = 0;
    int numOfDisparities = 64;
    int blockSize = 5;
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(minDisparity, numOfDisparities, blockSize);

    sgbm->setPreFilterCap(31);
    int cn = left.channels();

    sgbm->setP1(8 * cn*blockSize*blockSize);
    sgbm->setP2(32 * cn*blockSize*blockSize);
    sgbm->setUniquenessRatio(15);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(1);

    Mat imgDisparity16S = Mat(left.rows, left.cols, CV_16S);
    sgbm->compute(left, right, imgDisparity16S);
    // imgDisparity16S.convertTo(disparity, CV_8U, 255/(numOfDisparities*16.)); for imshow
    imgDisparity16S.convertTo(disparity, CV_32F, 1.0/16.);

    // 深度图
    depth = Mat(left.rows, left.cols, 0);
    DisparityToDepth(disparity, bf, depth);

    return true;
}

void DisparityToDepth(const Mat& disparity, const float& bf, Mat& depth) {
    if (disparity.empty() || bf == 0) {
        cout << "Empty disparity." << endl;
        return;
    }

    ofstream out_disp("./data/disparity.txt", ios::out);
    ofstream out_depth("./data/depth.txt", ios::out);
    if (!out_disp.is_open() || !out_depth.is_open()) {
        cout << "Open file failed!" << endl;
        return;
    }

    depth = Mat(disparity.rows, disparity.cols, 0);
    for (int r = 0; r < disparity.rows; ++r) {
        out_disp << r << ": " << endl;
        out_depth << r << ": " << endl;
        for (int c = 0; c < disparity.cols; ++c) {
            float d = disparity.at<float>(r,c);
            if (d == -1) {
                continue;
            }
            float z = bf / (float)d;
            if (z <= 0) {
                continue;
            }
            depth.at<uchar>(r, c) = z;

            out_disp << d << " ";
            out_depth << "(" << r << ", " << c << ") = " << z << endl;
        }
        out_disp << endl;
        out_depth << endl;
    }
}