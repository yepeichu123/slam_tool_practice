#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

using namespace cv;
using namespace std;
using namespace Eigen;

void matchByNcc(const cv::Mat& left_img, const cv::Mat& right_img, vector<cv::Point2f>& left_p, vector<cv::Point2f>& right_p, vector<cv::Point2f>& sec_right_p);

void featureExtraction(const cv::Mat& img, vector<cv::Point2f>& points);

void matchDisplay(const cv::Mat& left_img, const cv::Mat& right_img, vector<cv::Point2f>& left_p, vector<cv::Point2f>& right_p, vector<cv::Point2f>& sec_right_p);

void subPiexl(const cv::Mat& left_img, const cv::Mat& right_img, cv::Point2f& left_p, cv::Point2f& right_p);

bool remapFrames(vector<Mat>& left_img, vector<Mat>& right_img);

bool readEurocData(const string& filename_left, const string& filename_right, vector<Mat>& left_img, vector<Mat>& right_img);

bool runRansac(vector<cv::Point2f>& left_p, vector<cv::Point2f>& right_p);

void computeDepth(const vector<cv::Point2f>& left_p, const vector<cv::Point2f>& right_p, const float& bf, vector<float>& vdepth);

string data_path_left = "/home/ypc/xiaoc/dataset/EUROC/MH_01_easy/mav0/cam0/data/";
string data_path_right = "/home/ypc/xiaoc/dataset/EUROC/MH_01_easy/mav0/cam1/data/";

int main(int argc, char** argv) {

    if (argc != 3) {
        cout << "Please input ./bin/StereoMatchNcc ./data/filename_left.txt ./data/filename_right.txt." << endl;
        return -1;
    }

    // Mat left_img = imread(argv[1], IMREAD_UNCHANGED);
    // Mat right_img = imread(argv[2], IMREAD_UNCHANGED);
    // 校正
    // vector<Mat> vleft_img, vright_img;
    // vleft_img.push_back(left_img);
    // vright_img.push_back(right_img);
    float baseline = 0.110073808127187;
    float fx = 458.654;
    float bf = baseline * fx;

    string left_path = argv[1];
    string right_path = argv[2];
    vector<Mat> vLeftImg, vRightImg;
    readEurocData(left_path, right_path, vLeftImg, vRightImg);
    if (vLeftImg.size() != vRightImg.size() || vLeftImg.size() == 0) {
        cout << "Empty images. " << endl;
        return -1;
    } 

    // undistort and remap
    remapFrames(vLeftImg, vRightImg);
    vector<float> vdepth;
    for (int i = 0; i < vLeftImg.size(); i++) {
        cout << "******************Process frame " << i << "*************************" << endl;
        Mat left_img = vLeftImg[i];
        Mat right_img = vRightImg[i];

        // extract orb features
        vector<Point2f> left_p, right_p, right_p_sec;
        featureExtraction(left_img, left_p);

        // ncc search 
        matchByNcc(left_img, right_img, left_p, right_p, right_p_sec);

        // ransac
        runRansac(left_p, right_p);

        // display
        matchDisplay(left_img, right_img, left_p, right_p, right_p_sec);

        // depth 
        computeDepth(left_p, right_p, bf, vdepth);
    }    

    return 0;
}

void matchByNcc(const cv::Mat& left_img, const cv::Mat& right_img, vector<cv::Point2f>& left_p, 
    vector<cv::Point2f>& right_p, vector<cv::Point2f>& sec_right_p) {
    
    if (left_img.empty() || right_img.empty()) {
        cout << "Empty input image, please check it and try again." << endl;
        return;
    }

    if (left_p.size() == 0) {
        cout << "No features extracted, please check it again." << endl;
        return;
    }

    const int PADDING = 4;
    int u_patchSize = 5;
    int v_patchSize = 3;
    int nRows = left_img.rows;
    int nCols = left_img.cols;
    cout << "Image size = " << left_img.size() << endl;

    // float u_min, v_min, u_max, v_max;
    vector<cv::Point2f> new_left_p;
    for (int i = 0; i < left_p.size(); ++i) {
        cv::Point2f pt_2d = left_p[i];

        // 开始极线搜索进行NCC匹配
        float ncc_max = 0, ncc_second_max = 0, ncc_min = 0;
        float u_match = -1, u_sec_match = -1;
        float v_match = pt_2d.y;

        // 由于已经默认双目相机校正过,因此双目图像的极线是水平的
        for(float u = pt_2d.x / 2; u < pt_2d.x; u = u + 1.0) {
            float ncc = 0;
            float x_avg = 0, y_avg = 0;
            float xx = 0, yy = 0, xy = 0;
            float u_half_patch = (float)u_patchSize / 2.0;
            float v_half_patch = (float)v_patchSize / 2.0;
            int count = 0;
            // 计算对应窗口平均值
            for (float du = -u_half_patch; du < u_half_patch; du = du + 1.0) {
                for (float dv = -v_half_patch; dv < v_half_patch; dv = dv + 1.0) {
                    
                    float x = (float)left_img.at<uchar>(int(pt_2d.y + dv), int(pt_2d.x + du));
                    float y = (float)right_img.at<uchar>(int(v_match + dv), int(u + du));
                    x_avg += x;
                    y_avg += y;
                    ++count;
                }
            }
            x_avg /= (float)count;
            y_avg /= (float)count;

            // 计算ncc分数
            for (float du = -u_half_patch; du < u_half_patch; du = du + 1.0) {
                for (float dv = -v_half_patch; dv < v_half_patch; dv = dv + 1.0) {

                    float x = (float)left_img.at<uchar>(int(pt_2d.y + dv), int(pt_2d.x + du)) - x_avg;
                    float y = (float)right_img.at<uchar>(int(v_match + dv), int(u + du)) - y_avg;

                    xx += x * x;
                    yy += y * y;
                    xy += x * y;
                }
            }
            ncc = xy / sqrt(xx * yy);

            if (ncc > ncc_max) {
                ncc_second_max = ncc_max;
                u_sec_match = u_match;
                ncc_max = ncc;
                u_match = u;
            }
            if (ncc < ncc_min) {
                ncc_min = ncc;
            }
        }

        // 如果ncc太小,或者整体ncc差别太小,则剔除点
        if (abs(u_match - u_sec_match) > 5 || ncc_max < 0.9 || ncc_second_max < 0.9 || (ncc_max - ncc_second_max) > 0.5) {
            continue;
        }

        // cout << "ncc_max = " << ncc_max << ", ncc_second_max = " << ncc_second_max << ", ncc_min = " << ncc_min << endl;
        // cout << "v_match = " << v_match << ", best_match = " << u_match << ", second_match = " << u_sec_match << ", error = " << u_match - u_sec_match << endl;
        // cout << endl;

        Point2f bestRight(u_match, v_match);
        subPiexl(left_img, right_img, pt_2d, bestRight);
        // cout << "After subpixel: " <<  " best_match = " << bestRight.x << endl;
        if (bestRight.x <= 0) {
            continue;
        }
        
        new_left_p.push_back(pt_2d);
        right_p.push_back(bestRight);
        sec_right_p.push_back(Point2f(u_sec_match, v_match));
    }
    left_p.clear();
    left_p.insert(left_p.end(), new_left_p.begin(), new_left_p.end());
    cout << "Finished NCC. And we found " << right_p.size() << " points." << endl;
}

void featureExtraction(const cv::Mat& img, vector<cv::Point2f>& points) {
    Ptr<ORB> orb = ORB::create(1200);
    vector<cv::KeyPoint> kpt;
    Mat desp;
    orb->detectAndCompute(img, cv::Mat(), kpt, desp);

    Mat draw_feat;
    drawKeypoints(img, kpt, draw_feat, Scalar::all(125), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow("draw_feat", draw_feat);
    waitKey(1);

    points.clear();
    for (auto p: kpt) {
        points.push_back(p.pt);
    }

    cout << "We extrac " << points.size() << " points." << endl;
}

void matchDisplay(const cv::Mat& left_img, const cv::Mat& right_img, vector<cv::Point2f>& 
    left_p, vector<cv::Point2f>& right_p, vector<cv::Point2f>& sec_right_p) {
    if (left_img.empty() || right_img.empty()) {
        cout << "Empty image, please check it again." << endl;
        return;
    }
    if (left_p.size() != right_p.size() || left_p.size() == 0) {
        cout << "Error matching or empty features." << endl;
        return;
    }

    int nRows = left_img.rows;
    int nCols = left_img.cols;
    Mat draw_match(nRows, nCols*2, CV_8UC1);
    left_img.copyTo(draw_match(Rect(0, 0, nCols, nRows)));
    right_img.copyTo(draw_match(Rect(nCols, 0, nCols, nRows)));
    for (int i = 0; i < left_p.size(); ++i) {
        Point2f start = left_p[i];
        Point2f end = right_p[i];
        Point2f sec_end = sec_right_p[i];
        end.x = end.x + nCols;
        sec_end.x = sec_end.x + nCols;

        circle(draw_match, start, 5, Scalar(125, 255, 125), 3);
        circle(draw_match, end, 5, Scalar(125, 255, 125), 3);
        circle(draw_match, sec_end, 5, Scalar(0, 0, 255), 3);
        line(draw_match, start, end, Scalar(125, 125, 125), 1);
    }
    
    imshow("draw_match", draw_match);
    waitKey(0);
}

bool remapFrames(vector<Mat>& left_img, vector<Mat>& right_img) {
    if (left_img.empty() || right_img.empty()) {
        cout << "Input empty images, please check it again." << endl;
        return false;
    }

    // Euroc
    Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    K_l = (Mat_<float>(3, 3) << 458.654, 0.0, 367.215, 
                                0.0, 457.296, 248.375, 
                                0.0, 0.0, 1.0);
    K_r = (Mat_<float>(3, 3) << 457.587, 0.0, 379.999, 
                                0.0, 456.134, 255.238, 
                                0.0, 0.0, 1.0);

    P_l = (Mat_<float>(3, 4) << 435.2046959714599, 0, 367.4517211914062, 0,  
                                0, 435.2046959714599, 252.2008514404297, 0,  
                                0, 0, 1, 0);
    P_r = (Mat_<float>(3, 4) << 435.2046959714599, 0, 367.4517211914062, -47.90639384423901, 
                                0, 435.2046959714599, 252.2008514404297, 0, 
                                0, 0, 1, 0);

    R_l = (Mat_<float>(3, 3) << 0.999966347530033, -0.001422739138722922, 0.008079580483432283, 
                                0.001365741834644127, 0.9999741760894847, 0.007055629199258132, 
                                -0.008089410156878961, -0.007044357138835809, 0.9999424675829176);
    R_r = (Mat_<float>(3, 3) << 0.9999633526194376, -0.003625811871560086, 0.007755443660172947, 
                                0.003680398547259526, 0.9999684752771629, -0.007035845251224894, 
                                -0.007729688520722713, 0.007064130529506649, 0.999945173484644);
    
    D_l = (Mat_<float>(1, 5) << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0);
    D_r = (Mat_<float>(1, 5) << -0.28368365, 0.07451284, -0.00010473, -3.555907e-05, 0.0);

    // cout << "K_l = \n" << K_l << endl;
    // cout << "P_l = \n" << P_l << endl;
    // cout << "R_l = \n" << R_l << endl;
    // cout << "D_l = \n" << D_l << endl;

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty())
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return false;
    }

    int cols_l = left_img[0].cols;
    int rows_l = left_img[0].rows;
    int cols_r = right_img[0].cols;
    int rows_r = right_img[0].rows;
    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    // cout << "M1l = " << M1l << endl;
    // cout << "M2l = " << M2l << endl;
    // cout << "M1r = " << M1r << endl;
    // cout << "M2r = " << M2r << endl;

    vector<Mat> new_left, new_right;
    for (int i = 0; i < left_img.size(); ++i) {
        Mat left = left_img[i];
        Mat right = right_img[i];

        Mat leftRect, rightRect;
        remap(left, leftRect, M1l, M2l, INTER_LINEAR);
        remap(right, rightRect, M1r, M2r, INTER_LINEAR);
        // imshow("left_test", leftRect);
        // imshow("right_test", rightRect);
        // waitKey(0);
        new_left.push_back(leftRect);
        new_right.push_back(rightRect);
    }

    left_img.clear();
    right_img.clear();
    left_img.insert(left_img.end(), new_left.begin(), new_left.end());
    right_img.insert(right_img.end(), new_right.begin(), new_right.end());

    return true;
}

bool readEurocData(const string& filename_left, const string& filename_right, vector<Mat>& left_img, vector<Mat>& right_img) {
    ifstream fin_left, fin_right;
    fin_left.open(filename_left);
    fin_right.open(filename_right);
    if (!fin_left.is_open() || !fin_right.is_open()) {
        return false;
    }

    left_img.clear();
    right_img.clear();
    string left_s, right_s;
    getline(fin_left, left_s);
    getline(fin_right, right_s);

    int count = 0;
    while (!fin_left.eof() && !fin_right.eof()) {
        string left_path = data_path_left + left_s;
        string right_path = data_path_right + right_s;
        cout << count++ << endl;
        cout << "left : " << left_path << endl;
        cout << "right : " << right_path << endl;
        Mat left = imread(left_path.c_str(), 0);
        Mat right = imread(right_path.c_str(), 0);
        if (left.empty() || right.empty()) {
            return false;
        }
        left_img.push_back(left);
        right_img.push_back(right);

        getline(fin_left, left_s);
        getline(fin_right, right_s);

        // Only read first 100 frames
        if (count > 100) {
            break;
        }
    }
    cout << "Successfully load images." << endl;

    return true;
}

bool runRansac(vector<cv::Point2f>& left_p, vector<cv::Point2f>& right_p) {
    if (left_p.empty() || right_p.empty()) {
        cout << "Empty points, please check it again." << endl;
        return false;
    }
    if (left_p.size() != right_p.size()) {
        cout << "two points array are not equal." << endl;
        return false;
    }
    if (left_p.size() < 8) {
        cout << "Few points, exit." << endl;
        return false;
    }

    vector<uchar> inliers;
    // findFundamentalMat(left_p, right_p, inliers, FM_RANSAC);
    findHomography(left_p, right_p, inliers);
    vector<cv::Point2f> new_left_p, new_right_p;
    for (int i = 0; i < inliers.size(); ++i) {
        if (inliers[i] == 0) {
            continue;
        }
        new_left_p.push_back(left_p[i]);
        new_right_p.push_back(right_p[i]);
    }
    left_p.clear();
    right_p.clear();
    left_p.insert(left_p.end(), new_left_p.begin(), new_left_p.end());
    right_p.insert(right_p.end(), new_right_p.begin(), new_right_p.end());

    cout << "After ransac, we have " << left_p.size() << " points left." << endl;

    return true;
}

void subPiexl(const cv::Mat& left_img, const cv::Mat& right_img, cv::Point2f& left_p, cv::Point2f& right_p) {
    
    const float uR0 = right_p.x;
    const float uL0 = left_p.x;
    const float vL0 = left_p.y;
    const int w = 5;
    cv::Mat IL = left_img.rowRange(vL0 - w, vL0 + w + 1).colRange(uL0 - w, uL0 + w + 1);
    IL.convertTo(IL, CV_32F);
    IL = IL - IL.at<float>(w, w) * Mat::ones(IL.rows, IL.cols, CV_32F);

    int bestDist = INT_MAX;
    int bestincR = 0;
    const int L = 5;
    vector<float> vDists;
    vDists.resize(2*L + 1);
    
    const float iniu = uR0 + L - w;
    const float endu = uR0 + L + w + 1;
    if (iniu < 0 || endu >= left_img.cols) {
        right_p.x = -1;
        return;
    }

    for (int incR = -L; incR <= L; ++incR) {
        cv::Mat IR = right_img.rowRange(vL0 - w, vL0 + w + 1).colRange(uR0 + incR - w, uR0 + incR + w + 1);
        IR.convertTo(IR, CV_32F);
        IR = IR - IR.at<float>(w, w) * Mat::ones(IR.rows, IR.cols, CV_32F);

        float dist = cv::norm(IL, IR, cv::NORM_L1);
        if (dist < bestDist) {
            bestDist = dist;
            bestincR = incR;
        }

        vDists[L+incR] = dist;
    }

    // cout << "bestincR = " << bestincR << endl;
    if(bestincR==-L || bestincR==L) {
        right_p.x = -1;
        return;
    }

    // Sub-pixel match (Parabola fitting)
    const float dist1 = vDists[L+bestincR-1];
    const float dist2 = vDists[L+bestincR];
    const float dist3 = vDists[L+bestincR+1];

    const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

    // cout << "deltaR = " << deltaR << endl;

    if(deltaR < -1 || deltaR > 1) {
        right_p.x = -1;
        return;
    }

    float bestuR = uR0 + bestincR + deltaR;

    right_p.x = bestuR;
}

void computeDepth(const vector<cv::Point2f>& left_p, const vector<cv::Point2f>& right_p, const float& bf, vector<float>& vdepth) {
    if (left_p.size() == 0 || left_p.size() != right_p.size()) {
        cout << "error input points." << endl;
        return;
    }

    vdepth.clear();
    for (int i = 0; i < left_p.size(); ++i) {
        float delta = left_p[i].x - right_p[i].x;
        float depth = bf / delta;
        if (depth < 0) {
            continue;
        }
        vdepth.push_back(depth);
        cout << "Point[" << i << "] = " << depth << endl;
    }
}