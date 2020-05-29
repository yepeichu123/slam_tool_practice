#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

string data_path_left = "/home/ypc/xiaoc/dataset/EUROC/MH_01_easy/mav0/cam0/data/";
string data_path_right = "/home/ypc/xiaoc/dataset/EUROC/MH_01_easy/mav0/cam1/data/";
string filename_left = "./data/filename_left.txt";
string filename_right = "./data/filename_right.txt";

bool remapFrames(vector<Mat>& left_img, vector<Mat>& right_img);

bool readEurocData(const string& filename_left, const string& filename_right, vector<Mat>& left_img, vector<Mat>& right_img);

bool trackEuroc(vector<Mat>& left_img, vector<Mat>& right_img, const double& bf, const string& out_file);

bool trackSingleFrame(const Mat& left, const Mat& right, const double& bf);

bool ComputeDepthByNCCNew(const Mat& left, const Mat& right, const double& bf, vector<Point2f>& left_pt, vector<Point2f>& right_pt);

int main(int argc, char** argv) {

    // euroc
    Mat K = (Mat_<float>(3, 3) << 458.654, 0, 367.215, 0, 457.296, 248.375, 0, 0, 1);
    float baseline = -0.11007;
    // kitti
    // Mat K = (Mat_<float>(3, 3) << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1);
    // float baseline = 0.537150653267924;
    float bf = baseline * K.at<float>(0, 0);

    // read images 
    if (argc != 3) {
        cout << "If you want to use trackSingleFrame." << endl;
        cout << "Please input ./build/KLTStereoMatching ./data/left.png ./data/right.png" << endl;
        cout << "Now, we will use the trackEuroc." << endl;
        string out = "./data/out.txt";
        // read images
        vector<Mat> left_img, right_img;
        bool flag = readEurocData(filename_left, filename_right, left_img, right_img);
        flag == true ? (cout << "Read images successfully." << endl) : (cout << "Read images failed." << endl);
        cout << "We read " << left_img.size() << " images from " << data_path_left << " and " << data_path_right << endl;
        // undistort and remap
        flag = remapFrames(left_img, right_img);
        cout << "After remapping, we have " << left_img.size() << " images left." << endl;
        // track
        flag = trackEuroc(left_img, right_img, bf, out);
        flag == true ? (cout << "trackEuroc successfully." << endl) : (cout << "trackEuroc failed." << endl);
    }
    else {
        cout << "Now, we will use the trackSingleFrame." << endl;
        Mat left = imread(argv[1], IMREAD_GRAYSCALE);
        Mat right = imread(argv[2], IMREAD_GRAYSCALE);
        vector<Mat> left_test, right_test;
        left_test.push_back(left);
        right_test.push_back(right);
        // undistort and remap
        bool flag = remapFrames(left_test, right_test);
        if (flag) {
            // imshow("origin", left);
            // imshow("remap_left", left_test[0]);
            // waitKey(0);
        }
        else {
            return -1;
        }
        flag = trackSingleFrame(left, right, bf);
        flag == true ? (cout << "trackSingleFrame successfully." << endl) : (cout << "trackSingleFrame failed." << endl);
    }

    return 0;
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
    }
    cout << "Successfully load images." << endl;

    return true;
}

bool trackEuroc(vector<Mat>& left_img, vector<Mat>& right_img, const double& bf, const string& out_file) {
    if (left_img.empty() || right_img.empty()) {
        return false;
    }
    if (bf == 0) {
        return false;
    }
    ofstream out(out_file, ios::out);
    if (!out.is_open()) {
        return false;
    }

    int count = 0;
    while (count < left_img.size() && count < left_img.size()) {
        cout << "Image " << count << ": " << endl;
        Mat left = left_img[count];
        Mat right = right_img[count];

        vector<Point2f> points[2];

        // extract points
        // vector<KeyPoint> kpts;
        // Mat desp;
        // Ptr<ORB> orb = ORB::create(1200);
        // orb->detectAndCompute(left, Mat(), kpts, desp);
        TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
        goodFeaturesToTrack(left, points[0], 1200, 0.01, 10);
        cornerSubPix(left, points[0], Size(10,10), Size(-1,-1), termcrit);
        // convert
        // for (int i = 0; i < kpts.size(); ++i) {
            // points[0].push_back(kpts[i].pt);
        // }
        cout << "we extract " << points[0].size() << " points." << endl;

        vector<uchar> state;
        vector<float> error;
        calcOpticalFlowPyrLK(left, right, points[0], points[1], state, error, 
            Size(21, 21), 5);

        // draw points
        Mat draw_match = Mat(Size(2*left.cols, left.rows), CV_8UC1);
        left.copyTo(draw_match(Rect(0, 0, left.cols, left.rows)));
        right.copyTo(draw_match(Rect(left.cols, 0, left.cols, left.rows)));
        vector<Point2f> new_points[2];
        for (int i = 0; i < points[0].size(); ++i) {

            if (state[i] == 0) {
                continue;
            }

            Point2f start = points[0][i];
            Point2f end = Point2f(points[1][i].x + left.cols, points[1][i].y);
            circle(draw_match, start, 5, Scalar(125, 255, 125), 2);
            circle(draw_match, end, 5, Scalar(125, 255, 125), 2);
            line(draw_match, start, end, Scalar(125, 0, 125), 2);

            // make depth image
            float d = points[0][i].x - points[1][i].x;
            // if (d >= 0) {
                // continue;
            // }

            new_points[0].push_back(points[0][i]);
            new_points[1].push_back(points[1][i]);
        }
        cout << "After KLT, we have " << new_points[0].size() << " points left." << endl;

        // ransac
        vector<uchar> ransac_state;
        vector<cv::Point2f> left_final, right_final;
        if (new_points[0].size() > 0)
            findFundamentalMat(new_points[0], new_points[1], ransac_state, cv::FM_RANSAC);
        for (int i = 0; i < ransac_state.size(); ++i) {
            if (ransac_state[i] != 0) {
                left_final.push_back(new_points[0][i]);
                right_final.push_back(new_points[1][i]);
            }
        }
        cout << "After ransac, we have " << left_final.size() << " points left." << endl;

        out << "id = " << count << ", KLT = " << new_points[0].size() << ", ransac = " << left_final.size() << endl;

        if (left_final.size() > 0) {
            imshow("draw_match", draw_match);
            waitKey(1);
        }
        ++count;
        /*
        if (left_final.size() == 0) {
            bool flag = ComputeDepthByNCCNew(left, right, bf, points[0], points[1]);
            if (flag) {
                for (int i = 0; i < points[0].size(); i++) {
                    Point2f start = points[0][i];
                    Point2f end = Point2f(points[1][i].x + left.cols, points[1][i].y);
                    line(draw_match, start, end, Scalar(125, 0, 125));
                }
                imshow("ncc_matching", draw_match);
                waitKey(0);
            }
            else {
                cout << "Oh shit, we cannot match any points." << endl;
            }
        }
        */
    }
    out.close();
    return true;
}
 
bool trackSingleFrame(const Mat& left, const Mat& right, const double& bf) {

    bool use_ncc = true;
    // camera intrinsics and baseline 
    // euroc
    // Mat K = (Mat_<float>(3, 3) << 458.654, 0, 367.215, 0, 457.296, 248.375, 0, 0, 1);
    // float baseline = -0.11007;
    // kitti
    // Mat K = (Mat_<float>(3, 3) << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1);
    // float baseline = 0.537150653267924;
    // float bf = baseline * K.at<float>(0, 0);

    // feature extracting
    vector<Point2f> points[2];

    vector<KeyPoint> kpts;
    Mat desp;
    Ptr<ORB> orb = ORB::create(500);
    orb->detectAndCompute(left, Mat(), kpts, desp);
    // convert00
    for (int i = 0; i < kpts.size(); ++i) {
        points[0].push_back(kpts[i].pt);
    }
    cout << "we extract " << points[0].size() << " points." << endl;

    // show image
    Mat draw_left = left.clone();
    Mat draw_right = right.clone();
    Mat draw_match = Mat(Size(2*left.cols, left.rows), CV_8UC1);
    left.copyTo(draw_match(Rect(0, 0, left.cols, left.rows)));
    right.copyTo(draw_match(Rect(left.cols, 0, left.cols, left.rows)));
    // Mat depth, disparity;
    // depth = Mat(left.rows, left.cols, CV_32F);
    // disparity = Mat(left.rows, left.cols, CV_32F);

    if (use_ncc) {
        bool flag = ComputeDepthByNCCNew(left, right, bf, points[0], points[1]);
        for (int i = 0; i < points[0].size(); ++i) {
            Point2f start = points[0][i];
            Point2f end = Point2f(points[1][i].x + left.cols, points[1][i].y);
            circle(draw_match, start, 5, Scalar(125, 255, 125), 2);
            circle(draw_match, end, 5, Scalar(125, 255, 125), 2);
            line(draw_match, start, end, Scalar(125, 125, 255));
        }
        if (points[0].size() > 0) {
            imshow("draw_match", draw_match);
            waitKey(0);
            return true;
        }
        else {
            return false;
        }
    }
    else {
        // track
        TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
        // goodFeaturesToTrack(left, points[0], 100, 0.01, 10);
        // cornerSubPix(left, points[0], Size(10,10), Size(-1,-1), termcrit);
        // cout << "we extract " << points[0].size() << " points." << endl;
        vector<uchar> state;
        vector<float> error;
        calcOpticalFlowPyrLK(left, right, points[0], points[1], state, error, 
            Size(31, 31), 3, termcrit, 0, 0.001);
        // calcOpticalFlowPyrLK(left, right, points[0], points[1], state, error, 
        //     Size(21, 21), 5);

        int count = 0;
        vector<Point2f> new_points[2];
        for (int i = 0; i < points[0].size(); ++i) {

            if (state[i] == 0) {
                continue;
            }

            circle(draw_left, points[0][i], 5, Scalar(125, 255, 125));
            circle(draw_right, points[1][i], 5, Scalar(125, 255, 125));
            Point2f start = points[0][i];
            Point2f end = Point2f(points[1][i].x + left.cols, points[1][i].y);
            line(draw_match, start, end, Scalar(125, 125, 255));

            // make depth image
            float d = points[0][i].x - points[1][i].x;
            if (d >= 0) {
                continue;
            }
            int x = points[0][i].x;
            int y = points[0][i].y;
            ++count;
            // disparity.at<float>(y, x) = d;
            // depth.at<float>(y, x) = bf / d;
            // cout << "depth = " << bf / d << endl;

            new_points[0].push_back(points[0][i]);
            new_points[1].push_back(points[1][i]);
        }
        cout << "After KLT, we have " << count  << " points left." << endl;
        cout << endl;

        count = 0;
        vector<uchar> ransac_state;
        findFundamentalMat(new_points[0], new_points[1], ransac_state, cv::FM_RANSAC);
        for (int i = 0; i < ransac_state.size(); ++i) {
            if (ransac_state[i] != 0) {
                ++count;
            }
        }
        cout << "After ransac, we have " << count  << " points left." << endl;
        
        if (count > 0) {
            imshow("draw_left", draw_left);
            imshow("draw_right", draw_right);
            imshow("draw_match", draw_match);
            // imshow("depth", depth);
            waitKey(0);
        } 
        else {
            return false;
        }
    }
    return true;
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
    cout << cols_l << ", " << rows_l << ", " << cols_r << ", " << rows_r << endl;
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

bool ComputeDepthByNCCNew(const Mat& left, const Mat& right, const double& bf, 
    vector<Point2f>& left_pt, vector<Point2f>& right_pt) {

    const int PADDING = 4;
    int u_patchSize = 5;
    int v_patchSize = 3;

    // 遍历提取的所有未成熟点
    vector<Point2f> new_left_pt;
    right_pt.clear();
    for (int i = 0; i < left_pt.size(); ++i) {
            
        Point2f p_l = left_pt[i];
        // 开始极线搜索进行NCC匹配
        float ncc_max = 0, ncc_second_max = 0, ncc_min = 0;
        float u_match = -1;
        float v_match = p_l.y;

        // 由于已经默认双目相机校正过,因此双目图像的极线是水平的
        for (float u = p_l.x-50; u < p_l.x+50; u = u + 1.0) {
            float ncc = 0;
            float x_avg = 0, y_avg = 0;
            float xx = 0, yy = 0, xy = 0;
            float u_half_patch = (float)u_patchSize / 2.0;
            float v_half_patch = (float)v_patchSize / 2.0;
            int count = 0;
            // 计算对应窗口平均值
            for (float du = -u_half_patch; du < u_half_patch; du = du + 1.0) {
                for (float dv = -v_half_patch; dv < v_half_patch; dv = dv + 1.0) {
                    if (p_l.y + dv < PADDING || p_l.y + dv > left.rows - PADDING ||
                        p_l.x + du < PADDING || p_l.x + du > left.cols - PADDING) {
                            continue;
                    }
                    if (v_match + dv < PADDING || v_match + dv > right.rows - PADDING ||
                        u + du < PADDING || u + du > right.cols - PADDING) {
                            continue;
                    }

                    float x = left.at<float>(p_l.y + dv, p_l.x + du);
                    float y = right.at<float>(v_match + dv, u + du);
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
                    if (p_l.y + dv < PADDING || p_l.y + dv > left.rows - PADDING ||
                        p_l.x + du < PADDING || p_l.x + du > left.cols - PADDING) {
                            continue;
                    }
                    if (v_match + dv < PADDING || v_match + dv > right.rows - PADDING ||
                        u + du < PADDING || u + du > right.cols - PADDING) {
                            continue;
                    }

                    float x = left.at<float>(p_l.y + dv, p_l.x + du);
                    float y = right.at<float>(v_match + dv, u + du);

                    xx += x * x;
                    yy += y * y;
                    xy += x * y;
                }
            }
            ncc = xy / sqrt(xx * yy);
            if (ncc > ncc_max) {
                ncc_second_max = ncc_max;
                ncc_max = ncc;
                u_match = u;
            }
            if (ncc < ncc_min) {
                ncc_min = ncc;
            }
        }

        // 利用三角化获取点的逆深度
        // float idepth_match = getIdepthByTriangulate(ph->u, ph->v, u_match, v_match, T_10);
        float delta = p_l.x - u_match;
        float idepth_match = delta / bf;
        float depth = -1.0 / idepth_match;
        cout << "depth = " << depth << endl;

        // 如果ncc太小,或者整体ncc差别太小,则剔除点
        if (ncc_max < 0.90 || (ncc_max - ncc_min) < 0.1 || (ncc_max - ncc_second_max) < 0.001) {
            continue;
        }

        // cout << "depth = " << depth << endl; 
        if (depth <= 0) {
            continue;
        }

        new_left_pt.push_back(p_l);
        right_pt.push_back(Point2f(u_match, v_match));
    }
    left_pt.clear();
    left_pt.insert(left_pt.end(), new_left_pt.begin(), new_left_pt.end());
    cout << "After ncc matching, we have " << left_pt.size() << " points left." << endl;

    return left_pt.size() > 0;
}


