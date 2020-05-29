#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

string dataset = "MH_01_easy";
string data_path = "/home/ypc/xiaoc/dataset/EUROC/";
Mat K_l, K_r, D_l, D_r, P_l, P_r, R_l, R_r;

bool loadCameraParams(Mat& K_left, Mat& K_right, Mat& D_left, Mat& D_right,
     Mat& P_left, Mat& P_right, Mat& R_left, Mat& R_right);

bool readEurocData(const string& filename_left, const string& filename_right, vector<Mat>& left_img, vector<Mat>& right_img);

bool undistortAndRemap(const vector<Mat>& left_input, const vector<Mat>& right_input, vector<Mat>& left_output, vector<Mat>& right_output);

int main(int argc, char* argv[]) {

    if (argc != 3) {
        cout << "Please input ./bin/UndistorAndRemap ./data/filename_left.txt ./data/filename_right.txt." << endl;
        return -1;
    }

    bool flag = loadCameraParams(K_l, K_r, D_l, D_r, P_l, P_r, R_l, R_r);
    if (flag) {
        cout << "Read camera parameters successfully." << endl;
    }

    string leftFile = argv[1];
    string rightFile = argv[2];
    vector<Mat> vLeftImg, vRightImg;
    flag = readEurocData(leftFile, rightFile, vLeftImg, vRightImg);
    if (flag && vLeftImg.size() > 0) {
        imshow("FirstOneImage", vLeftImg[0]);
        waitKey(0);
    }

    vector<Mat> vLeftImgRect, vRightImgRect;
    flag = undistortAndRemap(vLeftImg, vRightImg, vLeftImgRect, vRightImgRect);
    if (flag && vLeftImg.size() > 0) {
        imshow("FirstOneImageRect", vLeftImgRect[0]);
        waitKey(0);
    }

    return 0;
}

bool loadCameraParams(Mat& K_left, Mat& K_right, Mat& D_left, Mat& D_right,
     Mat& P_left, Mat& P_right, Mat& R_left, Mat& R_right) {

    // Euroc
    K_left = (Mat_<float>(3, 3) << 458.654, 0.0, 367.215, 
                                0.0, 457.296, 248.375, 
                                0.0, 0.0, 1.0);
    K_right = (Mat_<float>(3, 3) << 457.587, 0.0, 379.999, 
                                0.0, 456.134, 255.238, 
                                0.0, 0.0, 1.0);

    D_left = (Mat_<float>(1, 5) << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0);
    D_right = (Mat_<float>(1, 5) << -0.28368365, 0.07451284, -0.00010473, -3.555907e-05, 0.0);

    P_left = (Mat_<float>(3, 4) << 435.2046959714599, 0, 367.4517211914062, 0,  
                                0, 435.2046959714599, 252.2008514404297, 0,  
                                0, 0, 1, 0);
    P_right = (Mat_<float>(3, 4) << 435.2046959714599, 0, 367.4517211914062, -47.90639384423901, 
                                0, 435.2046959714599, 252.2008514404297, 0, 
                                0, 0, 1, 0);

    R_left = (Mat_<float>(3, 3) << 0.999966347530033, -0.001422739138722922, 0.008079580483432283, 
                                0.001365741834644127, 0.9999741760894847, 0.007055629199258132, 
                                -0.008089410156878961, -0.007044357138835809, 0.9999424675829176);
    R_right = (Mat_<float>(3, 3) << 0.9999633526194376, -0.003625811871560086, 0.007755443660172947, 
                                0.003680398547259526, 0.9999684752771629, -0.007035845251224894, 
                                -0.007729688520722713, 0.007064130529506649, 0.999945173484644);

    cout << "K_left = \n" << K_left << endl;
    cout << "D_left = \n" << D_left << endl;
    cout << "P_left = \n" << P_left << endl;
    cout << "R_left = \n" << R_left << endl;

    if(K_left.empty() || K_right.empty() || D_left.empty() || D_right.empty() || 
        P_left.empty() || P_right.empty() || R_left.empty() || R_right.empty())
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return false;
    }

    return true;
}

bool readEurocData(const string& filename_left, const string& filename_right, vector<Mat>& left_img, vector<Mat>& right_img) {
    ifstream left_file(filename_left);
    ifstream right_file(filename_right);
    if (!left_file.is_open() || !right_file.is_open()) {
        cout << "Sorry, cannot open file " << filename_left << " or " << filename_right << endl;
        return false;
    }

    string path = data_path + dataset + "/mav0/";

    int count = 0;
    left_img.clear();
    right_img.clear();
    string left_s, right_s;
    getline(left_file, left_s);
    getline(right_file, right_s);
    while (!left_file.eof() || !right_file.eof()) {
        string l_path = path + "cam0/data/" + left_s;
        string r_path = path + "cam1/data/" + right_s;
        cout << "l_path = " << l_path << endl;
        cout << "r_path = " << r_path << endl;
        Mat l_img = imread(l_path.c_str(), CV_LOAD_IMAGE_UNCHANGED);
        Mat r_img = imread(r_path.c_str(), CV_LOAD_IMAGE_UNCHANGED);
        if (l_img.empty() || r_img.empty()) {
            getline(left_file, left_s);
            getline(right_file, right_s);
            continue;
        }
        left_img.push_back(l_img);
        right_img.push_back(r_img);
        getline(left_file, left_s);
        getline(right_file, right_s);

        if (++count > 100) {
            break;
        }
    }

    cout << "Ok, we read " << left_img.size() << " images." << endl;
    return left_img.size() > 0 ? true : false;
}

bool undistortAndRemap(const vector<Mat>& left_input, const vector<Mat>& right_input, vector<Mat>& left_output, vector<Mat>& right_output) {
    if (left_input.size() != right_input.size() || left_input.size() == 0) {
        cout << "Error input dataset." << endl;
        return false;
    }
    int cols_l = left_input[0].cols;
    int rows_l = left_input[0].rows;
    int cols_r = right_input[0].cols;
    int rows_r = right_input[0].rows;
    Mat M1l, M2l, M1r, M2r;
    initUndistortRectifyMap(K_l, D_l, R_l,  P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l), CV_32F, M1l, M2l);
    initUndistortRectifyMap(K_r, D_r, R_r,  P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r), CV_32F, M1r, M2r);

    left_output.clear();
    right_output.clear();
    for (int i = 0; i < left_input.size(); ++i) {
        Mat left_img = left_input[i];
        Mat right_img = right_input[i];

        Mat left_rect, right_rect;
        remap(left_img, left_rect, M1l, M2l, INTER_LINEAR);
        remap(right_img, right_rect, M1r, M2r, INTER_LINEAR);
        left_output.push_back(left_rect);
        right_output.push_back(right_rect);
    }

    cout << "OK, we have finished the pre-proccessing. All images are undistorted and remapped." << endl;
    return left_output.size() > 0 ? true : false; 
}