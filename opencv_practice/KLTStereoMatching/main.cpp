#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

bool readEurocData(const string& filename, vector<Mat>& left_img, vector<Mat>& right_img);

int main(int argc, char** argv) {

    // read images 
    if (argc != 3) {
        cout << "Please input ./build/KLTStereoMatching ./data/left.png ./data/right.png" << endl;
        return -1;
    }
    Mat left = imread(argv[1], IMREAD_GRAYSCALE);
    Mat right = imread(argv[2], IMREAD_GRAYSCALE);

    // camera intrinsics and baseline 
    // euroc
    Mat K = (Mat_<float>(3, 3) << 458.654, 0, 367.215, 0, 457.296, 248.375, 0, 0, 1);
    float baseline = -0.11007;
    // kitti
    // Mat K = (Mat_<float>(3, 3) << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1);
    // float baseline = 0.537150653267924;
    float bf = baseline * K.at<float>(0, 0);

    // feature extracting
    vector<Point2f> points[2];
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    // goodFeaturesToTrack(left, points[0], 100, 0.01, 10);
    // cornerSubPix(left, points[0], Size(10,10), Size(-1,-1), termcrit);
    // cout << "we extract " << points[0].size() << " points." << endl;

    vector<KeyPoint> kpts;
    Mat desp;
    Ptr<ORB> orb = ORB::create(100);
    orb->detectAndCompute(left, Mat(), kpts, desp);
    // convert00
    for (int i = 0; i < kpts.size(); ++i) {
        points[0].push_back(kpts[i].pt);
    }
    cout << "we extract " << points[0].size() << " points." << endl;


    // track
    vector<uchar> state;
    vector<float> error;
    // calcOpticalFlowPyrLK(left, right, points[0], points[1], state, error, 
        // Size(31, 31), 3, termcrit, 0, 0.001);
    calcOpticalFlowPyrLK(left, right, points[0], points[1], state, error, 
        Size(21, 21), 5);

    // imshow
    Mat draw_left = left.clone();
    Mat draw_right = right.clone();
    Mat draw_match = Mat(Size(2*left.cols, left.rows), CV_8UC1);
    draw_left.copyTo(draw_match(Rect(0, 0, left.cols, left.rows)));
    draw_right.copyTo(draw_match(Rect(left.cols, 0, left.cols, left.rows)));
    Mat depth, disparity;
    depth = Mat(left.rows, left.cols, CV_32F);
    disparity = Mat(left.rows, left.cols, CV_32F);
    
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
        disparity.at<float>(y, x) = d;
        depth.at<float>(y, x) = bf / d;
        // cout << "depth = " << bf / d << endl;

        new_points[0].push_back(points[0][i]);
        new_points[1].push_back(points[1][i]);
    }
    cout << "count = " << count << endl;
    cout << endl;

    count = 0;
    vector<uchar> ransac_state;
    findFundamentalMat(new_points[0], new_points[1], ransac_state, cv::FM_RANSAC);
    for (int i = 0; i < ransac_state.size(); ++i) {
        if (ransac_state[i] != 0) {
            ++count;
        }
    }
    cout << "count = " << count << endl;

    imshow("draw_left", draw_left);
    imshow("draw_right", draw_right);
    imshow("draw_match", draw_match);
    imshow("depth", depth);
    waitKey(0);


    return 0;
}

bool readEurocData(const string& filename, vector<Mat>& left_img, vector<Mat>& right_img) {

}