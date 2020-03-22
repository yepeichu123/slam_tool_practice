// c++
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// pcl
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#define IMG_NUM 12

using namespace cv;
using namespace std;

void ComputeRelativePose(const Mat &ref_img, const Mat &cur_img, const Mat &K, Mat &T);

int main(int argc, char** argv) {

    string in_path = "./data/";
    string out_file = "./data/pcd/results.pcd";

    string rgb_path = in_path + "rgb/";
    string depth_path = in_path + "depth/";
    vector<Mat> rgb_img, depth_img;
    for (int i = 1; i <= IMG_NUM; ++i) {
        string rgb_file, depth_file;
        stringstream ss;
        ss << rgb_path << i << ".png";
        ss >> rgb_file;
        ss.clear();
        ss << depth_path << i << ".png";
        ss >> depth_file;
        ss.clear();

        Mat rgb = imread(rgb_file, IMREAD_COLOR);
        Mat depth = imread(depth_file, IMREAD_GRAYSCALE);
        if (rgb.empty() || depth.empty()) {
            cout << "Please go to the source dir, and input ./bin/ConcatePointCloud." << endl;
            return 1;
        }
        rgb_img.push_back(rgb);
        depth_img.push_back(depth);
    }
    cout << "Here we have " << rgb_img.size() << " images!" << endl;

    Mat K;
    K = (Mat_<float>(3,3) << 525.0, 0, 319.5,
                             0, 525.0, 239.5,
                             0, 0, 1);
    double scale = 5000.0;

    Mat img1 = rgb_img[0];
    Mat img2 = rgb_img[1];
    Mat T;
    cout << "Enter ComputeRelativePose!" << endl;
    ComputeRelativePose(img1, img2, K, T);


    return 0;
}

void ComputeRelativePose(const Mat &ref_img, const Mat &cur_img, const Mat &K, Mat &T) {

    if (ref_img.empty() || cur_img.empty()) {
        cout << "Error input images. Please check it again." << endl;
        return;
    }

    // feature extracting
    Ptr<ORB> orb = ORB::create();
    vector<KeyPoint> ref_kpt, cur_kpt;
    Mat ref_desp, cur_desp;
    orb->detectAndCompute(ref_img, Mat(), ref_kpt, ref_desp);
    orb->detectAndCompute(cur_img, Mat(), cur_kpt, cur_desp);

    // feature matching 
    vector<DMatch> matches;
    Ptr<BFMatcher> bfmatch = BFMatcher::create(NORM_HAMMING2);
    bfmatch->match(ref_desp, cur_desp, matches);

    // find good matching 
    double min_dist = min_element(matches.begin(), matches.end(), 
        [](DMatch &m1, DMatch &m2) {
            return m1.distance < m2.distance;
        })->distance;
    vector<DMatch> good_matches;
    for (auto m : matches) {
        if (m.distance < 2*min_dist) {
            good_matches.push_back(m);
        }
    }
    Mat out_img;
    drawMatches(ref_img, ref_kpt, cur_img, cur_kpt, good_matches, out_img);
    imshow("matching_img", out_img);
    waitKey(0);

    // compute relative pose 
    vector<Point2f> ref_pixel, cur_pixel;
    vector<Point3f> ref_cam, cur_cam;
    for (int i = 0; i < good_matches.size(); ++i) {
        Point2f ref_p = ref_kpt[good_matches[i].queryIdx].pt;
        Point2f cur_p = cur_kpt[good_matches[i].trainIdx].pt;
        ref_pixel.push_back(ref_p);
        cur_pixel.push_back(cur_p);

        Point3f ref_c(
            (ref_p.x - K.at<float>(0, 2) / K.at<float>(0, 0)),
            (ref_p.y - K.at<float>(1, 2) / K.at<float>(1, 1)),
            1
        );
        Point3f cur_c(
            (cur_p.x - K.at<float>(0, 2) / K.at<float>(0, 0)),
            (cur_p.y - K.at<float>(1, 2) / K.at<float>(1, 1)),
            1
        );
        ref_cam.push_back(ref_c);
        cur_cam.push_back(cur_c);
    } 

    // find essential matrix 
    Mat R, t;
    Mat E = findEssentialMat(ref_pixel, cur_pixel, K, 8, 0.999, 1.0);
    recoverPose(E, ref_pixel, cur_pixel, K, R, t);
    cout << "R = " << R << endl;
    cout << "t = " << t << endl;

    // compute tx
    Mat new_t;
    t.convertTo(new_t, CV_32F);
    Mat tx = (Mat_<float>(3, 3) << 0, -new_t.at<float>(2, 0), new_t.at<float>(1, 0),
                               new_t.at<float>(2, 0), 0, -new_t.at<float>(0, 0),
                               -new_t.at<float>(1, 0), new_t.at<float>(0, 0), 0);
    cout << "tx = " << tx << endl;
    // check error 
    float err = 0;
    Mat new_R;
    R.convertTo(new_R, CV_32F);
    Mat new_E = tx * new_R;
    for (int i = 0; i < good_matches.size(); ++i) {
        Mat ref_p3d, cur_p3d;
        ref_p3d = (Mat_<float>(3,1) << ref_cam[i].x, ref_cam[i].y, ref_cam[i].z);
        cur_p3d = (Mat_<float>(3,1) << cur_cam[i].x, cur_cam[i].y, cur_cam[i].z);
        Mat e = cur_p3d.t() * new_E * ref_p3d;
        cout << i << " : " << e << endl;
        err = norm(e);
    }
    cout << "average error is " << err / good_matches.size() << endl;

    T = Mat::eye(4, 4, CV_32F);
    T(Range(0,3), Range(0,3)) = R(Range(0,3), Range(0,3));
    T(Range(0,3), Range(3,4)) = t(Range(0,3), Range(0,1));
    cout << "R = " << R << "\n, t = " << t << endl;
    cout << "T = " << T << endl;
}