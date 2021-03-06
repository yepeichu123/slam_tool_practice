// c++
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// pcl
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#define IMG_NUM 12

using namespace cv;
using namespace std;
using namespace pcl;

void ReadImagesFromFiles(const string &in_path, vector<Mat> &rgb_img, vector<Mat> &depth_img);

void ComputeRelativePose(const Mat &ref_img, const Mat &ref_depth, const Mat &K, Mat &T);

void ComputeRelativePosePNP(const Mat &ref_img, const Mat &ref_depth, const float &scale, const Mat &cur_img, const Mat &K, Mat &r_vec, Mat &t_vec);

PointCloud<PointXYZRGB>::Ptr getPointCloud(const Mat &rgb_img, const Mat &depth_img, const Mat &K, const float &scale);

int main(int argc, char** argv) {

    if (argc != 2) {
        cout << "Please go to the source dir, and input ./bin/ConcatePointCloud. ./data/!" << endl;
        return 1;
    }
    string in_path = argv[1];
    string out_file = in_path + "result/results.pcd";

    // read images from files
    vector<Mat> rgb_img, depth_img;
    ReadImagesFromFiles(in_path, rgb_img, depth_img);

    // compute relative poses
    Mat K;
    K = (Mat_<float>(3,3) << 518.0, 0, 325.5,
                             0, 519.0, 253.5,
                             0, 0, 1);
    float scale = 1000.0;   

    Eigen::Isometry3f T_rw = Eigen::Isometry3f::Identity();
    PointCloud<PointXYZRGB>::Ptr cloud_1 = getPointCloud(rgb_img[0], depth_img[0], K, scale);
    PointCloud<PointXYZRGB>::Ptr ref_cloud(new PointCloud<PointXYZRGB>());
    transformPointCloud(*cloud_1, *ref_cloud, T_rw.matrix());

    for (int i = 1; i < rgb_img.size(); ++i) {
        Mat ref_img = rgb_img[i-1];
        Mat ref_img_depth = depth_img[i-1];
        Mat cur_img = rgb_img[i];
        Mat cur_img_depth = depth_img[i];

        Mat r_vec, t_vec;
        ComputeRelativePosePNP(ref_img, ref_img_depth, scale, cur_img, K, r_vec, t_vec);
        Mat R_cr;
        Rodrigues(r_vec, R_cr);
        Eigen::Matrix3f r_cr;
        cv2eigen(R_cr, r_cr);
        Eigen::Isometry3f T_cr;
        Eigen::AngleAxisf angle(r_cr);
        T_cr = angle;
        T_cr(0, 3) = t_vec.at<float>(0);
        T_cr(1, 3) = t_vec.at<float>(1);
        T_cr(2, 3) = t_vec.at<float>(2);
        PointCloud<PointXYZRGB>::Ptr out_cloud(new PointCloud<PointXYZRGB>());
        PointCloud<PointXYZRGB>::Ptr cloud_cur = getPointCloud(cur_img, cur_img_depth, K, scale);
        transformPointCloud(*ref_cloud, *out_cloud, T_cr.matrix());
        *cloud_cur += *out_cloud;

        ref_cloud = cloud_cur;
    }

    io::savePCDFileASCII(out_file, *ref_cloud);

    visualization::CloudViewer viewer("CloudViewer");
    viewer.showCloud(ref_cloud);
    while (!viewer.wasStopped())
    {
        /* code */
    }
    
    return 0;
}

void ReadImagesFromFiles(const string &in_path, vector<Mat> &rgb_img, vector<Mat> &depth_img) {
    string rgb_path = in_path + "rgb/";
    string depth_path = in_path + "depth/";

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
            cout << "Please check if you input the correct path." << endl;
            return;
        }
        rgb_img.push_back(rgb);
        depth_img.push_back(depth);
    }
    cout << "Here we have " << rgb_img.size() << " images!" << endl;
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
        if (m.distance <  max(30.0, 2*min_dist)) {
            good_matches.push_back(m);
        }
    }
    Mat out_img;
    drawMatches(ref_img, ref_kpt, cur_img, cur_kpt, good_matches, out_img);
    imshow("matching_img", out_img);
    waitKey(1);

    // compute relative pose 
    vector<Point2f> ref_pixel, cur_pixel;
    vector<Point3f> ref_cam, cur_cam;
    for (int i = 0; i < good_matches.size(); ++i) {
        Point2f ref_p = ref_kpt[good_matches[i].queryIdx].pt;
        Point2f cur_p = cur_kpt[good_matches[i].trainIdx].pt;
        ref_pixel.push_back(ref_p);
        cur_pixel.push_back(cur_p);

        Point3f ref_c(
            (ref_p.x - K.at<float>(0, 2)) / K.at<float>(0, 0),
            (ref_p.y - K.at<float>(1, 2)) / K.at<float>(1, 1),
            1
        );
        Point3f cur_c(
            (cur_p.x - K.at<float>(0, 2)) / K.at<float>(0, 0),
            (cur_p.y - K.at<float>(1, 2)) / K.at<float>(1, 1),
            1
        );
        ref_cam.push_back(ref_c);
        cur_cam.push_back(cur_c);
    } 

    // find essential matrix 
    Mat R, t;
    Mat E = findEssentialMat(ref_pixel, cur_pixel, K, 8, 0.999, 1.0);
    recoverPose(E, ref_pixel, cur_pixel, K, R, t);

    // compute tx
    Mat new_t;
    t.convertTo(new_t, CV_32F);
    Mat tx = (Mat_<float>(3, 3) << 0, -new_t.at<float>(2, 0), new_t.at<float>(1, 0),
                               new_t.at<float>(2, 0), 0, -new_t.at<float>(0, 0),
                               -new_t.at<float>(1, 0), new_t.at<float>(0, 0), 0);
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
        err = norm(e);
    }
    cout << "average error is " << err / good_matches.size() << endl;

    T = Mat::eye(4, 4, CV_32F);
    new_R.rowRange(0,3).colRange(0,3).copyTo(T(Rect(0,0,3,3)));
    new_t.rowRange(0,3).copyTo(T(Rect(3,0,1,3)));
    cout << "T = " << T << endl;
}

void ComputeRelativePosePNP(const Mat &ref_img, const Mat &ref_depth, const float &scale, const Mat &cur_img, const Mat &K, Mat &r_vec, Mat &t_vec) {

    if (ref_img.empty() || cur_img.empty()) {
        cout << "Error input images. Please check it again." << endl;
        return;
    }

    cout << "Feature extracting." << endl;
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
    cout << "we have " << matches.size() << " matching pairs firstly." << endl;
 
    // find good matching 
    double min_dist = min_element(matches.begin(), matches.end(), 
        [](DMatch &m1, DMatch &m2) {
            return m1.distance < m2.distance;
        })->distance;
    vector<DMatch> good_matches;
    for (auto m : matches) {
        if (m.distance < max(30.0, 2*min_dist)) {
            good_matches.push_back(m);
        }
    }
    cout << "Here we found " << good_matches.size() << " matching pairs!" << endl;
    Mat out_img;
    drawMatches(ref_img, ref_kpt, cur_img, cur_kpt, good_matches, out_img);
    imshow("matching_img", out_img);
    waitKey(1);

    // compute relative pose 
    vector<Point2f> ref_pixel, cur_pixel;
    vector<Point3f> ref_cam, cur_cam;
    for (int i = 0; i < good_matches.size(); ++i) {
        Point2f ref_p = ref_kpt[good_matches[i].queryIdx].pt;
        Point2f cur_p = cur_kpt[good_matches[i].trainIdx].pt;
        // check depth 
        ushort d = ref_depth.at<ushort>(ref_p.y, ref_p.x);
        if (d == 0) {
            continue;
        }
        float z = d / scale;
        ref_pixel.push_back(ref_p);
        cur_pixel.push_back(cur_p);

        Point3f ref_c(
            (ref_p.x - K.at<float>(0, 2)) * z / K.at<float>(0, 0),
            (ref_p.y - K.at<float>(1, 2)) * z / K.at<float>(1, 1),
            z
        );
        Point3f cur_c(
            (cur_p.x - K.at<float>(0, 2)) / K.at<float>(0, 0),
            (cur_p.y - K.at<float>(1, 2)) / K.at<float>(1, 1),
            1
        );
        ref_cam.push_back(ref_c);
        cur_cam.push_back(cur_c);
    } 
    cout << "There are " << ref_cam.size() << " valuable matching pairs!" << endl;

    // find essential matrix 
    solvePnPRansac(ref_cam, cur_pixel, K, Mat(), r_vec, t_vec, false, 100, 1.0, 0.999);
    Mat R;
    Rodrigues(r_vec, R);

    Mat new_t, new_R;
    t_vec.convertTo(new_t, CV_32F);    
    R.convertTo(new_R, CV_32F);
    t_vec = new_t;

    // check error 
    float err = 0;
    for (int i = 0; i < ref_cam.size(); ++i) {
        Mat ref_p3d, cur_p3d;
        ref_p3d = (Mat_<float>(3,1) << ref_cam[i].x, ref_cam[i].y, ref_cam[i].z);
        cur_p3d = (Mat_<float>(3,1) << cur_cam[i].x, cur_cam[i].y, cur_cam[i].z);
        Mat e = cur_p3d - (new_R * ref_p3d + new_t);
        err = norm(e);
    }
    cout << "average error is " << err / ref_cam.size() << endl;

    Mat T_cr = Mat::eye(4, 4, CV_32F);
    new_R.rowRange(0,3).colRange(0,3).copyTo(T_cr(Rect(0,0,3,3)));
    new_t.rowRange(0,3).copyTo(T_cr(Rect(3,0,1,3)));
    cout << "T_cr = " << T_cr << endl;

    
    Mat T_rc = Mat::eye(4, 4, CV_32F);
    new_R = new_R.t();
    new_t = -new_R * new_t;
    new_R.rowRange(0,3).colRange(0,3).copyTo(T_rc(Rect(0,0,3,3)));
    new_t.rowRange(0,3).copyTo(T_rc(Rect(3,0,1,3)));
    cout << "T_rc = " << T_rc << endl;
}

PointCloud<PointXYZRGB>::Ptr getPointCloud(const Mat &rgb_img, const Mat &depth_img, const Mat &K, const float &scale) {
    if (rgb_img.empty() || depth_img.empty()) {
        cout << "error images!" << endl;
        return nullptr;
    }

    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>());
    for (int y = 0; y < rgb_img.rows; ++y) {
        for (int x = 0; x < rgb_img.cols; ++x) {
            ushort d = depth_img.ptr<ushort>(y)[x];
            if (d == 0) {
                continue;
            }
            float z = (float)d / scale;
            Vec3b color = rgb_img.ptr<Vec3b>(y)[x];
            PointXYZRGB point;
            point.x = (x - K.at<float>(0, 2)) * z / K.at<float>(0, 0);
            point.y = (y - K.at<float>(1, 2)) * z / K.at<float>(1, 1);
            point.z = z;
            point.r = color[2];
            point.g = color[1];
            point.b = color[0];
            cloud->points.push_back(point);
        }
    }
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cloud->is_dense = false;

    return cloud;
}