// c++
#include <iostream>
#include <vector>
// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// pcl
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace cv;
using namespace std;

void ComputeRelativePose(const Mat &ref_img, const Mat &cur_img, const Mat &K, Mat &R, Mat &t);

int main(int argc, char** argv) {

    return 0;
}

void ComputeRelativePose(const Mat &ref_img, const Mat &cur_img, const Mat &K, Mat &R, Mat &t) {

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
        if (m.distance < max(30.0, min_dist*2)) {
            good_matches.push_back(m);
        }
    }

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
    Mat E = findEssentialMat(ref_pixel, cur_pixel, K, 8, 0.999, 1.0);
    recoverPose(E, ref_pixel, cur_pixel, K, R, t);

    // compute tx
    Mat tx;
    tx = (Mat_<float>(3, 3) << 0, -t.at<float>(2), t.at<float>(1),
                               t.at<float>(2), 0, -t.at<float>(0),
                               -t.at<float>(1), t.at<float>(0), 0);

    // check error 
    float err = 0;
    for (int i = 0; i < good_matches.size(); ++i) {
        Mat ref_p3d, cur_p3d;
        ref_p3d = (Mat_<float>(3,1) << ref_cam[i].x, ref_cam[i].y, ref_cam[i].z);
        cur_p3d = (Mat_<float>(3,1) << cur_cam[i].x, cur_cam[i].y, cur_cam[i].z);
        Mat e = cur_p3d.t() * tx * R * ref_p3d;
        cout << i << " : " << e << endl;
        err = norm(e);
    }
    cout << "average error is " << err / good_matches.size() << endl;

}