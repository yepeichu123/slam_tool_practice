// c++
#include <iostream>
#include <string>
#include <vector>
// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace cv;
using namespace pcl;

int main(int argc, char** argv) {

    if (argc != 3) {
        cout << "Please input ./bin/ReadPointsFromImg ./data/rgb.png ./data/depth.png." << endl;
        return 1;
    }

    // 读取图像以及显示图像
    Mat rgb = imread(argv[1], IMREAD_COLOR);
    Mat depth = imread(argv[2], IMREAD_GRAYSCALE);
    imshow("rgb_img", rgb);
    waitKey(0);

    // 相机内参数
    double fx = 525.0, fy = 525.0;
    double cx = 319.5, cy = 239.5;
    double scale = 5000.0;

    // 创建点云数据结构
    PointCloud<PointXYZRGB>::Ptr pointcloud(new PointCloud<PointXYZRGB>());
    int row = rgb.rows;
    int col = rgb.cols;
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            // 读取彩色信息
            uchar b, g, r;
            b = rgb.at<Vec3b>(i, j)[0];
            g = rgb.at<Vec3b>(i, j)[1];
            r = rgb.at<Vec3b>(i, j)[2];

            // 读取深度信息及重建三维位置
            ushort d = depth.at<ushort>(i, j);
            if (d <= 0) {
                continue;
            }
            double z = d / scale;
            double x = (j - cx) * z / fx;
            double y = (i - cy) * z / fy;
            
            // 创建点云数据
            PointXYZRGB p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.r = r;
            p.g = g;
            p.b = b;
            pointcloud->push_back(p);
        }
    }
    cout << "Read " << pointcloud->points.size() << " points!" << endl;

    // 保存点云
    io::savePCDFileASCII("./data/image_pcd.pcd", *pointcloud);

    // 显示点云信息
    visualization::CloudViewer viewer("viewer_points");
    while (1) {
        viewer.showCloud(pointcloud);
    }

    return 0;
}