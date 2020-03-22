#include <iostream>

#include <pcl/common/io.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace pcl;
using namespace std;

int main(int argc, char** argv) {

    string in_file = "./data/pointcloud.ply";
    string out_file = "./data/pointcloud.pcd";

    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
    io::loadPLYFile(in_file, *cloud);
    io::savePCDFileASCII(out_file, *cloud);

    cout << "From file : " << in_file << ", we read " << cloud->points.size() << " points!" << endl;
    visualization::CloudViewer viewer("Cloud Viewer");

    viewer.showCloud(cloud);
    while(1) {
        
    }

    return 0;
}