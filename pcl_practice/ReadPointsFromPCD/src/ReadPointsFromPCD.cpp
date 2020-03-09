// c++
#include <iostream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv) {

    if (argc != 2) {
        cout << "Please input ./bin/ReadPointsFromPCD ./data/image_pcd.pcd." << endl;
        return 1;
    }

    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>());

    if (io::loadPCDFile<PointXYZRGB>(argv[1], *cloud) == -1) {
        cout << "Could not read pcd from " << argv[1] << endl;
        return 1;
    }

    cout << "Loaded " << cloud->width * cloud->height << " data points from " << argv[1] << " with following fields: " << endl;

    for (int i = 0; i < cloud->points.size(); ++i) {
        PointXYZRGB p = cloud->points[i];
        double x, y, z;
        int r, g, b;
        x = p.x;
        y = p.y;
        z = p.z;
        r = p.r;
        g = p.g;
        b = p.b;
        cout << "id = " << i << ": [x y z] = [" << x << " " << y << " " << z << "], [r g b] = [" << r << " " << g << " " << b << "]" << endl;
    }

    cout << "Finished reading!" << endl;

    return 0;
}