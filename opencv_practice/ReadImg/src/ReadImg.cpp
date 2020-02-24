// c++
#include <iostream>
#include <string>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {

    string default_path = "../data/1.jpg";
    string path;

    if (argc == 2) {
        path = argv[1];
        cout << "point the path is " << path << endl;
    }
    else {
        path = default_path;
        cout << "using the default path is " << path << endl;
    }

    Mat img = imread(path, IMREAD_COLOR);
    if (img.empty()) {
        cout << "empty image input!" << endl;
        return -1;
    }
    imshow("img", img);
    waitKey(0);

    return 0;
}