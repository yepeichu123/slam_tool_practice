#include "RenameSequence.h"

#include <iostream>
#include <string>
#include <fstream>
using namespace std;

int main(int argc, char** argv) {

    if (argc < 3) {
        cout << "请输入应用程序， 图像数据集路径以及输出文件路径！" << endl;
        return -1;
    }
    string input_path = argv[1];
    string output_path = argv[2];

    RenameSequence my_rs;
    my_rs.RunRename(input_path, output_path);

    return 0;
}