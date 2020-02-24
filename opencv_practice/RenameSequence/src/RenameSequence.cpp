#include "RenameSequence.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <iomanip>

RenameSequence::RenameSequence() {
    std::cout << "进入 RenameSequence 默认构造函数.\n" << std::endl;
}

RenameSequence::~RenameSequence() {
    std::cout << "进入 RenameSequence 析构函数.\n" << std::endl;
}

bool RenameSequence::RunRename(const std::string &input_path, const std::string &output_path) {
    std::cout << "进入 RunRename 函数.\n" << std::endl;
    
    std::string file_name = "rgb.txt";
    std::string input_name = input_path + "/" + file_name;
    std::cout << "文件夹路径为 " << input_name << std::endl;
    std::ifstream find;
    find.open(input_name);
    if (!find.is_open()) {
        std::cout << "文件打开失败，请确认路径！" << std::endl;
        return false;
    }

    std::string line;
    int index = 0;
    while (index++ < 3) {
        std::getline(find, line);
    }

    int count = 0;
    std::vector<std::string> out_index;
    std::vector<std::string> out_img_name;

    while (std::getline(find, line) && count < 10) {
        cv::Mat img;
        std::string output_index, output_file;
        if (ReadImg(input_path, line, img)) {
            if (WriteImg(count, img, output_path, output_index, output_file)) {
                out_index.push_back(output_index);
                out_img_name.push_back(output_file);
                ++count;
            }
        }
    }
    find.close();

    std::string out_file_name = output_path + "/new_rgb.txt";
    std::ofstream fout;
    fout.open(out_file_name);
    for (size_t i = 0; i < out_index.size(); ++i) {
        fout << out_index[i] << " " << out_img_name[i] << std::endl;
        std::cout << out_index[i] << " " << out_img_name[i] << std::endl;
    }
    fout.close();
}

// 读取图像
bool RenameSequence::ReadImg(const std::string &input_path, 
    const std::string &file_name, cv::Mat &out_img) {

    std::stringstream ss;
    ss << file_name;

    double in_time;
    std::string in_name;
    ss >> in_time;
    ss >> in_name;

    std::string in_img_path = input_path + "/" + in_name;
    out_img = cv::imread(in_img_path, 1);

    if (out_img.empty()) {
        return false;
    }
    return true;
}

// 输出图像
bool RenameSequence::WriteImg(const int &index, cv::Mat &in_img, const std::string &output_path,
    std::string &output_index, std::string &output_file) {

    if (in_img.empty()) {
        std::cout << "输入writeImg函数的图像为空！" << std::endl;
        return false;
    }

    std::stringstream ss;
    ss << std::setw(4) << std::setfill('0') << index;
    output_index = ss.str();
    output_file = std::string("rgb/") + output_index + ".png";
    std::string out_img_path = output_path + "/" + output_file;

    bool flag = cv::imwrite(out_img_path.c_str(), in_img);
    return flag;
}