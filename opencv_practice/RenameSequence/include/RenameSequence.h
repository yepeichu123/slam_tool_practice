#ifndef RENAME_SEQUENCE_H
#define RENAME_SEQUENCE_H

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class RenameSequence {
    public:
        // 默认构造函数
        RenameSequence();

        // 默认析构函数
        ~RenameSequence();

        // 运行重命名
        // input_path 为输入的TUM数据集路径，图像均由时间戳命名
        // output_path 为输出的文件夹路径，图像均由0000-9999中的顺序命名
        bool RunRename(const std::string &input_path, const std::string &output_path);

    private:

        // 从硬盘中读取图像
        bool ReadImg(const std::string &input_path,
                     const std::string &file_name,
                     cv::Mat &out_img);

        // 重命名图像并输出到硬盘中
        bool WriteImg(const int &index,
                      cv::Mat &in_img,
                      const std::string &output_path,
                      std::string &output_index,
                      std::string &output_file);

};


#endif // RENAME_SEQUENCE_H