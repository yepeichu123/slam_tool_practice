# RenameSequence————Designed by 叶培楚

1. 编译：
            cd ${PROJECT_SOURCE_DIR}/build
            cmake ..
            make 
2. 运行方法：
            ${PROJECT_SOURCE_DIR}/bin/RenameSequence \
            ${PATH_TO_TUM_DATASET} (比如 ../dataset/TUM/rgbd_dataset_freiburg1_desk，最后不用斜杠) \
            ../output_data (最后也不用斜杠)

3. 代码思路：
            a. 读图像程序：从指定数据库文件夹中的rgb.txt文件中，以时间戳读取图像；
            b. 写图像程序：用0000-9999作为文件名，按照序号将读取的图像存入指定输出路径，并保存新的new_rgb.txt文件夹。

ps：由于本程序只用于测试，为防止运行时间过长，设置了count计数器，读取10张图后就停止！
    为了便于读者直接使用，笔者直接将程序封装成类，调用非常方便。
