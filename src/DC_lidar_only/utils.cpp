#include <iostream>
#include <string>
#include "img_completion.h"


void toColorImage(const cv::Mat &r_img, cv::Mat &color_img){
    cv::Mat normalize_dense_r_img;
    cv::normalize(r_img,normalize_dense_r_img,1.0,0,cv::NORM_MINMAX);
    //CV_8UC1 -> a 8-bit single-channel array
    cv::Mat dense_r_img_CV_8UC1;
    normalize_dense_r_img.convertTo(dense_r_img_CV_8UC1, CV_8UC1, 255.0);
    cv::applyColorMap(dense_r_img_CV_8UC1, color_img, cv::COLORMAP_JET);
}

void read_M (const std::string &filename, cv::Mat &mat){
    int rows, cols, data, depth,type, channels;
    //check
    std::ifstream file(filename, std::fstream::binary);
    if (!file.is_open())
        return;
    try {
        std::cout << "size of int = " << sizeof(int) << std::endl;
        file.read(reinterpret_cast<char *>(&rows), sizeof(rows));
        file.read(reinterpret_cast<char *>(&cols), sizeof(cols));
        file.read(reinterpret_cast<char *>(&depth), sizeof(depth));
        file.read(reinterpret_cast<char *>(&type), sizeof(type));
        file.read(reinterpret_cast<char *>(&channels), sizeof(channels));
        file.read(reinterpret_cast<char *>(&data), sizeof(data));
        std::cout << "rows = " << rows << "cols = " << cols << "type = " << type << "channels = " << channels << std::endl; 
        mat = cv::Mat(rows, cols, type);
        file.read(reinterpret_cast<char *>(mat.data), data);
    } catch (...) {
        file.close();
        return;
    }
    file.close();
}

void write_M(const std::string& filename, const cv::Mat& mat){

    // cv::Mat --> save as .bin file
    std::ofstream file;
    file.open (filename, std::fstream::binary);
    if (!file.is_open())
        return ;
    file.write(reinterpret_cast<const char *>(&mat.rows), sizeof(int));
    file.write(reinterpret_cast<const char *>(&mat.cols), sizeof(int));
    const int depth = mat.depth();
    const int type  = mat.type();
    const int channels = mat.channels();
    file.write(reinterpret_cast<const char *>(&depth), sizeof(depth));
    file.write(reinterpret_cast<const char *>(&type), sizeof(type));
    file.write(reinterpret_cast<const char *>(&channels), sizeof(channels));
    int sizeInBytes = mat.step[0] * mat.rows;
    file.write(reinterpret_cast<const char *>(&sizeInBytes), sizeof(int));
    file.write(reinterpret_cast<const char *>(mat.data), sizeInBytes);
    file.close();
}