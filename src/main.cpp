#include "img_completion.h"
#include <iostream>
#include <string>

void toColorImage(const cv::Mat &r_img, cv::Mat &color_img)
{
    cv::Mat normalize_dense_r_img;
    cv::normalize(r_img, normalize_dense_r_img, 1.0, 0, cv::NORM_MINMAX);
    // CV_8UC1 -> a 8-bit single-channel array
    cv::Mat dense_r_img_CV_8UC1;
    normalize_dense_r_img.convertTo(dense_r_img_CV_8UC1, CV_8UC1, 255.0);
    cv::applyColorMap(dense_r_img_CV_8UC1, color_img, cv::COLORMAP_JET);
}




void get_vals(cv::Mat &image_r){
    int rows = image_r.rows;
    std::cout<< "number of rows is " << rows << std::endl;
    int cols = image_r.cols;
    std::cout<< "number of cols is " << cols << std::endl;
    int max_depth = 0;
    for (int i = 0; i<rows;i++){
        for (int j = 0; j<cols;j++){
           
            if (image_r.at<float>(i, j)> max_depth){
                max_depth = image_r.at<float>(i, j);
            }
           
        }
    }
    std::cout<< "max value of depth is " << max_depth +1<< std::endl;
    cv::imshow("src", image_r);
    cv::waitKey(0);

}
int main(int argc, char **argv){

    std::string image_raw;
    cv::Mat image_sparse;
    cv::Mat image_r;

    //image_raw="../depth_selection/val_selection_cropped/velodyne_raw/2011_09_26_drive_0005_sync_velodyne_raw_0000000047_image_03.png";
    image_raw = "../depth_selection/val_selection_cropped/velodyne_raw/2011_09_26_drive_0002_sync_velodyne_raw_0000000005_image_02.png";
    image_r = cv::imread(image_raw, cv::IMREAD_ANYDEPTH);
    cv::Mat projected_depths;
    image_r.convertTo(projected_depths, CV_32F, 1.0 / 256.0);
   
    get_vals(projected_depths);
    std::string blur_type = std::string("gaussian");
    bool extr = true;

    cv::Mat dense_range_img;
    cv::Mat range_img;

    img_completion(projected_depths, dense_range_img, extr, blur_type);

    cv::Mat color_dense_range_img;

    toColorImage(dense_range_img, color_dense_range_img);

    // show
    std::vector<cv::Mat> hImg;

    hImg.push_back(color_dense_range_img);
    cv::Mat cat_img;
    cv::vconcat(hImg, cat_img);
    cv::imshow("src", cat_img);
    cv::waitKey(0);
    std::cout << "DONE" << std::endl;

    return 0;
}