// Here we perform RGB-guided depth completion. The aim is to use also the informations that are available 
// trought a super pixel identification. The filling procedure will be done based on depth value
// and also based on the superpixel information 
#include <iostream>
#include <vector>
#include "slic.h" 

void check_valid_invalid(cv::Mat &dense_r_img){
    int rows = dense_r_img.rows;
    int cols = dense_r_img.cols;
    int number_of_pixels = rows*cols;
    std::cout << "number of pixels is: " << number_of_pixels << std::endl;
    int number_of_valid_pixels=0;
    int number_of_invalid_pixels=0;
    for(int i = 0; i<rows; i++){
        for(int j = 0; j<cols; j++){
            float depth = dense_r_img.at<float>(i,j);
            if(depth > 0.1){
                number_of_valid_pixels++;
            }
            else{
                number_of_invalid_pixels++;
            }
        }
    }
    std::cout << "number of valid pixels is: " << number_of_valid_pixels << std::endl;

    std::cout << "number of invalid pixels is: " << number_of_invalid_pixels << std::endl;

    std::cout << "TOTAL: " << number_of_pixels - number_of_valid_pixels -number_of_invalid_pixels << std::endl;

}

void interpolate_with_superpixels(Slic& slic,
                                 const cv::Mat &sparse_r_img,
                                 cv::Mat &dense_r_img, 
                                 const std::string &blur_type,
                                 int use_superpixel){
                                 
    float max_depth = 100;
    int rows = sparse_r_img.rows;
    int cols = sparse_r_img.cols;
    dense_r_img = sparse_r_img.clone();    
    //invert values
    for (int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            float depth = dense_r_img.at<float>(i,j);
            if(depth > 0.1){
                dense_r_img.at<float>(i,j) = max_depth - depth;
            }
        }
    }              

    int d[5][5] = { 0, 0, 1, 0, 0, 
                    0, 1, 1, 1, 0, 
                    1, 1, 1, 1, 1, 
                    0, 1, 1, 1, 0, 
                    0, 0, 1, 0, 0};
    if(use_superpixel == 0){
        cv::Mat diamond_kernel_5(5, 5, CV_8UC1, d); 
        cv::dilate(dense_r_img, dense_r_img, diamond_kernel_5);  
        cv::Mat full_kernel_5 = cv::Mat::ones(5, 5, CV_8UC1);
        cv::morphologyEx(dense_r_img, dense_r_img, cv::MORPH_CLOSE, full_kernel_5);
    }
    else{
        //cv::Mat diamond_kernel_5(5, 5, CV_8UC1, d); 
        //cv::dilate(dense_r_img, dense_r_img, diamond_kernel_5);  
        //for(int i=0; i < rows; ++i){
        //    for(int j = 0; j < cols; ++j){
        //        float depth = dense_r_img.at<float>(i, j);
        //        if (depth > 0.1 ){
        //            dense_r_img.at<float>(i, j) = max_depth- dense_r_img.at<float>(i, j);
        //        }
        //    }
        //}
        //cv::Mat full_kernel_5 = cv::Mat::ones(5, 5, CV_8UC1);
        //cv::morphologyEx(dense_r_img, dense_r_img, cv::MORPH_CLOSE, full_kernel_5);
        for(int c = 0; c < (int) slic.centers.size(); c++){
            cv::Mat superpixelMask = cv::Mat::zeros(rows,cols,CV_8UC1);
            for (int i = 0; i < rows; i++){
                for (int j = 0; j < cols; j++){

                    if (slic.clusters[j][i] == c){
                        superpixelMask.at<uchar>(i,j)=255;
                    }

                }
            }
            //std::string sp_number = std::to_string(c);
            //std::string save_path = "../image_sp/" + sp_number + ".png";
            //cv::imwrite(save_path,superpixelMask);
            //cv::imshow("sp_mask",superpixelMask );
            //cv::waitKey(0);
            cv::Mat superpixelRegion;
            dense_r_img.copyTo(superpixelRegion, superpixelMask);

            cv::Mat diamond_kernel_5(5, 5, CV_8UC1, d); 
            cv::dilate(superpixelRegion, superpixelRegion, diamond_kernel_5);  
            cv::Mat full_kernel_5 = cv::Mat::ones(5, 5, CV_8UC1);
            cv::morphologyEx(superpixelRegion, superpixelRegion, cv::MORPH_CLOSE, full_kernel_5);
            superpixelRegion.copyTo(dense_r_img, superpixelMask);
        }
    }
    
    cv::Mat specific_range_img = dense_r_img.clone();
    cv::Mat full_kernel_7 = cv::Mat::ones(7, 7, CV_8UC1);
    cv::dilate(specific_range_img, specific_range_img, full_kernel_7);       
   
    for (int i = 0; i<rows;i++){
        for(int j = 0; j<cols; j++){
                float depth = dense_r_img.at<float>(i,j);
                if (depth<0.1){
                    dense_r_img.at<float>(i,j) = specific_range_img.at<float>(i,j);
                }
        }
    }
  

    int densify = true;
    if (densify){    
        for(int j = 0; j < cols; ++j){
            int max_index = 0;
            float max_val = -1;
            float min_val = 100;
            int min_index = rows - 1;
            for(int i = 0; i < rows; ++i){
                if (dense_r_img.at<float>(i, j) > 0.1){
                    max_index = i; 
                    max_val = dense_r_img.at<float>(i, j);
                };
                if (dense_r_img.at<float>(rows - 1 - i, j) > 0.1){
                    min_index = rows - 1 - i; 
                    min_val = dense_r_img.at<float>(rows - 1 - i, j);
                };
            }
            for(int i = max_index; i < rows; ++i){
                dense_r_img.at<float>(i, j) = max_val;
            }
            for(int i = min_index; i >= 0; --i){
                dense_r_img.at<float>(i, j) = min_val;
            }
        }
    }
    
    cv::Mat full_kernel_31 = cv::Mat::ones(31, 31, CV_8UC1);
    specific_range_img = dense_r_img.clone();
    cv::dilate(specific_range_img, specific_range_img, full_kernel_31);
    int hole_pixel_count = 0;
    for(int i=0; i < rows; ++i){
        for(int j = 0; j < cols; ++j){
            float depth = dense_r_img.at<float>(i, j);
            if (depth < 0.1){
                dense_r_img.at<float>(i, j) = specific_range_img.at<float>(i, j); 
            }
        }
    }
    ///*
    while(densify){
            specific_range_img = dense_r_img.clone();
            cv::dilate(specific_range_img, specific_range_img, full_kernel_31);
            int hole_pixel_count = 0;
            for(int i=0; i < rows; ++i){
                for(int j = 0; j < cols; ++j){
                    float depth = dense_r_img.at<float>(i, j);
                    //std::cout <<depth<< std::endl;
                    if (depth < 0.1){
                        dense_r_img.at<float>(i, j) = specific_range_img.at<float>(i, j); 
                        hole_pixel_count++;
                    }
                }
            }
            //image filled
            std::cout << hole_pixel_count << std::endl;
            if (hole_pixel_count == 0){
                     //std::cout << "ho finito de estrapola" << std::endl;
                break;
                }
        }
    //*/
    cv::medianBlur(dense_r_img, dense_r_img, 5); 

    
    specific_range_img = dense_r_img.clone();
    cv::GaussianBlur(specific_range_img, specific_range_img, cv::Size(5, 5), 0);
    for(int i=0; i < rows; ++i){
        for(int j = 0; j < cols; ++j){
            float depth = dense_r_img.at<float>(i, j);
            if (depth > 0.1){
                dense_r_img.at<float>(i, j) = specific_range_img.at<float>(i, j);
            }
        }
    }
    
    for(int i=0; i < rows; ++i){
        for(int j = 0; j < cols; ++j){
            float depth = dense_r_img.at<float>(i, j);
            if (depth > 0.1){
                dense_r_img.at<float>(i, j) = max_depth - dense_r_img.at<float>(i, j);
                continue;
            }
        }
    }       
}
