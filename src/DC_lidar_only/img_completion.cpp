//step to follows are straightforward.
// given a sparse and accurate estimate of the lidar, and a dense image, create a dense depth estimate.
//In the firs approach we assume that the relative transform between the 2 sensors is not a problem 
//since we already have the lidar projected into the image plane. In general this is not true 
//and in order to properly have needed matching, we must transform the lidars output into the image 
//plane. The main problem to start with is how to 'fill' empty pixels. There are different ideas,
//which exploits different features of the image and can be more or less elaborated. In the first
//stage a simple morphological operator applied starting from the values already available on the depth is 
//enough. For further development I should also include everything with ROS so that the real values coming 
//from the bastone can be used. The second approach, slightly more sophisticated uses normals and 
//outliers rejection. 
//Once this first step is achieved, I'll focus on refining the estimate using the other camera.
//This can be carried out in two different ways. Self supervised or 'probabiliscal' (i.e. use the epipolar constraint)
//and estimate the photometric loss patch-wise. But for know these last two approaches are not to be taken into account
#include "img_completion.h"

void img_completion(const cv::Mat &sparse_r_img, 
                    cv::Mat &dense_r_img, 
                    const bool &extr, 
                    const std::string &blur_type){

    float max_r_img_val = 0.0;
    float max_depth = 100.0;
    int rows = sparse_r_img.rows;
    int cols = sparse_r_img.cols;
    //densify
    dense_r_img = sparse_r_img.clone();

    std::cout << "NUMERO ROWS, COLS: " << rows << " " << cols << std::endl; 

    //for (int i = 0; i<rows; i++){
    //    for (int j = 0; j<cols; j++){
    //        dense_r_img.at<float>(i,j)=dense_r_img.at<float>(i,j)/256;
    //       // std::cout<<depth<<std::endl;
    //        if (dense_r_img.at<float>(i,j)<0){
    //            dense_r_img.at<float>(i,j)=0;
    //        }
    //    }
    //}

    for (int i = 0; i<rows; i++){
        for (int j = 0; j<cols; j++){
            float depth = dense_r_img.at<float>(i,j);
           // std::cout<<depth<<std::endl;
            if (depth > max_r_img_val){
                max_r_img_val=depth;
            }
        }
    }
    std::cout <<"max range is" << max_r_img_val << std::endl;

    //first invert the values, in this way we also create a buffer for wrong values
    float invert_buffer = 20;

    for (int i =0; i<rows; i++){
        for(int j = 0; j<cols;j++){
            float depth = dense_r_img.at<float>(i,j);
            
            if (depth > 0.1){ 
                //is a valid pixel -> invert its value
                //dense_r_img.at<float>(i,j) = max_r_img_val + invert_buffer - depth;
                //QUESTA È QUELLA BONA
                dense_r_img.at<float>(i,j) = max_depth - depth;
            }

        }
    }
    //std::cout << "0k" << std::endl;

    //diamond kernerl to dilate first known depth values
    int d[5][5] = { 0, 0, 1, 0, 0, 
                    0, 1, 1, 1, 0, 
                    1, 1, 1, 1, 1, 
                    0, 1, 1, 1, 0, 
                    0, 0, 1, 0, 0};

    cv::Mat diamond_kernel_5(5, 5, CV_8UC1, d);

   // std::cout<<sparse_r_img<<std::endl;
    cv::dilate(dense_r_img, dense_r_img, diamond_kernel_5);
   


    cv::Mat full_kernel_5 = cv::Mat::ones(5, 5, CV_8UC1);
    cv::morphologyEx(dense_r_img, dense_r_img, cv::MORPH_CLOSE, full_kernel_5);

   
    cv::Mat specific_range_img = dense_r_img.clone();
    cv::Mat full_kernel_7 = cv::Mat::ones(7, 7, CV_8UC1);
    cv::dilate(specific_range_img, specific_range_img, full_kernel_7);

    for (int i = 0; i<rows;i++){
        for(int j = 0; j<cols; j++){
            float depth = dense_r_img.at<float>(i,j);
            //if its an invalid pixel, first filling of empty spaces
            if (depth<0.1){
                dense_r_img.at<float>(i,j) = specific_range_img.at<float>(i,j);
            }
        }
    }
     //extend pixels to the top of the image   
    //extr = true;  
    int densify = true;
    if (densify){
    //if (true){
        
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
        //  Large Fill
        cv::Mat full_kernel_31 = cv::Mat::ones(31, 31, CV_8UC1);
            // std::cout << "comincio a extrapola nel while " << std::endl;
        specific_range_img = dense_r_img.clone();
        cv::dilate(specific_range_img, specific_range_img, full_kernel_31);
        int hole_pixel_count = 0;
        for(int i=0; i < rows; ++i){
            for(int j = 0; j < cols; ++j){
                float depth = dense_r_img.at<float>(i, j);
                //std::cout <<depth<< std::endl;
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

    //} MESSA SOPRA
    cv::medianBlur(dense_r_img, dense_r_img, 5);
    
    if(blur_type == std::string("bilateral")){
        // Bilateral blur
        cv::bilateralFilter(dense_r_img, dense_r_img, 5, 1.5, 2.0);
    }
    else if(blur_type == std::string("gaussian")){
        // Gaussian blur 
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
    }
    // Invert
    for(int i=0; i < rows; ++i){
        for(int j = 0; j < cols; ++j){
            float depth = dense_r_img.at<float>(i, j);
            if (depth > 0.1){
                //dense_r_img.at<float>(i, j) = max_r_img_val + invert_buffer- dense_r_img.at<float>(i, j);
                //COMMENT THIS LINE IN ORDER TO GET A DEPTH MAP WHICH IS RED AND THEN BLUE,
                //LATER TRY TO ADJUST THIS 
                dense_r_img.at<float>(i, j) = max_depth- dense_r_img.at<float>(i, j);
                //continue;
            }
        }
    }

    }