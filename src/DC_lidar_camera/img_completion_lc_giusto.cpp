// Here we perform RGB-guided depth completion. The aim is to use also the informations that are available 
// trought a super pixel identification. The filling procedure will be done based on depth value
// and also based on the superpixel information 
#include <iostream>
#include <vector>
#include "slic.h" 

//void check_valid_invalid(cv::Mat &dense_r_img){
//    int rows = dense_r_img.rows;
//    int cols = dense_r_img.cols;
//    int number_of_pixels = rows*cols;
//    std::cout << "number of pixels is: " << number_of_pixels << std::endl;
//    int number_of_valid_pixels=0;
//    int number_of_invalid_pixels=0;
//    for(int i = 0; i<rows; i++){
//        for(int j = 0; j<cols; j++){
//            float depth = dense_r_img.at<float>(i,j);
//            if(depth > 0.1){
//                number_of_valid_pixels++;
//            }
//            else{
//                number_of_invalid_pixels++;
//            }
//        }
//    }
//    std::cout << "number of valid pixels is: " << number_of_valid_pixels << std::endl;
//
//    std::cout << "number of invalid pixels is: " << number_of_invalid_pixels << std::endl;
//
//    std::cout << "TOTAL: " << number_of_pixels - number_of_valid_pixels -number_of_invalid_pixels << std::endl;
//
//}
//
//void interpolate_with_superpixels(Slic& slic,
//                                 const cv::Mat &sparse_r_img,
//                                 cv::Mat &dense_r_img, 
//                                 const std::string &blur_type,
//                                 int use_superpixel){
//
//    // THIS WORKS... MAYBE STATISTICIAN  
//    //for(int j = 0; j < (int) slic.centers.size(); j++){
//    //    std::cout << "FROM CAHE x: " << (int) slic.centers[j][3] << " y: " << (int) slic.centers[j][4] << std::endl;
//    //}
//    float max_r_img_val = 0.0;
//    float max_depth = 100.0;
//    int rows = sparse_r_img.rows;
//    int cols = sparse_r_img.cols;
//    
//    dense_r_img = sparse_r_img.clone();
//
//    check_valid_invalid(dense_r_img);
//    
//
////    
////    // Get max range
//    for (int i = 0; i<rows; i++){
//        for (int j = 0; j<cols; j++){
//            float depth = dense_r_img.at<float>(i,j);
//           // std::cout<<depth<<std::endl;
//            if (depth > max_r_img_val){
//                max_r_img_val=depth;
//            }
//        }
//    }
//    float invert_buffer = 20.0;
//    // Invert depth
//    for (int i =0; i<rows; i++){
//        for(int j = 0; j<cols;j++){
//            float depth = dense_r_img.at<float>(i,j);
//            
//            if (depth > 0.1){ 
//                //is a valid pixel -> invert its value
//                //dense_r_img.at<float>(i,j) = max_r_img_val + invert_buffer - depth;
//                dense_r_img.at<float>(i,j) = max_depth - depth;
//            }
//
//        }
//    }
////
//    int d[5][5] = { 0, 0, 1, 0, 0, 
//                    0, 1, 1, 1, 0, 
//                    1, 1, 1, 1, 1, 
//                    0, 1, 1, 1, 0, 
//                    0, 0, 1, 0, 0};
//    cv::Mat diamond_kernel_5(5, 5, CV_8UC1, d);
//    cv::dilate(dense_r_img, dense_r_img, diamond_kernel_5);
//    //check_valid_invalid(dense_r_img);
//    //IF COMMENTED ALSO LAST INVERSION MUST BE COMMENTED AND SAME FOR UNCOMMENTED
////
////
//    //for(int i=0; i < rows; ++i){
//    //    for(int j = 0; j < cols; ++j){
//    //        float depth = dense_r_img.at<float>(i, j);
//    //        if (depth > 0.1 ){//&& (slic.clusters[i][j] == c || slic.clusters[i][j] == -1)){
//    //            //dense_r_img.at<float>(i, j) = max_r_img_val + invert_buffer- dense_r_img.at<float>(i, j);
//    //            dense_r_img.at<float>(i, j) = max_depth- dense_r_img.at<float>(i, j);
//    //        }
//    //    }
//    //}
//    //check_valid_invalid(dense_r_img);
////    
//////https://xplorestaging.ieee.org/document/7351624
//////Superpixel-based depth map inpainting for RGB-D view synthesis
//    cv::Mat full_kernel_5 = cv::Mat::ones(5, 5, CV_8UC1);
//    cv::morphologyEx(dense_r_img, dense_r_img, cv::MORPH_CLOSE, full_kernel_5);
//    check_valid_invalid(dense_r_img);
//    cv::Mat specific_range_img = dense_r_img.clone();
//    
//    cv::Mat full_kernel_7 = cv::Mat::ones(7, 7, CV_8UC1);
//    cv::dilate(specific_range_img, specific_range_img, full_kernel_7);
//
//    if(use_superpixel==1){ 
////
//    for(int c = 0; c < (int) slic.centers.size(); c++){
//   //         cv::Mat full_kernel_7 = cv::Mat::ones(7, 7, CV_8UC1);
//   // cv::dilate(specific_range_img, specific_range_img, full_kernel_7);
//        for (int i = 0; i<rows;i++){
//            for(int j = 0; j<cols; j++){
//                if (slic.clusters[i][j] == c /*|| slic.clusters[i][j] == -1*/){ 
//                    float depth = dense_r_img.at<float>(i,j);
//                  //  std::cout<< "la matregna" << std::endl;
//                //if its an invalid pixel, first filling of empty spaces
//                    if (depth<0.1){
//                        dense_r_img.at<float>(i,j) = specific_range_img.at<float>(i,j);
//                    }
//                }
//            }
//        }
//    }
//    }
//    else{ 
//
//        for (int i = 0; i<rows;i++){
//            for(int j = 0; j<cols; j++){
//                    float depth = dense_r_img.at<float>(i,j);
//                //if its an invalid pixel, first filling of empty spaces
//                    if (depth<0.1){
//                        dense_r_img.at<float>(i,j) = specific_range_img.at<float>(i,j);
//                    }
//
//            }
//        }
//
//    }
//    check_valid_invalid(dense_r_img);
//
//
//
//
//    //for (int val = 0; val < (int) slic.centers.size(); val++){ 
//    //    std::cout << "avg_depth_p_sp " << avg_depth_p_sp[val]  <<std::endl;
//    //}
//
//
////
// 
//    cv::Mat full_kernel_31 = cv::Mat::ones(31, 31, CV_8UC1);
//    specific_range_img = dense_r_img.clone();
//    cv::dilate(specific_range_img, specific_range_img, full_kernel_31);
//    
//    
//    
//    //for(int c = 0; c < (int) slic.centers.size(); c++){
//   
//        for(int i=0; i < rows; ++i){
//            for(int j = 0; j < cols; ++j){
//                float depth = dense_r_img.at<float>(i, j);
//    //             if (slic.clusters[i][j] == c || slic.clusters[i][j] == -1){ 
//                //std::cout <<dpth<< std::endl;
//                    if (depth < 0.1){
//                        dense_r_img.at<float>(i, j) = specific_range_img.at<float>(i, j); 
//                    }
//     //            }
//            }
//        }
//    //}
//        
//
//    check_valid_invalid(dense_r_img);
//
//
////I COULD GET THE AVERAGE VALUE FOR EACH SUPERPIXEL SET AND THEN FILL THE REMAINING VALUES WITH THAT
////WHITHIN A SUPERPIXEL
//    vector<float> avg_depth_p_sp;
//
//    for (int c = 0; c < (int) slic.centers.size(); c++){
//        float depth_ = 0;
//        int num = 0;
//        float value = 0;
//        for (int i = 0; i < rows; i++){
//            for (int j = 0; j < cols; j++){
//                if (slic.clusters[i][j]==c){
//                    depth_+= dense_r_img.at<float>(i,j);
//                    num++;
//                }
//            }
//        }
//        //std::cout << "num is " << num << std::endl;
//        //std::cout << "depth is " << depth_ << std::endl;
//        if (num!=0){
//            value = depth_/num;
//            std::cout << "depth is avg " << value << std::endl;
//        }
//       
//        avg_depth_p_sp.push_back(value);
//    }
//
//
//////
//
//for (int c = 0; c < (int) slic.centers.size(); c++){
//    for(int i=0; i < rows; ++i){
//            for(int j = 0; j < cols; ++j){
//                float depth = dense_r_img.at<float>(i, j);
//                //std::cout <<depth<< std::endl;
//                if( (slic.clusters[i][j]==c) && depth < 0.1 && avg_depth_p_sp[c] > 20)
//                    continue;
//                    //dense_r_img.at<float>(i, j) =max_r_img_val- avg_depth_p_sp[c]; 
//                
//            }
//        }
//    }
////    check_valid_invalid(dense_r_img);
//
//        specific_range_img = dense_r_img.clone();
//        cv::GaussianBlur(specific_range_img, specific_range_img, cv::Size(5, 5), 0);
////
//        for(int i=0; i < rows; ++i){
//            for(int j = 0; j < cols; ++j){
//                float depth = dense_r_img.at<float>(i, j);
//                if (depth > 0.1){
//                    dense_r_img.at<float>(i, j) = specific_range_img.at<float>(i, j);
//                }
//            }
//        }
//
//    for(int i=0; i < rows; ++i){
//        for(int j = 0; j < cols; ++j){
//            float depth = dense_r_img.at<float>(i, j);
//            if (depth > 0.1){
//                //uncomment this one for proper depth
//                //dense_r_img.at<float>(i, j) = max_r_img_val + invert_buffer- dense_r_img.at<float>(i, j);
//                dense_r_img.at<float>(i, j) = max_depth- dense_r_img.at<float>(i, j);
//                //dense_r_img.at<float>(i, j) =0;
//                continue;
//            }
//        }
//    }
//
//}

////////////////////////////////////////////////////////////

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

    // THIS WORKS... MAYBE STATISTICIAN  
    //for(int j = 0; j < (int) slic.centers.size(); j++){
    //    std::cout << "FROM CAHE x: " << (int) slic.centers[j][3] << " y: " << (int) slic.centers[j][4] << std::endl;
    //}
    float max_r_img_val = 0.0;
    float max_depth = 100.0;
    int rows = sparse_r_img.rows;
    int cols = sparse_r_img.cols;
    
    dense_r_img = sparse_r_img.clone();

    for (int i = 0; i<rows; i++){
        for (int j = 0; j<cols; j++){
            float depth = dense_r_img.at<float>(i,j);
           // std::cout<<depth<<std::endl;
            if (depth > max_r_img_val){
                max_r_img_val=depth;
            }
        }
    }
    float invert_buffer = 0.0;
    for (int i =0; i<rows; i++){
        for(int j = 0; j<cols;j++){
            float depth = dense_r_img.at<float>(i,j);
            
            if (depth > 0.1){ 

                dense_r_img.at<float>(i,j) = max_depth - depth;
            }

        }
    }

    int d[5][5] = { 0, 0, 1, 0, 0, 
                    0, 1, 1, 1, 0, 
                    1, 1, 1, 1, 1, 
                    0, 1, 1, 1, 0, 
                    0, 0, 1, 0, 0};
    cv::Mat diamond_kernel_5(5, 5, CV_8UC1, d);
    cv::dilate(dense_r_img, dense_r_img, diamond_kernel_5);

    for(int i=0; i < rows; ++i){
        for(int j = 0; j < cols; ++j){
            float depth = dense_r_img.at<float>(i, j);
            if (depth > 0.1 ){//&& (slic.clusters[i][j] == c || slic.clusters[i][j] == -1)){
                //dense_r_img.at<float>(i, j) = max_r_img_val + invert_buffer- dense_r_img.at<float>(i, j);
                dense_r_img.at<float>(i, j) = max_depth- dense_r_img.at<float>(i, j);
            }
        }
    }

//    
////https://xplorestaging.ieee.org/document/7351624
////Superpixel-based depth map inpainting for RGB-D view synthesis
    cv::Mat full_kernel_5 = cv::Mat::ones(5, 5, CV_8UC1);
    cv::morphologyEx(dense_r_img, dense_r_img, cv::MORPH_CLOSE, full_kernel_5);
    
    cv::Mat specific_range_img = dense_r_img.clone();
    
    cv::Mat full_kernel_7 = cv::Mat::ones(7, 7, CV_8UC1);
    cv::dilate(specific_range_img, specific_range_img, full_kernel_7);

    if(use_superpixel==1){ 
//
    for(int c = 0; c < (int) slic.centers.size(); c++){
   // cv::Mat full_kernel_7 = cv::Mat::ones(7, 7, CV_8UC1);
   // cv::dilate(specific_range_img, specific_range_img, full_kernel_7);
        for (int i = 0; i<rows;i++){
            for(int j = 0; j<cols; j++){
                if (slic.clusters[i][j] == c /*|| slic.clusters[i][j] == -1*/){ 
                    float depth = dense_r_img.at<float>(i,j);
                  //  std::cout<< "la matregna" << std::endl;
                //if its an invalid pixel, first filling of empty spaces
                    if (depth<0.1){
                        dense_r_img.at<float>(i,j) = specific_range_img.at<float>(i,j);
                    }
                }
            }
        }
    }
    }
    else{ 

        for (int i = 0; i<rows;i++){
            for(int j = 0; j<cols; j++){
                    float depth = dense_r_img.at<float>(i,j);
                //if its an invalid pixel, first filling of empty spaces
                    if (depth<0.1){
                        dense_r_img.at<float>(i,j) = specific_range_img.at<float>(i,j);
                    }

            }
        }

    }
    //check_valid_invalid(dense_r_img);




    //for (int val = 0; val < (int) slic.centers.size(); val++){ 
    //    std::cout << "avg_depth_p_sp " << avg_depth_p_sp[val]  <<std::endl;
    //}


//
 
    cv::Mat full_kernel_31 = cv::Mat::ones(31, 31, CV_8UC1);
    specific_range_img = dense_r_img.clone();
    cv::dilate(specific_range_img, specific_range_img, full_kernel_31);
    
    
    
    //for(int c = 0; c < (int) slic.centers.size(); c++){
   
        for(int i=0; i < rows; ++i){
            for(int j = 0; j < cols; ++j){
                float depth = dense_r_img.at<float>(i, j);
    //             if (slic.clusters[i][j] == c || slic.clusters[i][j] == -1){ 
                //std::cout <<dpth<< std::endl;
                    if (depth < 0.1){
                        dense_r_img.at<float>(i, j) = specific_range_img.at<float>(i, j); 
                    }
     //            }
            }
        }
    //}
        

    //check_valid_invalid(dense_r_img);


//I COULD GET THE AVERAGE VALUE FOR EACH SUPERPIXEL SET AND THEN FILL THE REMAINING VALUES WITH THAT
//WHITHIN A SUPERPIXEL
    vector<float> avg_depth_p_sp;

    for (int c = 0; c < (int) slic.centers.size(); c++){
        float depth_ = 0;
        int num = 0;
        float value = 0;
        for (int i = 0; i < rows; i++){
            for (int j = 0; j < cols; j++){
                if (slic.clusters[i][j]==c){
                    depth_+= dense_r_img.at<float>(i,j);
                    num++;
                }
            }
        }
        //std::cout << "num is " << num << std::endl;
        //std::cout << "depth is " << depth_ << std::endl;
        if (num!=0){
            value = depth_/num;
            //std::cout << "depth is avg " << value << std::endl;
        }
       
        
            
        
        avg_depth_p_sp.push_back(value);
    }


////

for (int c = 0; c < (int) slic.centers.size(); c++){
    for(int i=0; i < rows; ++i){
            for(int j = 0; j < cols; ++j){
                float depth = dense_r_img.at<float>(i, j);
                //std::cout <<depth<< std::endl;
                if( (slic.clusters[i][j]==c) && depth < 0.1 && avg_depth_p_sp[c] > 20)
                    continue;
                    //dense_r_img.at<float>(i, j) =max_r_img_val- avg_depth_p_sp[c]; 
                
            }
        }
    }
//    check_valid_invalid(dense_r_img);

        specific_range_img = dense_r_img.clone();
        cv::GaussianBlur(specific_range_img, specific_range_img, cv::Size(5, 5), 0);
//
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
                //dense_r_img.at<float>(i, j) = max_r_img_val + invert_buffer- dense_r_img.at<float>(i, j);
                //dense_r_img.at<float>(i, j) = max_depth- dense_r_img.at<float>(i, j);
                continue;
            }
        }
    }

}
