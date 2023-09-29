#include <opencv2/opencv.hpp>
#include "img_completion_lc.cpp"
#include "../DC_lidar_only/img_completion.cpp"
#include <stdio.h>
#include <math.h>
#include <vector>
#include <float.h>
#include <string>

using namespace std;

#include "slic.h"

void toColorImage(const cv::Mat &r_img, cv::Mat &color_img)
{
    cv::Mat normalize_dense_r_img;
    cv::normalize(r_img, normalize_dense_r_img, 1.0, 0, cv::NORM_MINMAX);
    // CV_8UC1 -> a 8-bit single-channel array
    cv::Mat dense_r_img_CV_8UC1;
    normalize_dense_r_img.convertTo(dense_r_img_CV_8UC1, CV_8UC1, 255.0);
    cv::applyColorMap(dense_r_img_CV_8UC1, color_img, cv::COLORMAP_JET);
}

int prova_(){
    std::string n_sp;
    
    std::string s_sp;

    cv::Mat mat_n_sp;
    cv::Mat mat_s_sp;
    


    n_sp = "../No_sp.png";
    s_sp = "../Si_sp.png";

    mat_n_sp = cv::imread(n_sp, cv::IMREAD_ANYDEPTH);
    mat_s_sp = cv::imread(s_sp, cv::IMREAD_ANYDEPTH);
    if (mat_n_sp.empty()) {
        printf("Failed to load image\n");
        return -1;
    }
    if (mat_s_sp.empty()) {
        printf("Failed to load image\n");
        return -1;
    }
    cv::Mat mat_n_sp_proj;
    mat_n_sp.convertTo(mat_n_sp_proj, CV_32F, 1.0 / 256.0);
    
    cv::Mat mat_s_sp_proj;
    mat_s_sp.convertTo(mat_s_sp_proj, CV_32F, 1.0 / 256.0);
    cv::imshow("no super pixel", mat_n_sp_proj);    
    cv::waitKey(0);
    
    cv::imshow("si super pixel", mat_s_sp_proj);    
    cv::waitKey(0);

    int num_equals = 0;

    int num_diff = 0;
    for (int i = 0; i < mat_n_sp.rows; i++){
        for(int j = 0; j < mat_n_sp.cols; j++){
            if (mat_n_sp_proj.at<float>(i, j) !=mat_s_sp_proj.at<float>(i, j) ){
                num_diff++;
            }
            else{
                num_equals++;
            }
            
            mat_s_sp_proj.at<float>(i, j) -=mat_n_sp_proj.at<float>(i, j); 

        }
    }
    std::cout << "num different pixels is: " << num_diff << std::endl;
    std::cout << "num equal pixels is: " << num_equals << std::endl;
    
    cv::imshow("difference", mat_s_sp_proj);    
    cv::waitKey(0);

    std::cout << "FINISHED" <<std::endl;
    return 1;

}

void evaluate_performance(const cv::Mat& GT_img, const cv::Mat& r_img, float& mse, float& mae ){
    int rows = GT_img.rows;
    int cols = GT_img.cols;
    int tolerance = 0.1;
    float sum_error_mse = 0;
    float sum_error_mae = 0;
    int count = 0;
    //ERROR METRICS

    //const float d_err = fabs(depth_gt_m - depth_ipol_m);
    //const float d_err_squared = d_err * d_err;
    //const float d_err_inv = fabs( 1.0 / depth_gt_m - 1.0 / depth_ipol_m);
    //const float d_err_inv_squared = d_err_inv * d_err_inv;
    //const float d_err_log = fabs(log(depth_gt_m) - log(depth_ipol_m));
    //const float d_err_log_squared = d_err_log * d_err_log;
    for (int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            float gt_value = GT_img.at<float>(i,j);
            float r_value = r_img.at<float>(i,j);
            if ( GT_img.at<float>(i,j) > tolerance && r_value > tolerance){
                
                float d_err = fabs(gt_value - r_value);
                //sum_error+= fabs(1.0 / gt_value - 1.0 / r_value);//*(gt_value - r_value);
                sum_error_mse+=d_err*d_err;
                sum_error_mae+=d_err;
                count++;
            }
        }
    }
    mse = sqrt(sum_error_mse/count);
    mae = sum_error_mae/count;
}



int main(int argc, char *argv[]) {
    /* Load the image and convert to Lab colour space. */

    std::string image_raw, image_raw_gt;
    
    //int r = prova_();
    
    
    std::cout << "STARTED (EXITED)" <<std::endl;

    cv::Mat image_sparse;
    cv::Mat image_r;
    cv::Mat image_r_gt;
    std::string image_path;
    /*// GOLDEN IMAGE
    std::string todo = "2011_10_03";
    std::string year = "0047";
    std::string sample = "0000000245";
    
    
    image_path= "../depth_selection/val_selection_cropped/image/"+ todo + "_drive_"+ year +"_sync_image_"+sample+"_image_02.png";
    image_raw = "../depth_selection/val_selection_cropped/velodyne_raw/2011_10_03_drive_0047_sync_velodyne_raw_0000000245_image_02.png";
    //image_raw = "../depth_selection/val_selection_cropped/velodyne_raw/"+ todo +" _drive_"+ year + "_sync_velodyne_raw_"+sample+"_image_02.png";
    image_raw_gt = "../depth_selection/val_selection_cropped/groundtruth_depth/"+ todo +"_drive_"+ year + "_sync_groundtruth_depth_"+sample+"_image_02.png";
    */
    //std::string todo = "2011_09_26";
    //std::string year = "0002";
    //std::string sample = "0000000005";
    std::string todo = "2011_09_28";
    std::string year = "0037";
    std::string sample = "0000000053";
    //int bi = prova_();
    
    image_path= "../depth_selection/val_selection_cropped/image/"+ todo + "_drive_"+ year +"_sync_image_"+sample+"_image_02.png";
    //image_raw = "../depth_selection/val_selection_cropped/velodyne_raw/2011_10_03_drive_0047_sync_velodyne_raw_0000000245_image_02.png";
    image_raw = "../depth_selection/val_selection_cropped/velodyne_raw/"+ todo +"_drive_"+ year + "_sync_velodyne_raw_"+sample+"_image_02.png";
    image_raw_gt = "../depth_selection/val_selection_cropped/groundtruth_depth/"+ todo +"_drive_"+ year + "_sync_groundtruth_depth_"+sample+"_image_02.png";
    

    image_r = cv::imread(image_raw, cv::IMREAD_ANYDEPTH);
    cv::Mat projected_depths;
    image_r.convertTo(projected_depths, CV_32F, 1.0 / 256.0);

    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
     if (image.empty()) {
        printf("Failed to load image\n");
        return -1;
    }
    if (image_r.empty()) {
        printf("Failed to load image raw (Lidar)\n");
        return -1;
    }

    image_r_gt = cv::imread(image_raw_gt, cv::IMREAD_ANYDEPTH);
    if (image_r_gt.empty()) {
        printf("Failed to load gt lidar\n");
        return -1;
    }

    cv::Mat projected_depths_gt;
    image_r_gt.convertTo(projected_depths_gt, CV_32F, 1.0 / 256.0);

    cv::Mat lab_image;
    cv::cvtColor(image, lab_image, cv::COLOR_BGR2Lab);
    

    /* Yield the number of superpixels and weight-factors from the user. */
    int w = image.cols, h = image.rows;
    int nr_superpixels = 1200;
    //int nc = 40;
    //int nr_superpixels = 20;
    ////nc is set to be a constant m -> no need to add complexity
    int nc = 50;

    time_t start, end;
    time(&start);
    // S
    double step = sqrt((w * h) / (double) nr_superpixels);
    
    /* Perform the SLIC superpixel algorithm. */
    Slic slic;
    slic.generate_superpixels(lab_image, step, nc);
    slic.create_connectivity(lab_image);
    //
    ///* Display the contours and show the result. */
    
    time(&end);

    cv::Mat dense_range_img;

    cv::Mat dense_range_img_sp;

    std::string blur_type = std::string("gaussian");

    slic.display_contours(image, CV_RGB(200,0,0));
    cv::imshow("img", image);
    cv::waitKey(0);

    //interpolate_with_superpixels(slic, projected_depths, dense_range_img, blur_type,0);
    img_completion(projected_depths,dense_range_img,0,blur_type);
    interpolate_with_superpixels(slic, projected_depths, dense_range_img_sp, blur_type,1);

    float mse, mse_p, mae, mae_p;
    
    evaluate_performance(projected_depths_gt, dense_range_img, mse, mae );
    evaluate_performance(projected_depths_gt, dense_range_img_sp, mse_p, mae_p );
    
    std::cout << "The value of the MSE without SP is: " << mse <<std::endl;
    std::cout << "The value of the MSE with SP is: " << mse_p <<std::endl;
    std::cout << "The value of the MAE without SP is: " << mae <<std::endl;
    std::cout << "The value of the MAE with SP is: " << mae_p <<std::endl;


    //cv::imwrite("../No_sp.png", dense_range_img);
    //cv::imwrite("../Si_sp.png", dense_range_img_sp);


    double time_taken = double(end - start);
    std::cout << "Time taken by program is : " << time_taken << setprecision(5) << std::endl;

  
    

    cv::Mat color_dense_range_img_sp;
    cv::Mat color_dense_range_img;

    toColorImage(dense_range_img_sp, color_dense_range_img_sp);
    toColorImage(dense_range_img, color_dense_range_img);

    //slic.colour_with_cluster_means(image);
    cv::imshow("NO SP", color_dense_range_img);    
    cv::waitKey(0);
    

    cv::imshow("SI SP", color_dense_range_img_sp);    
    cv::waitKey(0);

    

        /////
     for (int i = 0; i < dense_range_img.rows; i++){
        for(int j = 0; j < dense_range_img.cols; j++){
          
            dense_range_img_sp.at<float>(i, j) -=dense_range_img.at<float>(i, j); 

        }
    }
    
    
    cv::imshow("difference", dense_range_img_sp);    
    cv::waitKey(0);

    /////
    //cvSaveImage(argv[4], image);
    int bo = prova_();
}