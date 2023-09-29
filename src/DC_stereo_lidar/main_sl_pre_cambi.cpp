#include <opencv2/opencv.hpp>
#include <iostream>
#include <random>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Dense> 

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include "../DC_lidar_only/img_completion.cpp"
#include "../DC_lidar_camera/img_completion_lc.cpp"
#include "../DC_lidar_camera/slic.cpp"
using namespace pcl;
// IMPORTANT NOTE ABOUT SENSORS SETUP
/*
There are four cameras (from left to right) Cam2(RGB) Cam0(GRAY) Cam3(RGB) Cam1(GRAY).
And one lidar (Velodyne). The reference frame is Cam0 (origin). All other quantities are expressed
as transform with respect to the orgin frame (Cam0). Moreover in the calibration file are provided
both the values of the intrinsic and extrinsic BEFORE rectification and AFTER rectification.
In order to perform stereo matching the lidar points must be projected into the RECTIFIED image plane 
of the two cameras. Namely Cam2 and Cam3 which are the colored one.
To projects a velodyne coordinate point into the camera_2 image
                    
                    T_velo2cam2 = P2_rect * R2_rect * T_C02C2 * T_V2C0
                    y_image_2 = T_velo2cam2 * x_velo_coord
*/

void toColorImage(const cv::Mat &r_img, cv::Mat &color_img)
{
    cv::Mat normalize_dense_r_img;
    cv::normalize(r_img, normalize_dense_r_img, 1.0, 0, cv::NORM_MINMAX);
    // CV_8UC1 -> a 8-bit single-channel array
    cv::Mat dense_r_img_CV_8UC1;
    normalize_dense_r_img.convertTo(dense_r_img_CV_8UC1, CV_8UC1, 255.0);
    cv::applyColorMap(dense_r_img_CV_8UC1, color_img, cv::COLORMAP_JET);
}

//FARE FUNZIONE READ 3X3 MATRIX CHE COSI NON SE PO VEDE
int read_kitti_ds(std::string left_img_path,
                   std::string right_img_path,
                   cv::Mat& img_left,
                   cv::Mat& img_right,
                   std::string calibration_v2c_path,
                   Eigen::Matrix4f& T_V_2_C0,
                   std::string calibration_ci2cj_path,
                   Eigen::Matrix4f& T_C0_2_C2,
                   Eigen::Matrix4f& T_C0_2_C3,
                   Eigen::Matrix4f& R_rect_00,
                   Eigen::Matrix4f& R_rect_02,
                   Eigen::Matrix4f& R_rect_03,
                   Eigen::Matrix<float, 3, 4>& P_rect_02,
                   Eigen::Matrix<float, 3, 4>&  P_rect_03){
    //cv::Mat img_left = cv::imread(left_img);//, cv::COLOR_BGR2GRAY);
    //cv::Mat img_right = cv::imread(right_img);//, cv::COLOR_BGR2GRAY);
    //read images
    img_left = cv::imread(left_img_path);
    img_right = cv::imread(right_img_path);
    if (img_left.empty()) {
        printf("Failed to load left image\n");
        return -1;
    }
    if (img_right.empty()) {
        printf("Failed to load right image\n");
        return -1;
    }
    //read lidar datas... TODO

    //read calibration file lidar to camera 0 
    std::ifstream file(calibration_v2c_path);

    if (!file.is_open()) {
         printf("Failed to load calibration file\n");
        return -1;
    }

    std::string line;
    Eigen::Matrix3f R_v2c;
    Eigen::Vector3f t_v2c;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        //std::cout << line <<std::endl;
        std::string identifier;
        iss >> identifier;
        //std::cout << line <<std::endl;
        std::cout << "identifier is: " << identifier <<std::endl;
        if (identifier == "R:") {
            // Create an Eigen matrix and populate it
            int rows = 3;int cols = 3;
           
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    float value;
                    iss >> value;
                    R_v2c(i,j) = value;
                }
            }
        }
        if (identifier == "T:"){
           int rows = 3;
           for (int i = 0; i < rows; ++i){
               iss >> t_v2c(i);
           }
        }

    }

    // Transform velodyne to camere in homogeneous coordinates
    T_V_2_C0 << R_v2c(0,0), R_v2c(0,1), R_v2c(0,2), t_v2c(0),
                R_v2c(1,0), R_v2c(1,1), R_v2c(1,2), t_v2c(1),
                R_v2c(2,0), R_v2c(2,1), R_v2c(2,2), t_v2c(2),
                0,         0,          0,             1;               

    // Now we need to read the values of the cameras which are:
    //R_02 and t_02 -> T_C02C2
    //R_rect_02 and P_rect_02
    //Same for 3 


    std::ifstream file_c(calibration_ci2cj_path);

    if (!file_c.is_open()) {
         printf("Failed to load calibration file_c\n");
        return -1;
    }

    std::string line_c;
    Eigen::Matrix3f R_C0_2_C2;
    Eigen::Vector3f t_C0_2_C2;

    Eigen::Matrix3f R_C0_2_C3;
    Eigen::Vector3f t_C0_2_C3;

    while (std::getline(file_c, line_c)) {
        std::istringstream iss(line_c);
        //std::cout << line_c <<std::endl;
        std::string identifier;
        iss >> identifier;
        //std::cout << line_c <<std::endl;
        //std::cout << "identifier is: " << identifier <<std::endl;
        if (identifier == "R_02:") {
            // Create an Eigen matrix and populate it
            int rows = 3;int cols = 3;
           
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    float value;
                    iss >> value;
                    R_C0_2_C2(i,j) = value;
                }
            }

        }
        if (identifier == "T_02:"){
           int rows = 3;
           for (int i = 0; i < rows; ++i){
               iss >> t_C0_2_C2(i);
           }
        }
        if (identifier == "R_03:") {
            // Create an Eigen matrix and populate it
            int rows = 3;int cols = 3;
           
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    float value;
                    iss >> value;
                    R_C0_2_C3(i,j) = value;
                }
            }
        }
        if (identifier == "T_03:"){
           int rows = 3;
           for (int i = 0; i < rows; ++i){
               iss >> t_C0_2_C3(i);
           }
        }
        //0
        if (identifier == "R_rect_00:") {
            //is a (3X3)
            // Create an Eigen matrix and populate it
            int rows = 3;int cols = 3;
           
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    float value;
                    iss >> value;
                    R_rect_00(i,j) = value;
                }
            }
            R_rect_00(0,3)=0;R_rect_00(1,3)=0;R_rect_00(2,3)=0;R_rect_00(3,3)=1;
            R_rect_00(3,0)=0;R_rect_00(3,1)=0;R_rect_00(3,2)=0;
        }
        // 1
        if (identifier == "R_rect_02:") {
            //is a (3X3)
            // Create an Eigen matrix and populate it
            int rows = 3;int cols = 3;
           
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    float value;
                    iss >> value;
                    R_rect_02(i,j) = value;
                }
            }
            R_rect_02(0,3)=0;R_rect_02(1,3)=0;R_rect_02(2,3)=0;R_rect_02(3,3)=1;
            R_rect_02(3,0)=0;R_rect_02(3,1)=0;R_rect_02(3,2)=0;
        }
        // 2
        if (identifier == "P_rect_02:") {
            // Create an Eigen matrix and populate it
            //is a (3X4)
            int rows = 3;int cols = 4;
           
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    float value;
                    iss >> value;
                    P_rect_02(i,j) = value;
                }
            }
        }
        // 3
        if (identifier == "R_rect_03:") {
            // Create an Eigen matrix and populate it
            //is a (3X3)
            int rows = 3;int cols = 3;
           
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    float value;
                    iss >> value;
                    R_rect_03(i,j) = value;
                }
            }
            R_rect_03(0,3)=0;R_rect_03(1,3)=0;R_rect_03(2,3)=0;R_rect_03(3,3)=1;
            R_rect_03(3,0)=0;R_rect_03(3,1)=0;R_rect_03(3,2)=0;
        }
        // 4
        if (identifier == "P_rect_03:") {
            // Create an Eigen matrix and populate it
            //is a (3X4)
            int rows = 3;int cols = 4;
           
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    float value;
                    iss >> value;
                    P_rect_03(i,j) = value;
                }
            }
        }


    }

    T_C0_2_C2 << R_C0_2_C2(0,0), R_C0_2_C2(0,1), R_C0_2_C2(0,2), t_C0_2_C2(0),
                 R_C0_2_C2(1,0), R_C0_2_C2(1,1), R_C0_2_C2(1,2), t_C0_2_C2(1),
                 R_C0_2_C2(2,0), R_C0_2_C2(2,1), R_C0_2_C2(2,2), t_C0_2_C2(2),
                 0,         0,          0,             1;     
    T_C0_2_C3 << R_C0_2_C3(0,0), R_C0_2_C3(0,1), R_C0_2_C3(0,2), t_C0_2_C3(0),
                 R_C0_2_C3(1,0), R_C0_2_C3(1,1), R_C0_2_C3(1,2), t_C0_2_C3(1),
                 R_C0_2_C3(2,0), R_C0_2_C3(2,1), R_C0_2_C3(2,2), t_C0_2_C3(2),
                 0,         0,          0,             1;   

    return 1;


}



/*
P_rect_00: 
7.215377e+02 0.000000e+00 6.095593e+02 0.000000e+00 
0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 
0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00
0               0               0            1
*/



int vedi_pc(std::string inputFilePath,Eigen::Matrix4f& T, Eigen::Matrix<float, 3, 4>& P,
            cv::Mat& image,cv::Mat & projected_depths,cv::Mat& dense_range_img){
    
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Load lidar point cloud data from binary file
    std::ifstream input(inputFilePath, std::ios::binary);
    if (!input)
    {
        std::cerr << "Error opening input file" << std::endl;
        return -1;
    }
    //cv::Mat image(375, 1280, CV_8UC3, cv::Scalar(0, 0, 0));

    input.seekg(0, std::ios::end);
    long size = input.tellg();
    input.seekg(0, std::ios::beg);

    int numPoints = size / sizeof(float) / 4; // Each point has 4 float values (x, y, z, intensity)

    cloud->resize(numPoints);
    input.read(reinterpret_cast<char*>(&cloud->points[0]), size);

    input.close();

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);

    //pcl::transformPointCloud(*cloud, *transformedCloud, T);

    for (size_t i = 0; i < cloud->size(); ++i){
        const pcl::PointXYZ &originalPoint= cloud->at(i);
         pcl::PointXYZ transformedPoint;
     
        transformedPoint.x = T(0, 0) * originalPoint.x + T(0, 1) * originalPoint.y + T(0, 2) * originalPoint.z + T(0, 3);
        transformedPoint.y = T(1, 0) * originalPoint.x + T(1, 1) * originalPoint.y + T(1, 2) * originalPoint.z + T(1, 3);
        transformedPoint.z = T(2, 0) * originalPoint.x + T(2, 1) * originalPoint.y + T(2, 2) * originalPoint.z + T(2, 3);
        if (transformedPoint.z > 0   ){
            
            transformedCloud->push_back(transformedPoint);
        }
    }
    int projected = 0;

   

    for (size_t i = 0; i < transformedCloud->size(); ++i){

        const pcl::PointXYZ &point = transformedCloud->at(i);

        // Project the point using projection matrix P
        Eigen::Vector3f p(point.x, point.y, point.z);
        Eigen::Vector3f projectedPoint = P * p.homogeneous();
        projectedPoint(0)/=projectedPoint(2);
        projectedPoint(1)/=projectedPoint(2);
       // std::cout << " projected point is " << projectedPoint << std::endl;
        // Check if the projected point is within the image bounds
        if (projectedPoint(0) >= 0 && projectedPoint(0) < image.cols &&
            projectedPoint(1) >= 0 && projectedPoint(1) < image.rows)
        {
            projected++;
            // Convert the projected point to image coordinates and draw it on the image
            int u = static_cast<int>(projectedPoint(0));
            int v = static_cast<int>(projectedPoint(1));
            float depth = projectedPoint(2);
            //cv::Scalar color = cv::Scalar(0, depth * 255, 255 - depth * 255);
            //cv::circle(image, cv::Point(u, v), 2, color, -1);
            cv::circle(image, cv::Point(u, v), 2, cv::Scalar(255, 255, 255), -1);
            //cv::circle(image, cv::Point(u, v), 2, cv::Scalar(projectedPoint(2),0, 0), -1);
            projected_depths.at<float>(v, u) = projectedPoint(2);
          
        }
    }
 
    cv::Mat normalized_depths;
    cv::normalize(projected_depths, normalized_depths, 0, 100, cv::NORM_MINMAX);

///prova

    
    cv::Mat range_img;

    std::string blur_type = std::string("gaussian");
    bool extr = false;
    
    // THIS IS IN ORDER TO SHOW THE PROJECTED POINTS INTO THE IMAGE

    //cv::imshow("src", image);
    //
    //cv::waitKey(0);
    
    img_completion(normalized_depths, dense_range_img, extr, blur_type);
    cv::Mat color_dense_range_img;

    toColorImage(dense_range_img, color_dense_range_img);
    std::vector<cv::Mat> hImg;

     std::cout << "have been projected " << projected << std::endl;
    
    //I have to renstitute dense_range_image

    //HERE ARE THE IMAGES THAT ARE OBTAINET, UNCOMMENT THIS TO SHOW THEM

    //cv::imshow("Projected Point Cloud", normalized_depths);
    //cv::waitKey(0);
    //
    cv::imshow("src", color_dense_range_img);
    cv::waitKey(0);
    
//
    //cv::imshow("Projected Point Cloud", depthImage);
    //cv::waitKey(0);

    //FOR CICLE TO CHECK THE VALUES OF THE RESULTING IMAGE (DEPTH)

    //for (int i = 0; i < color_dense_range_img.rows; ++i){
    //    for (int j = 0; j < color_dense_range_img.cols; ++j){
    //        if(dense_range_img.at<float>(i,j) > 20){
    //            std::cout << dense_range_img.at<float>(i,j)<<std::endl;
    //        }
    //    }
    //}
   



    // Visualize the point cloud -> VERY NICE SKILL

    //pcl::visualization::CloudViewer viewer("Lidar Point Cloud Viewer (transformed)");
    //viewer.showCloud(transformedCloud);
//
    //while (!viewer.wasStopped())
    //{
    //}

    return 0;
}


/////////////////////////////// *SUPERPIXEL////////////////////////////
int withSuperPixels(std::string inputFilePath,Eigen::Matrix4f& T, Eigen::Matrix<float, 3, 4>& P,
            cv::Mat& image,cv::Mat & projected_depths,cv::Mat& dense_range_img){
    
    cv::Mat lab_image;
    cv::cvtColor(image, lab_image, cv::COLOR_BGR2Lab);
    Slic slic;

    int w = image.cols, h = image.rows;
    int nr_superpixels = 100;
    int nc = 40;

    
    double step = sqrt((w * h) / (double) nr_superpixels);
    
    /* Perform the SLIC superpixel algorithm. */
    slic.generate_superpixels(lab_image, step, nc);

     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Load lidar point cloud data from binary file
    std::ifstream input(inputFilePath, std::ios::binary);
    if (!input)
    {
        std::cerr << "Error opening input file" << std::endl;
        return -1;
    }
    //cv::Mat image(375, 1280, CV_8UC3, cv::Scalar(0, 0, 0));

    input.seekg(0, std::ios::end);
    long size = input.tellg();
    input.seekg(0, std::ios::beg);

    int numPoints = size / sizeof(float) / 4; // Each point has 4 float values (x, y, z, intensity)

    cloud->resize(numPoints);
    input.read(reinterpret_cast<char*>(&cloud->points[0]), size);

    input.close();

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);

    //pcl::transformPointCloud(*cloud, *transformedCloud, T);

    for (size_t i = 0; i < cloud->size(); ++i){
        const pcl::PointXYZ &originalPoint= cloud->at(i);
         pcl::PointXYZ transformedPoint;
     
        transformedPoint.x = T(0, 0) * originalPoint.x + T(0, 1) * originalPoint.y + T(0, 2) * originalPoint.z + T(0, 3);
        transformedPoint.y = T(1, 0) * originalPoint.x + T(1, 1) * originalPoint.y + T(1, 2) * originalPoint.z + T(1, 3);
        transformedPoint.z = T(2, 0) * originalPoint.x + T(2, 1) * originalPoint.y + T(2, 2) * originalPoint.z + T(2, 3);
        if (transformedPoint.z > 0   ){
            
            transformedCloud->push_back(transformedPoint);
        }
    }
    int projected = 0;

   

    for (size_t i = 0; i < transformedCloud->size(); ++i){

        const pcl::PointXYZ &point = transformedCloud->at(i);

        // Project the point using projection matrix P
        Eigen::Vector3f p(point.x, point.y, point.z);
        Eigen::Vector3f projectedPoint = P * p.homogeneous();
        projectedPoint(0)/=projectedPoint(2);
        projectedPoint(1)/=projectedPoint(2);
       // std::cout << " projected point is " << projectedPoint << std::endl;
        // Check if the projected point is within the image bounds
        if (projectedPoint(0) >= 0 && projectedPoint(0) < image.cols &&
            projectedPoint(1) >= 0 && projectedPoint(1) < image.rows)
        {
            projected++;
            // Convert the projected point to image coordinates and draw it on the image
            int u = static_cast<int>(projectedPoint(0));
            int v = static_cast<int>(projectedPoint(1));
            float depth = projectedPoint(2);
            //cv::Scalar color = cv::Scalar(0, depth * 255, 255 - depth * 255);
            //cv::circle(image, cv::Point(u, v), 2, color, -1);
            cv::circle(image, cv::Point(u, v), 2, cv::Scalar(255, 255, 255), -1);
            //cv::circle(image, cv::Point(u, v), 2, cv::Scalar(projectedPoint(2),0, 0), -1);
            projected_depths.at<float>(v, u) = projectedPoint(2);
          
        }
    }
 
    cv::Mat normalized_depths;
    cv::normalize(projected_depths, normalized_depths, 0, 80, cv::NORM_MINMAX);

///prova

    
    cv::Mat range_img;

    std::string blur_type = std::string("gaussian");
    bool extr = false;
    
    // THIS IS IN ORDER TO SHOW THE PROJECTED POINTS INTO THE IMAGE

    //cv::imshow("src", image);
    //
    //cv::waitKey(0);
    

    interpolate_with_superpixels(slic, normalized_depths, dense_range_img, blur_type,1);

    //img_completion(normalized_depths, dense_range_img, extr, blur_type);
    cv::Mat color_dense_range_img;

    toColorImage(dense_range_img, color_dense_range_img);
    std::vector<cv::Mat> hImg;

     std::cout << "have been projected " << projected << std::endl;
    
    //I have to renstitute dense_range_image

    //HERE ARE THE IMAGES THAT ARE OBTAINET, UNCOMMENT THIS TO SHOW THEM

    //cv::imshow("Projected Point Cloud", normalized_depths);
    //cv::waitKey(0);
    //
    cv::imshow("src", color_dense_range_img);
    cv::waitKey(0);
    


    return 0;
}
///////////////////////////////




//this is just a function that takes a point in an image and shows the corresponding one
//in the other image (using the initial guess on the depth)
void check_reproj_epip(cv::Mat& initial_guess,cv::Mat& image_left,cv::Mat& image_right ){
    /* FIRST thing that I want to do is the following:
        given a specific pixel in the left image, follow the epipolar line
        (using the notion of the depth of that pixel) and check where that pixel is located
        in the other image -> this is very important since the disparity should be
        consistent for the algorithm to work
    */
   //step uno -> take a point in the first image and underline it
   //step dwo -> use the depth of that pixel and see where it falls into the other image.

    //cv::circle(image_left, cv::Point(300, 1200), 2, cv::Scalar(255, 255, 255), -1);
    
    float baseline = 0.54;
    
    float focal = 9.597910e+02;
    //focal=9.569251e+02;
    
    float x_left = 700; //(it should be num cols [0 1242])
    float y_left = 250; //(it should be num rows [0 375])

    cv::circle(image_left, cv::Point((int)x_left,(int)y_left ), 3, cv::Scalar(255, 255, 255), 3);
    cv::imshow("first image", image_left);
    cv::waitKey(0);

    std::cout << "depth is" << initial_guess.at<float>((int)y_left,(int)x_left) << std::endl;

    float depth = initial_guess.at<float>((int)y_left,(int)x_left);
   
    std::cout << "depth is (NON BUONA): " << initial_guess.at<float>(10,10) << std::endl;

    float disparity = (baseline*focal)/depth;

    float x_right = x_left - disparity;
    std::cout << "x_right" << (int)x_right << std::endl;
   // x_right=1065;
    
    std::cout << "num row/cols is: " << image_right.rows << " " << image_right.cols<< std::endl;
    std::cout << "num row/cols is: " << image_left.rows << " " << image_left.cols<< std::endl;
    cv::circle(image_right, cv::Point((int)x_right,(int)y_left ), 3, cv::Scalar(255, 255, 255), 3);
    cv::imshow("second image", image_right);
    cv::waitKey(0);

   return ;
}


void get_initial_error(cv::Mat& initial_guess,cv::Mat& image_left,cv::Mat& image_right ){
    /* let's show the initial error for each pixel in gray formato
    */
   
    //conversion to grayscale
    cv::Mat image_left_gray;
    
    cv::cvtColor(image_left, image_left_gray, cv::COLOR_BGR2GRAY);
    //cv::imshow("second image", image_left_gray);
    //cv::waitKey(0);
    
    cv::Mat image_right_gray;
    cv::cvtColor(image_right, image_right_gray, cv::COLOR_BGR2GRAY);

    float baseline = 0.54;
    
    float focal = 9.597910e+02;
    //focal=9.569251e+02;
    int rows = image_left.rows;
    int cols = image_left.cols;
    int window_size = 2;
    int matches = 0;
    int row_cols=0;
    cv::Mat error_map = cv::Mat::zeros(rows, cols, CV_8U);
    for (int i = 0; i < rows; ++i ){
        for (int j = 0; j < cols; ++j){
            //get depth of the pixel
            float depth = initial_guess.at<float>(i,j);
            if (depth > 0){
                //get arrival point:
                float disparity = (baseline*focal)/depth;
                int pixel_reprojected = (int)( j - disparity);
                //std::cout << "I'M STARTING FROM: " << j << "AND ARRIVING AT: " << pixel_reprojected <<std::endl;
                //check if I'm within image boundaries
                
                //NB: MUST INVESTIGATE THE CASES IN WHICH IT FALLS OUT OF THE IMAGE
                //HOW DO I HANDLE THIS SITUATION??? -> IT MEANS THAT THE DEPTH IS INCORRECT
                int errorWindow_ij = 0;
                if (pixel_reprojected > 0 && pixel_reprojected < cols){
                    row_cols++;
                      //costruisco window a sx e matcho ogni pixel con window a destra
                    
                    for(int k = -window_size; k < window_size; k++ ){
                        for (int p = -window_size; p<window_size;p++){ 
                            if(i+p> 0 && i+p< rows && j+k>0 && j+k<cols && 
                               pixel_reprojected+k>0 && pixel_reprojected+k<cols){

                                    int left_value = (int)image_left_gray.at<uchar>(i+p,j+k);
                                    int right_value = (int)image_right_gray.at<uchar>(i+p,pixel_reprojected+k);
                                    errorWindow_ij += (left_value-right_value)*(left_value-right_value);

                                    matches++;

                            }
                        }
                    }
            
                }

                error_map.at<uchar>(i,j) = static_cast<uchar> (sqrt(errorWindow_ij));///255;
            }
        }
    }
    std::cout << "NUM MATCHES= " <<matches << std::endl;

    // if depth > 0 -> vedere bene questione di rows and cols, mesa che devo fa al contrario 
 

    
    cv::Mat error_map_show = cv::Mat::zeros(rows, cols, CV_8U);
    int counts=0;
    for (int r = 0; r < rows;++r){
        for(int c = 0; c < cols; c++){
            int val =(int)error_map.at<uchar>(r,c) ;
            //if(val >200){val = 0;}
            cv::circle(error_map_show, cv::Point(c, r), 1, cv::Scalar(val, val, val), -1);
            if ((int)error_map.at<uchar>(r,c) > 0){
                counts++;
            }
            //std::cout << (int)error_map.at<uchar>(r,c) << std::endl;
        }
    }
   
    std::cout << "num of different zero = " << counts << " num total pixels= " << row_cols <<std::endl;
    // Normalize the input matrix values to the range [0, 255]
   // cv::normalize(error_map, error_map_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    
   
    cv::imshow("second image", error_map_show);
    cv::waitKey(0);

   return ;
}

int main() {

    // TODO: allow to pass this parameters as an input to the function by joining paths... mo è na sbatta
    
    //2 e 3 so quelle a colori

    //projects a velodyne coordinate point into the camera_2 image
    //y_image = P2 * R0_rect * Tr_velo_to_cam * x_velo_coord

    std::string path_to_dir = "../stereo_lidar_ds/2011_09_26_drive_0002_sync";

    std::string left_img_path  = "../stereo_lidar_ds/2011_09_26_drive_0002_sync/image_02/data/0000000000.png";
    std::string right_img_path = "../stereo_lidar_ds/2011_09_26_drive_0002_sync/image_03/data/0000000000.png";
    std::string calibration_v2c_path = "../stereo_lidar_ds/2011_09_26_drive_0002_sync/2011_09_26_calib/2011_09_26/calib_velo_to_cam.txt";
    std::string calibration_ci2cj_path = "../stereo_lidar_ds/2011_09_26_drive_0002_sync/2011_09_26_calib/2011_09_26/calib_cam_to_cam.txt";
    std::string pc_bin_path="../stereo_lidar_ds/2011_09_26_drive_0002_sync/velodyne_points/data/0000000000.bin";

    cv::Mat img_left;
    cv::Mat img_right;
    
    Eigen::Matrix4f T_V_2_C0;
    Eigen::Matrix4f T_C0_2_C2;
    Eigen::Matrix4f T_C0_2_C3;
    
    Eigen::Matrix4f R_rect_02;
    Eigen::Matrix4f R_rect_03;
    Eigen::Matrix4f R_rect_00;
    
    Eigen::Matrix<float, 3, 4> P_rect_02;
    Eigen::Matrix<float, 3, 4> P_rect_03;

   

    int ok_read = read_kitti_ds(left_img_path,right_img_path,
                                 img_left,img_right,calibration_v2c_path,
                                 T_V_2_C0,
                                 calibration_ci2cj_path,
                                 T_C0_2_C2,
                                 T_C0_2_C3,
                                 R_rect_00,
                                 R_rect_02,
                                 R_rect_03,
                                 P_rect_02,
                                 P_rect_03);
  
    if (ok_read!=1){
         printf("Failed to load some data, read error above\n");
        return -1;
    }
    
    
    // FINAL TRANSFORM in order to project a lidar point into the image plane
    Eigen::Matrix<float, 3, 4> T_V_2_C2;
    Eigen::Matrix<float, 3, 4> T_V_2_C3;
    /*Tr_velo_to_cam: 
    7.533745000000e-03 -9.999714000000e-01 -6.166020000000e-04 -4.069766000000e-03 
    1.480249000000e-02 7.280733000000e-04 -9.998902000000e-01 -7.631618000000e-02 
    9.998621000000e-01 7.523790000000e-03 1.480755000000e-02 -2.717806000000e-01
    */
    std::cout<< "the value is " << T_C0_2_C2*T_V_2_C0<<std::endl;
    T_V_2_C2 = P_rect_02*R_rect_02*T_C0_2_C2*T_V_2_C0;
    //T_V_2_C2 = P_rect_02*T_C0_2_C2*T_V_2_C0;
    T_V_2_C3 = P_rect_03*R_rect_03*T_C0_2_C3*T_V_2_C0;
    //T_V_2_C3 = P_rect_03*T_C0_2_C3*T_V_2_C0;

    Eigen::Matrix4f T2 = R_rect_00*T_C0_2_C2*T_V_2_C0;
   
    Eigen::Matrix4f T3 = R_rect_03*T_C0_2_C3*T_V_2_C0;
        
    cv::Mat projected_depths = cv::Mat::zeros(img_left.rows, img_left.cols, CV_32F);
    cv::Mat dense_range_img;

    cv::Mat img_left_copy = img_left.clone();
    cv::Mat img_right_copy = img_right.clone();

    int left_camera = withSuperPixels(pc_bin_path, T2, P_rect_02, img_left, projected_depths,dense_range_img);

    //int left_camera  = vedi_pc(pc_bin_path, T2, P_rect_02, img_left, projected_depths,dense_range_img);
    //int right_camera = vedi_pc(pc_bin_path, T3, P_rect_02, img_right, projected_depths);

    //SHOULD BE LIKE THIS
    //int left_camera  = vedi_pc(pc_bin_path, T2, P_rect_02, img_left);
    //int right_camera = vedi_pc(pc_bin_path, T3, P_rect_03, img_right);

  
    //check to see if the reprojection on the epipolar line is consitent

    //UNCOMMENT THIS TO SEE THE REPROJECTION
    //check_reproj_epip(dense_range_img, img_left_copy,img_right_copy);

    //let's see the initial error map :) 

    get_initial_error(dense_range_img, img_left_copy,img_right_copy);

      // DO THE ACTUAL ALGORITHM 

    //cv::Mat lab_image;
    //cv::cvtColor(image, lab_image, cv::COLOR_BGR2Lab);
    //Slic slic;
  


    //PERFORM STEREO MATCHING
    int block_size = 4;
    int min_disp = -128;
    int max_disp = 128;
    int num_disp = max_disp - min_disp;
    int uniquenessRatio = 10;
    int speckleWindowSize = 200;
    int speckleRange = 2;
    int disp12MaxDiff = 0;
    int numDisparity = 4;
    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create();
    stereo->setNumDisparities(numDisparity * 16);
    stereo->setBlockSize(block_size);
    
    /*  STANDARD STEREO MATCHING RESULT (OPENCV)
        cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
        min_disp, 
        num_disp, 
        block_size, 
        uniquenessRatio, 
        speckleWindowSize, 
        speckleRange, 
        disp12MaxDiff, 
        8 * 1 * block_size * block_size, 
        32 * 1 * block_size * block_size
    );*/

    cv::Mat disparity_SGBM;
    cv::Mat final_img;
    stereo->compute(img_left, img_right, disparity_SGBM);

    cv::normalize(disparity_SGBM, disparity_SGBM, 255, 0, cv::NORM_MINMAX,CV_8U);
    //disparity_SGBM.convertTo(disparity_SGBM, CV_8U);

    //toColorImage(disparity_SGBM, final_img);
    disparity_SGBM.convertTo(final_img,CV_8U);
    cv::applyColorMap(final_img,final_img,cv::COLORMAP_JET);

    //UNCOMMENT THIS TO SHOW THE IMAGE
    //cv::imshow("Depth Map", disparity_SGBM);
    //cv::imshow("Depth Map", final_img);
//
//
    //cv::waitKey(0);

    return 0;

}
//
////perform rectification


///////////////////////////JUST TRASH

/*
int rectify() {
    // Load the stereo camera parameters (calibration matrices and distortion coefficients)
    cv::FileStorage fs("stereo_calibration_params.xml", cv::FileStorage::READ);
    cv::Mat camera_matrix1, camera_matrix2, dist_coeffs1, dist_coeffs2, R, T;
    fs["camera_matrix1"] >> camera_matrix1;
    fs["camera_matrix2"] >> camera_matrix2;
    fs["dist_coeffs1"] >> dist_coeffs1;
    fs["dist_coeffs2"] >> dist_coeffs2;
    fs["R"] >> R;
    fs["T"] >> T;
    fs.release();

    // Load the left and right images
    cv::Mat img_left = cv::imread("left_image.jpg", cv::IMREAD_GRAYSCALE);
    cv::Mat img_right = cv::imread("right_image.jpg", cv::IMREAD_GRAYSCALE);

    // Rectify the stereo images
    cv::Mat rectified_img_left, rectified_img_right;
    cv::Mat R1, R2, P1, P2, Q;
    cv::Size img_size = img_left.size();
    cv::stereoRectify(camera_matrix1, dist_coeffs1, camera_matrix2, dist_coeffs2, img_size, R, T, R1, R2, P1, P2, Q);
    cv::initUndistortRectifyMap(camera_matrix1, dist_coeffs1, R1, P1, img_size, CV_32FC1, rect_map_left_x, rect_map_left_y);
    cv::initUndistortRectifyMap(camera_matrix2, dist_coeffs2, R2, P2, img_size, CV_32FC1, rect_map_right_x, rect_map_right_y);
    cv::remap(img_left, rectified_img_left, rect_map_left_x, rect_map_left_y, cv::INTER_LINEAR);
    cv::remap(img_right, rectified_img_right, rect_map_right_x, rect_map_right_y, cv::INTER_LINEAR);

    // Save the rectified images
    cv::imwrite("rectified_left.jpg", rectified_img_left);
    cv::imwrite("rectified_right.jpg", rectified_img_right);

    return 0;
}
*/




// SUPER WOWWW

// THIS WORKS FOR AN ONLINE TUNING OF THE PARAMETERS OF THE STEREO MATCHER

//int numDisparity = 8;
//int blockSize = 5;
//cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create();
//cv::Mat disp,disparity; //Disparity
//cv::Mat imgL;
//cv::Mat imgR;
//std::string disparity_window = "disparity";
//
//static void trackbar1(int , void* )
//{
//	stereo->setNumDisparities(numDisparity * 16);
//	numDisparity = numDisparity * 16;
//	stereo->compute(imgL,imgR,disp);
//	disp.convertTo(disparity,CV_8U);
//	cv::applyColorMap(disparity,disparity,cv::COLORMAP_JET);
//	cv::imshow(disparity_window.c_str(),disparity);
//}
//
//static void trackbar2(int , void* )
//{
//	stereo->setBlockSize(blockSize);
//  blockSize = blockSize;
//	stereo->compute(imgL,imgR,disp);
//	disp.convertTo(disparity,CV_8U);
//	cv::applyColorMap(disparity,disparity,cv::COLORMAP_JET);
//
//	cv::imshow(disparity_window.c_str(),disparity);
//}
//
//
//int main() {
//
//
//	
//
//    std::string left_img  = "../stereo_lidar_ds/2011_09_26_drive_0002_sync/image_02/data/0000000000.png";
//    std::string right_img = "../stereo_lidar_ds/2011_09_26_drive_0002_sync/image_03/data/0000000000.png";
//
//
//	imgL = cv::imread(left_img);
//	imgR = cv::imread(right_img);
//
//
//
//	cv::namedWindow(disparity_window);
//
//	cv::createTrackbar("numDisparities", disparity_window.c_str(), &numDisparity, 18, trackbar1);
//	cv::createTrackbar("blockSize", disparity_window.c_str(), &blockSize, 50, trackbar2);
//
//
//	cv::waitKey(0);
//	return 0;
//
//}