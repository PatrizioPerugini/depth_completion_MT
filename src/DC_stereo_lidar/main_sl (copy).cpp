#include <opencv2/opencv.hpp>
#include <iostream>
#include <random>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Dense> 

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include "../DC_lidar_only/img_completion.cpp"

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

//

struct LidarPoint { // single lidar point in space
    double x,y,z,r; // x,y,z in [m], r is point reflectivity
};

template<typename T> void read_pod(std::ifstream& in, T& t)
{
    in.read(reinterpret_cast<char*>(&t), sizeof(T));
}

template<typename T> void read_pod_vector(std::ifstream& in, std::vector<T>& vect)
{
    long size;
    
    read_pod(in, size);
    for(int i = 0;i < size;++i)
    {
        T t;
        read_pod(in, t);
        vect.push_back(t);
    }
}

//

void readLidarPts(const char* fileName, std::vector<LidarPoint> &output)
{
    std::ifstream in(fileName);
    read_pod_vector(in, output);
}

void toColorImage(const cv::Mat &r_img, cv::Mat &color_img)
{
    cv::Mat normalize_dense_r_img;
    cv::normalize(r_img, normalize_dense_r_img, 1.0, 0, cv::NORM_MINMAX);
    // CV_8UC1 -> a 8-bit single-channel array
    cv::Mat dense_r_img_CV_8UC1;
    normalize_dense_r_img.convertTo(dense_r_img_CV_8UC1, CV_8UC1, 255.0);
    cv::applyColorMap(dense_r_img_CV_8UC1, color_img, cv::COLORMAP_JET);
}

//check if the images are rectified or not

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

int read_pc(std::string infile,Eigen::Matrix<float, 3, 4>& T,cv::Mat& img_left){
    	// load point cloud
	//fstream input(infile.c_str(), std::ios::in | std::ios::binary);
	//if(!input.good()){
	//	std::cerr << "Could not read file: " << infile << std::endl;
	//	exit(EXIT_FAILURE);
	//}
	//boost::iostreams::input.seekg(0, std::ios::beg);
//
	//pcl::PointCloud<PointXYZI>::Ptr points (new pcl::PointCloud<PointXYZI>);
//
	//int i;
	//for (i=0; boost::iostreams::input.good() && !boost::iostreams::input.eof(); i++) {
	//	PointXYZI point;
	//	boost::iostreams::input.read((char *) &point.x, 3*sizeof(float));
	//	boost::iostreams::input.read((char *) &point.intensity, sizeof(float));
	//	points->push_back(point);
	//}
	//boost::iostreams::input.close();
   std::ifstream input(infile, std::ios::binary);

    if (!input.is_open()) {
        std::cerr << "Could not open file: " << infile << std::endl;
        return 1;
    }

    std::vector<Eigen::Vector4f> points;
    std::vector<Eigen::Vector3f> points_in_cam;
    int in_=0;
    int out_=0;

   

    Eigen::Vector4f point;
    while (input.read(reinterpret_cast<char*>(&point), sizeof(PointXYZI))) {
        //if (point[0]>0 && point[1]>0){ 
            points.push_back(point);
            Eigen::Vector4f hom_p = point;
            hom_p[3] = 1.0f;
            Eigen::Vector3f p(point[0], point[1], point[2]);
            Eigen::Vector3f transformed_point = T*hom_p;
            transformed_point[0]/=transformed_point[2];
            transformed_point[1]/=transformed_point[2];
             //if (transformed_point(0) >= 0 && transformed_point(0) <= img_left.cols && transformed_point(1) >= 0 && transformed_point(1) <= img_left.rows) {
                if(transformed_point[2] > 0){
                points_in_cam.push_back(transformed_point.head<3>());
        }
            

    }
    input.close();
    cv::Mat depthImage = cv::Mat::zeros(img_left.size(), CV_8UC3);
    for (const Eigen::Vector3f& camPoint : points_in_cam) {
        int u = static_cast<int>(camPoint(0));
        int v = static_cast<int>(camPoint(1));
        float z = camPoint(2);

        if (u >= 0 && u < img_left.cols && v >= 0 && v < img_left.rows) {
            //cv::Vec3b color = img_left.at<cv::Vec3b>(v, u);
            double minDepth = 0.0;  // Min depth value
            double maxDepth = 100.0; // Max depth value (adjust as needed)
            double normalizedDepth = (z - minDepth) / (maxDepth - minDepth);
            //normalizedDepth = z;
            cv::Scalar color = cv::Scalar::all(255 * normalizedDepth);
            //cv::Scalar color = cv::Scalar::all( normalizedDepth);
            unsigned char r = static_cast<unsigned char>(color[0]);
            unsigned char g = static_cast<unsigned char>(color[1]);
            unsigned char b = static_cast<unsigned char>(color[2]);

            // Draw points on the depth image with colormap color
            depthImage.at<cv::Vec3b>(v, u) = cv::Vec3b(b, g, r);
            //depthImage.at<cv::Vec3b>(v, u) = color;
            cv::Point circleCenter(u, v);
            cv::circle(img_left, circleCenter, 2, color, -1);
        }
    }

    //depth image has the depths
    

    
    
    //for (const Eigen::Vector3f& p : points_in_cam) {
    //    std::cout << "X: " << p[0] << " Y: " << p[1] << " Z: " << p[2] << " Intensity: " << "p[3]" << std::endl;
    //}
    //    std::cout << "number of in / out is" << in_ << " " << out_ << std::endl;

}

/*
P_rect_00: 
7.215377e+02 0.000000e+00 6.095593e+02 0.000000e+00 
0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 
0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00
0               0               0            1
*/



int vedi_pc(std::string inputFilePath,Eigen::Matrix4f& T, Eigen::Matrix<float, 3, 4>& P,
            cv::Mat & image){
    
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
    cv::Mat projected_depths = cv::Mat::zeros(image.rows, image.cols, CV_32F);

   

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
            //cv::circle(image, cv::Point(u, v), 2, cv::Scalar(255, 255, 255), -1);
            cv::circle(image, cv::Point(u, v), 2, cv::Scalar(projectedPoint(2),0, 0), -1);
            projected_depths.at<float>(v, u) = projectedPoint(2);
          
        }
    }
 
    cv::Mat normalized_depths;
    cv::normalize(projected_depths, normalized_depths, 0, 255, cv::NORM_MINMAX);

///prova

    cv::Mat dense_range_img;
    cv::Mat range_img;

    std::string blur_type = std::string("gaussian");
    bool extr = false;
    cv::imshow("src", image);
    //cv::imshow("src", dense_range_img);
    cv::waitKey(0);
    img_completion(normalized_depths, dense_range_img, extr, blur_type);
    cv::Mat color_dense_range_img;

    toColorImage(dense_range_img, color_dense_range_img);
    std::vector<cv::Mat> hImg;

     std::cout << "have been projected " << projected << std::endl;
    cv::imshow("Projected Point Cloud", normalized_depths);
    cv::waitKey(0);
    cv::Mat cat_img;
    //cv::vconcat(hImg, cat_img);
    cv::imshow("src", color_dense_range_img);
    //cv::imshow("src", dense_range_img);
    cv::waitKey(0);
    //std::cout << "DONE" << std::endl;
//
    //cv::imshow("Projected Point Cloud", depthImage);
    //cv::waitKey(0);


    //for (int i = 0; i < color_dense_range_img.rows; ++i){
    //    for (int j = 0; j < color_dense_range_img.cols; ++j){
    //            std::cout << dense_range_img.at<float>(i,j)<<std::endl;
    //    }
    //}
   



    // Visualize the point cloud


    //pcl::visualization::CloudViewer viewer("Lidar Point Cloud Viewer (transformed)");
    //viewer.showCloud(transformedCloud);
//
    //while (!viewer.wasStopped())
    //{
    //}

    return 0;
}






int main() {

    // TODO: allow to pass this parameters as an input to the function by joining paths... mo è na sbatta
    
    //2 e 3 so quelle a colori

    //projects a velodyne coordinate point into the camera_2 image
    //y_image = P2 * R0_rect * Tr_velo_to_cam * x_velo_coord

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
    
    // Load the original input images and the rectified output images

    // check done, images are rectified
    //std::cout<< "the dimension of the image is: " << img_right.rows << " X " << img_right.cols << std::endl; 

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
    T_V_2_C2 = P_rect_02*T_C0_2_C2*T_V_2_C0;
    T_V_2_C3 = P_rect_03*R_rect_03*T_C0_2_C3*T_V_2_C0;
    T_V_2_C3 = P_rect_03*T_C0_2_C3*T_V_2_C0;

    Eigen::Matrix4f T2 = R_rect_02*T_C0_2_C2*T_V_2_C0;
   
    Eigen::Matrix4f T3 = R_rect_03*T_C0_2_C3*T_V_2_C0;

    int left_camera  = vedi_pc(pc_bin_path, T2, P_rect_02, img_left);
    int right_camera = vedi_pc(pc_bin_path, T3, P_rect_03, img_right);

    //now I need to read the point cloud(.bin) and project each point into the image plane using
    // T_V_2_Ci
    int rpc = read_pc(pc_bin_path, T_V_2_C2, img_left);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            std::cout << T_V_2_C3(i,j) << " ";
        }
        std::cout << "\n";
    }

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

    int tot_ = 0;
    int num = 0;
    // Display the depth map
    //cv::normalize(depth_map, depth_map, 0, 255, cv::NORM_MINMAX);
    for (int i = 0; i < disparity_SGBM.rows;i++){
        for (int j = 0; j < disparity_SGBM.cols; j++){
            num++;
            if (disparity_SGBM.at<float>(i,j) > 0){
                tot_++;
            }
            
        }
    }
    std::cout << tot_ << std::endl;
    std::cout << num << std::endl;
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