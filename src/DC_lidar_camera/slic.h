#ifndef SLIC_H
#define SLIC_H

//
// #include <opencv/cv.h>
// #include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <math.h>
#include <vector>
#include <float.h>
using namespace std;

/* 2d matrices are handled by 2d vectors. */
#define vec2dd vector<vector<double>>
#define vec2di vector<vector<int>>
#define vec2db vector<vector<bool>>
/* The number of iterations run by the clustering algorithm. */
#define NR_ITERATIONS 10

/*
 * class Slic.
 *
 * In this class, an over-segmentation is created of an image, provided by the
 * step-size (distance between initial cluster locations) and the colour
 * distance parameter.
 */

class Slic
{
//private:
public:
    /* The cluster assignments and distance values for each pixel. */
    vec2di clusters; // 
    vec2dd distances;

    /* The LAB and xy values of the centers. */
    vec2dd centers;
    /* The number of occurences of each center. */
    vector<int> center_counts;

    /* The step size per cluster, and the colour (nc) and distance (ns)
     * parameters. */
    int step, nc, ns;

    /* Compute the distance between a center and an individual pixel. */
    double compute_dist(int ci, cv::Point pixel, cv::Scalar colour);
    /* Find the pixel with the lowest gradient in a 3x3 surrounding. */
    cv::Point find_local_minimum(cv::Mat &image, cv::Point center);

    /* Remove and initialize the 2d vectors. */
    void clear_data();
    void init_data(cv::Mat &image);

//public:
    /* Class constructors and deconstructors. */
    Slic();
    ~Slic();

    /* Generate an over-segmentation for an image. */
    void generate_superpixels(cv::Mat &image, int step, int nc);
    /* Enforce connectivity for an image. */
    void create_connectivity(cv::Mat &image);
    void create_connectivity_original(cv::Mat &image);

    /* Draw functions. Resp. displayal of the centers and the contours. */
    void display_center_grid(cv::Mat &image, cv::Scalar colour);
    void display_contours(cv::Mat &image, cv::Scalar colour);
    void colour_with_cluster_means(cv::Mat &image);
};

#endif

///////////////////////////////////////7

/*
void Slic::create_connectivity(cv::Mat& image) {
    int label = 0, adjlabel = 0;
    const int lims = (image.cols * image.rows) / ((int)centers.size());
    
    const int dx4[4] = {-1,  0,  1,  0};
	const int dy4[4] = { 0, -1,  0,  1};
    
    // Initialize the new cluster matrix. 
    vec2di new_clusters;
    for (int i = 0; i < image.cols; i++) { 
        vector<int> nc;
        for (int j = 0; j < image.rows; j++) {
            nc.push_back(-1);
        }
        new_clusters.push_back(nc);
    }

    for (int i = 0; i < image.cols; i++) {
        for (int j = 0; j < image.rows; j++) {
            if (new_clusters[i][j] == -1) {
                vector<cv::Point> elements;
                elements.push_back(cv::Point(i, j));
            
                //Find an adjacent label, for possible use later. 
                for (int k = 0; k < 4; k++) {
                    int x = elements[0].x + dx4[k], y = elements[0].y + dy4[k];
                    
                    if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                        std::cout<< " new cluster vaue is "<< new_clusters[x][y]<<std::endl;
                        if (new_clusters[x][y] >= 0) {
                            adjlabel = new_clusters[x][y];
                        }
                    }
                }
                
                int count = 1;
                for (int c = 0; c < count; c++) {
                    for (int k = 0; k < 4; k++) {
                        int x = elements[c].x + dx4[k], y = elements[c].y + dy4[k];
                        
                        if (x >= 0 && x < image.cols && y >= 0 && y < image.cols) {
                            if (new_clusters[x][y] == -1 && clusters[i][j] == clusters[x][y]) {
                                elements.push_back(cv::Point(x, y));
                                new_clusters[x][y] = label;
                                count += 1;
                            }
                        }
                    }
                }
                
                // use the earlier found adjacent label if a segment size is smaller than a limit. 
                if (count <= lims >> 2) {
                    for (int c = 0; c < count; c++) {
                        new_clusters[elements[c].x][elements[c].y] = adjlabel;
                    }
                    label -= 1;
                }
                label += 1;
            }
        }
    }
    //for (int j = 0; j < image.cols; j++) {
    //        for (int k = 0; k < image.rows; k++) {
    //            //cluster id for each pixel in the cluster
    //            std::cout<<"pixel in pos "<<j<< ", " << k << " in cluster "<<clusters[j][k]<<std::endl;
    //        }
    // }
}
*/