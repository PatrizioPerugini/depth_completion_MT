#include "slic.h"

/*
 * Constructor. Nothing is done here.
 */
Slic::Slic() {

}
Slic::~Slic() {
    clear_data();
}
void Slic::clear_data() {
    clusters.clear();
    distances.clear();
    centers.clear();
    center_counts.clear();
}
//initialitation is done by setting label to -1 and distance to infinity for each pixel
void Slic::init_data(cv::Mat &image) {
    /* Initialize the cluster and distance matrices. */
    for (int i = 0; i < image.cols; i++) { 
        vector<int> cr;
        vector<double> dr;
        for (int j = 0; j < image.rows; j++) {
            cr.push_back(-1);
            dr.push_back(FLT_MAX);
        }
        clusters.push_back(cr);
        distances.push_back(dr);
    }
    
    /* Initialize the centers and counters. */
    for (int i = step; i < image.cols - step/2; i += step) {
        for (int j = step; j < image.rows - step/2; j += step) {
            vector<double> center;
            /* Find the local minimum (gradient-wise) to avoid centering superpixels on an edge. */

            cv::Point nc = find_local_minimum(image, cv::Point(i,j));
            //cv::Point nc(i,j);
            cv::Vec3b pixel = image.at<cv::Vec3b>(nc.y, nc.x);
            cv::Scalar colour(pixel[0], pixel[1], pixel[2]);
            
            
            /* Generate the center vector. */
            //C_k = [L_k,A_k,B_k,X_k,Y_k]
            center.push_back(colour.val[0]);
            center.push_back(colour.val[1]);
            center.push_back(colour.val[2]);
            center.push_back(nc.x);
            center.push_back(nc.y);
            
            /* Append to vector of centers. */
            centers.push_back(center);
            center_counts.push_back(0);
        }
    }
}

double Slic::compute_dist(int ci, cv::Point pixel, cv::Scalar colour) {
    double dc = sqrt(pow(centers[ci][0] - colour.val[0], 2) + pow(centers[ci][1]
            - colour.val[1], 2) + pow(centers[ci][2] - colour.val[2], 2));
    double ds = sqrt(pow(centers[ci][3] - pixel.x, 2) + pow(centers[ci][4] - pixel.y, 2));
    
    return sqrt(pow(dc / nc, 2) + pow(ds / ns, 2));
    
    //double w = 1.0 / (pow(ns / nc, 2));
    //return sqrt(dc) + sqrt(ds * w);
}

//Return new point for the center
cv::Point Slic::find_local_minimum(cv::Mat& image, cv::Point center) {
    double min_grad = FLT_MAX;
    cv::Point loc_min = cv::Point(center.x, center.y);
    
    for (int i = center.x-1; i < center.x+2; i++) {
        for (int j = center.y-1; j < center.y+2; j++) {
            
            cv::Vec3b c1 = image.at<cv::Vec3b>(j+1, i);
            cv::Vec3b c2 = image.at<cv::Vec3b>(j, i+1);
            cv::Vec3b c3 = image.at<cv::Vec3b>(j, i);
            double i1 = c1[0];
            double i2 = c2[0];
            double i3 = c3[0];

            // Compute horizontal and vertical gradients and keep track of the minimum. 
            // G(x,y) = ‖I(x+ 1,y)−I(x−1,y)‖²+‖I(x,y+ 1)−I(x,y−1)‖²

   
            if (sqrt(pow(i1 - i3, 2)) + sqrt(pow(i2 - i3,2)) < min_grad) {
                min_grad = fabs(i1 - i3) + fabs(i2 - i3);
                loc_min.x = i;
                loc_min.y = j;
            }
        }
    }
    
    return loc_min;
}

 
void Slic::generate_superpixels(cv::Mat& image, int step, int nc) {
    this->step = step;
    this->nc = nc;
    this->ns = step;
    
    // Clear previous data (if any), and re-initialize it. 
    clear_data();
    init_data(image);
    
    //Run EM for 10 iterations (as prescribed by the algorithm). 
    for (int i = 0; i < NR_ITERATIONS; i++) {
        /* Reset distance values. */
        for (int j = 0; j < image.cols; j++) {
            for (int k = 0;k < image.rows; k++) {
                distances[j][k] = FLT_MAX;
            }
        }

        for (int j = 0; j < (int) centers.size(); j++) {
            // Only compare to pixels in a 2 x step by 2 x step region. 
            // Maybe          ------------->   k < centers[j][3] + step/2 -1
            for (int k = centers[j][3] - step; k < centers[j][3] + step; k++) {
                for (int l = centers[j][4] - step; l < centers[j][4] + step; l++) {
                
                    if (k >= 0 && k < image.cols && l >= 0 && l < image.rows) {
                        //Compute the distance between point and cluster
                        cv::Vec3b pixel = image.at<cv::Vec3b>(l, k);
                        cv::Scalar colour(pixel[0], pixel[1], pixel[2]);
                        double d = compute_dist(j, cv::Point(k,l), colour);
                        
                        // Update cluster allocation if the cluster minimizes the distance. 
                        if (d < distances[k][l]) {
                            distances[k][l] = d;
                            clusters[k][l] = j;//label for every pixel
                        }
                    }
                }
            }
        }
        
        /* Clear the center values. */
        for (int j = 0; j < (int) centers.size(); j++) {
            centers[j][0] = centers[j][1] = centers[j][2] = centers[j][3] = centers[j][4] = 0;
            center_counts[j] = 0;
        }
        
        /* Compute the new cluster centers. */
        for (int j = 0; j < image.cols; j++) {
            for (int k = 0; k < image.rows; k++) {
                //cluster id for each pixel in the cluster
                int c_id = clusters[j][k];
                //if this pixel is assignet to superpixel
                if (c_id != -1) {
                    cv::Vec3b pixel = image.at<cv::Vec3b>(k, j);
                    cv::Scalar colour(pixel[0], pixel[1], pixel[2]);
                    centers[c_id][0] += colour.val[0];
                    centers[c_id][1] += colour.val[1];
                    centers[c_id][2] += colour.val[2];
                    centers[c_id][3] += j;
                    centers[c_id][4] += k;
                    
                    center_counts[c_id] += 1;
                }
            }
        }

        /* Normalize the clusters. */
        for (int j = 0; j < (int) centers.size(); j++) {
            centers[j][0] /= center_counts[j];
            centers[j][1] /= center_counts[j];
            centers[j][2] /= center_counts[j];
            centers[j][3] /= center_counts[j];
            centers[j][4] /= center_counts[j];
        }
    }
    //for (int j = 0; j < image.cols; j++) {
    //        for (int k = 0; k < image.rows; k++) {
    //            //cluster id for each pixel in the cluster
    //            std::cout<<"pixel in pos "<<j<< ", " << k << " in cluster "<<clusters[j][k]<<std::endl;
    //        }
    // }
}



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
            
                /* Find an adjacent label, for possible use later. */
                for (int k = 0; k < 4; k++) {
                    int x = elements[0].x + dx4[k], y = elements[0].y + dy4[k];
                    
                    if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                       // std::cout<< " new cluster vaue is "<< new_clusters[x][y]<<std::endl;
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
                
                /* Use the earlier found adjacent label if a segment size is
                   smaller than a limit. */
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



void Slic::create_connectivity_original(cv::Mat& image) {
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
            
                /* Find an adjacent label, for possible use later. */
                for (int k = 0; k < 4; k++) {
                    int x = elements[0].x + dx4[k], y = elements[0].y + dy4[k];
                    
                    if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
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
                
                /* Use the earlier found adjacent label if a segment size is
                   smaller than a limit. */
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


void Slic::display_center_grid(cv::Mat& image, cv::Scalar colour) {
    for (int i = 0; i < (int) centers.size(); i++) {
        cv::circle(image, cv::Point(centers[i][3], centers[i][4]), 2, colour, 2);
        //cvCircle(image, cvPoint(centers[i][3], centers[i][4]), 2, colour, 2);
    }
}



void Slic::display_contours(cv::Mat& image, cv::Scalar colour) {
    const int dx8[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
	const int dy8[8] = { 0, -1, -1, -1, 0, 1, 1,  1};
	
	/* Initialize the contour vector and the matrix detailing whether a pixel
	 * is already taken to be a contour. */
	vector<cv::Point> contours;
	vec2db istaken;
	for (int i = 0; i < image.cols; i++) { 
        vector<bool> nb;
        for (int j = 0; j < image.rows; j++) {
            nb.push_back(false);
        }
        istaken.push_back(nb);
    }
    
    /* Go through all the pixels. */
    for (int i = 0; i < image.cols; i++) {
        for (int j = 0; j < image.rows; j++) {
            int nr_p = 0;
            
            /* Compare the pixel to its 8 neighbours. */
            for (int k = 0; k < 8; k++) {
                int x = i + dx8[k], y = j + dy8[k];
                
                if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                    if (istaken[x][y] == false && clusters[i][j] != clusters[x][y]) {
                        nr_p += 1;
                    }
                }
            }
            
            ///* Add the pixel to the contour list if desired. */
            if (nr_p >= 2) {
                contours.push_back(cv::Point(i,j));
                istaken[i][j] = true;
            }
        }
    }
    
    /* Draw the contour pixels. */
    for (int i = 0; i < (int)contours.size(); i++) {
        cv::Point point(contours[i].x, contours[i].y);
        cv::Vec3b color(colour[0], colour[1], colour[2]);
        image.at<cv::Vec3b>(point.y, point.x) = color;
        //cvSet2D(image, contours[i].y, contours[i].x, colour);
    }
}

/* Give the pixels of each cluster the same colour values. The specified colour
 * is the mean RGB colour per cluster.
 *
 * Input : The target image (IplImage*).
 * Output: -
 */
void Slic::colour_with_cluster_means(cv::Mat& image) {
    vector<cv::Scalar> colours(centers.size());
    
    /* Gather the colour values per cluster. */
    for (int i = 0; i < image.cols; i++) {
        for (int j = 0; j < image.rows; j++) {
            int index = clusters[i][j];
            cv::Vec3b colour = image.at<cv::Vec3b>(j, i);

            //cv::Scalar colour = cvGet2D(image, j, i);
            
            
            //colours[index].val[0] += colour.val[0];
            //colours[index].val[1] += colour.val[1];
            //colours[index].val[2] += colour.val[2];
            colours[index].val[0] += colour[0];
            colours[index].val[1] += colour[1];
            colours[index].val[2] += colour[2];
        }
    }
    
    /* Divide by the number of pixels per cluster to get the mean colour. */
    for (int i = 0; i < (int)colours.size(); i++) {
        colours[i].val[0] /= center_counts[i];
        colours[i].val[1] /= center_counts[i];
        colours[i].val[2] /= center_counts[i];
    }
    
    /* Fill in. */
    for (int i = 0; i < image.cols; i++) {
        for (int j = 0; j < image.rows; j++) {
            //cv::Scalar ncolour = colours[clusters[i][j]];
            cv::Vec3b ncolour(colours[clusters[i][j]].val[0],
                              colours[clusters[i][j]].val[1],
                              colours[clusters[i][j]].val[2]);

            image.at<cv::Vec3b>(j, i) = ncolour;

            //cvSet2D(image, j, i, ncolour);
        }
    }
}