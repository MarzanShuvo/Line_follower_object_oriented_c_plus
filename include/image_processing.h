#pragma once
#include <opencv4/opencv2/opencv.hpp>
#include "opencv4/opencv2/core/core.hpp"
#include "opencv4/opencv2/imgproc/imgproc.hpp"
#include "opencv4/opencv2/highgui/highgui.hpp"
#include <iostream>
#include <vector>
#include<cmath>
#include<tuple>

namespace robot{
    class Image_Processing{
        public:

            Image_Processing(cv::Mat image);
            //converting image to gray
            void image_convert_to_gray();
            //Gaussian Blur operation to reduce noise
            void noise_removal();
            cv::Mat get_image();
            // //preprocessing image for thresholding 
            void image_thresholding();
            // //bitwise operation is for reverse operation in image 
            void bitwise_not_operation();

            // //generating a black image, it will be used for segmenting the image into half
            void make_black_image();
            cv::Mat get_black_image_up();
            cv::Mat get_black_image_lower();
            // //creating upperhalf image
            void getUpperHalfMask();
            // //creating lowerhalf image
            void getLowerHalfMask();
            //created upper half mask marged with real to find contours point in the upper portion
            //of image
            void upper_mask();
            //created lower half mask marged with real to find contour point in the upper portion
            //of image
            void lower_mask();
             // //find contour contours points 
            std::vector<std::vector<cv::Point> > find_contours(cv::Mat image);
            //find centroid and draw the point and contours
            std::tuple <int, int> find_centroid_draw_frame(cv::Mat image);
            //find maximum contour from contours
            int getMaxAreaContourId(std::vector < std::vector<cv::Point>> contours);
            //find upper centroid and lower centroid
            std::tuple <int, int, int, int> find_upper_lower_centroid();
            //line draw in frame
            cv::Mat drawed_frame();
            //find error from the centroid
            void find_error();
            //final function to get the error after processing the image
            std::tuple<double, double> processing_image_and_get_error();
            
            

            



        public:
            cv::Mat image_read_;
            cv::Mat original_image_;
            int rows_;
            int cols_;
            double theta_upper;
            double theta_lower;
            double error_w1;
            double error_w2;

        private:
            cv::Mat black_image_up_;
            cv::Mat black_image_lower_;
    };
}