#include "image_processing.h"

namespace robot{
    
    Image_Processing::Image_Processing(cv::Mat image): image_read_{image}{
        rows_ = image_read_.rows;
        cols_ = image_read_.cols;
        original_image_ = image_read_;

    }

    void Image_Processing::image_convert_to_gray(){
        cv::cvtColor(image_read_, image_read_, cv::COLOR_BGR2GRAY);
    }

    void Image_Processing::noise_removal(){
        cv::GaussianBlur(image_read_,image_read_, cv::Size(9, 9), 0);
    }

    void Image_Processing::image_thresholding(){
        cv::threshold(image_read_, image_read_, 200, 255, cv::THRESH_BINARY);
    }

    cv::Mat Image_Processing::get_image(){
        return image_read_;
    }

    void Image_Processing::bitwise_not_operation(){
        cv::bitwise_not(image_read_, image_read_);
    }

    void Image_Processing::make_black_image(){
        black_image_up_ = cv::Mat::zeros(cv::Size(cols_, rows_),CV_8UC1);
        black_image_lower_ = cv::Mat::zeros(cv::Size(cols_, rows_),CV_8UC1);
    }

    cv::Mat Image_Processing::get_black_image_up(){
        return black_image_up_;
    }

    cv::Mat Image_Processing::get_black_image_lower(){
        return black_image_lower_;
    }

    void Image_Processing::getUpperHalfMask(){
        int width = black_image_up_.size().width;
        int height = black_image_up_.size().height/2;
        for(int i=0; i<width; i++){
            for(int j=0; j<height; j++){
                black_image_up_.at<uchar>(j,i) = 255;
            }
        }
    }

    void Image_Processing::getLowerHalfMask(){
        int width = black_image_lower_.size().width;
        int height = black_image_lower_.size().height;
        for(int i=0; i<width; i++){
            for(int j=height/2; j<height; j++){
                black_image_lower_.at<uchar>(j,i) = 255;
            }
        }
    }
    
    void Image_Processing::upper_mask(){
        cv::bitwise_and(image_read_,black_image_up_, black_image_up_);
    }

    void Image_Processing::lower_mask(){
        cv::bitwise_and(image_read_,black_image_lower_, black_image_lower_);
    }

    std::vector<std::vector<cv::Point> > Image_Processing::find_contours(cv::Mat image){
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours( image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
        return contours;
    }

    int Image_Processing::getMaxAreaContourId(std::vector < std::vector<cv::Point>> contours){
        double maxArea = 0;
        int maxAreaContourId = -1;
        for (int j = 0; j < contours.size(); j++) {
            double newArea = cv::contourArea(contours.at(j));
            if (newArea > maxArea) {
                maxArea = newArea;
                maxAreaContourId = j;
            } // End if
        } // End for
        return maxAreaContourId;
    }

    std::tuple <int, int> Image_Processing::find_centroid_draw_frame(cv::Mat image){
        auto contours = find_contours(image);
        if(contours.size()>0){
            auto contour1 = contours.at(getMaxAreaContourId(contours));
            if(contour1.size() != 0){
                auto moments = cv::moments(contour1,true);
                auto cx = int(moments.m10/moments.m00);
                auto cy = int(moments.m01/moments.m00);
                cv::drawContours(original_image_, std::vector<std::vector<cv::Point>>{contour1}, -1, cv::Scalar(0, 255, 0), 2);
                cv::Point centerCircle(cx,cy);
                int radiusCircle = 10;
                cv::Scalar colorCircle1(0,255,255);
                int thicknessCircle = 4;
                cv::circle(original_image_, centerCircle, radiusCircle, colorCircle1, thicknessCircle);
                return std::make_tuple(cx, cy);
            }
        }
        else{
            return std::make_tuple(0, 0);
        }
    }

    std::tuple <int, int, int, int> Image_Processing::find_upper_lower_centroid(){
        int upper_cx, upper_cy, lower_cx, lower_cy;
        std::tie(upper_cx, upper_cy) = find_centroid_draw_frame(black_image_up_);
        std::tie(lower_cx, lower_cy) = find_centroid_draw_frame(black_image_lower_);
        return std::make_tuple(upper_cx, upper_cy, lower_cx, lower_cy);
    }

    cv::Mat Image_Processing::drawed_frame(){
        return original_image_;
    }

    void Image_Processing::find_error(){
        int upper_cx, upper_cy, lower_cx, lower_cy;
        std::tie(upper_cx, upper_cy, lower_cx, lower_cy) = find_upper_lower_centroid();
        int ori_cx = cols_/2;
        cv::Point p1(ori_cx,  rows_), p2(lower_cx,lower_cy), p3(upper_cx, upper_cy);
        if(p1.x != p2.x){
            double y1 = (double) (p2.y - p1.y);
            double x1 = (double) (p2.x - p1.x);
            theta_lower = -1*atan2(y1,x1)*180/3.1415;
        }
        if(p2.x != p3.x){
            double y2 = (double) (p3.y - p2.y); 
            double x2 = (double) (p3.x - p2.x);
            theta_upper = -1*atan2(y2,x2)*180/3.1415;
        }
        error_w1 = theta_lower-90;
        error_w2 = theta_upper-90;
        cv::line(original_image_, p1,p2, cv::Scalar(255,0,0), 3, cv::LINE_8);
        cv::line(original_image_, p2,p3, cv::Scalar(255,0,0), 3, cv::LINE_8);
        cv::line(original_image_, p1,p3, cv::Scalar(0,0,255), 3, cv::LINE_8);
        cv::Point centerCircle3(ori_cx, rows_);
        int radiusCircle = 10;
        cv::Scalar colorCircle3(0,255,255);
        int thicknessCircle = 4;
        cv::circle(original_image_, centerCircle3, radiusCircle, colorCircle3, thicknessCircle);
    }


    std::tuple<double, double> Image_Processing::processing_image_and_get_error(){
        image_convert_to_gray();
        noise_removal();
        image_thresholding();
        bitwise_not_operation();
        make_black_image();
        getLowerHalfMask();
        getUpperHalfMask();
        upper_mask();
        lower_mask();
        find_error();
        return std::make_tuple(error_w1, error_w2);

    }





}