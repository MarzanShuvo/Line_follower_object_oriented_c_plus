#include <opencv4/opencv2/opencv.hpp>
#include "opencv4/opencv2/core/core.hpp"
#include "opencv4/opencv2/imgproc/imgproc.hpp"
#include "opencv4/opencv2/highgui/highgui.hpp"
#include<image_processing.h>
#include<motor_driver.h>
#include <iostream>
#include<tuple>

int main(int argc, char **argv) {

    cv::VideoCapture video_capture;
    if (!video_capture.open(0)) {
        return 0;
    }
    
    cv::Mat frame;
    while (true) {

        //offset_speed in left is 8rpm.
        double error1, error2, linear, angular;
        int rpm_l, rpm_r;
        video_capture >> frame;
        robot::Image_Processing take_image(frame);
        std::tie(error1, error2) = take_image.processing_image_and_get_error();
        robot::Motor_Driver motors;
        std::tie(linear, angular) = motors.move_robot(error1, error2);
        std::tie(rpm_l, rpm_r) = motors.diff_drive(linear, angular);
        cv::Mat black_up = take_image.get_black_image_up();
        cv::Mat black_low = take_image.get_black_image_lower();
        cv::Mat image = take_image.get_image();
        cv::Mat drawed = take_image.drawed_frame();
        imshow("Frame", image);
        //imshow("black_up", black_up);
        //imshow("black_low", black_low);
        imshow("drawed ", drawed );
        //std::cout<<"error1: "<<error1<<"error2: "<<error2<<std::endl;
        std::cout<<"left RPM: "<<rpm_l+8<<" Right RPM: "<<rpm_r<<std::endl;

        int esc_key = 27;
        if (cv::waitKey(10) == esc_key) {
            break;
        }
    }
}