/**
    
    @file motor_driver.cpp
    @author Marzan Alam
    @version 1.0 9/01/22 
*/

#include "motor_driver.h"

namespace robot{

    Motor_Driver::Motor_Driver(){
         width = 0.2;
         diameter = 0.06;
         offset_speed = 8;
    }

    std::tuple <double, double> Motor_Driver::move_robot(double error_w1=0.0, double error_w2=0.0){
        angular_vel_base = 0.2;
        linear_vel_base = 0.3;
        FACTOR_LINEAR_e1 = 0.01;
        FACTOR_ANGULAR_e1 = 0.01;
        FACTOR_ANGULAR_e2 = 0.02;
        double angular = 0.08* angular_vel_base * error_w1* FACTOR_ANGULAR_e1 + 0.07 * error_w2* FACTOR_ANGULAR_e2;
        double linear = .006;
        return std::make_tuple(linear, angular);
    }

    std::tuple <int, int> Motor_Driver::diff_drive(double v, double w){
        double vr = double(2*v+w*width)/diameter;
        double vl = double(2*v-w*width)/diameter;
        double wheel_rotation_l = double (vl*2)/diameter;
        double wheel_rotation_r = double (vr*2)/diameter;
        int rpm_l = int(wheel_rotation_l*60/(2*3.1416));
        int rpm_r = int(wheel_rotation_r*60/(2*3.1416));
        rpm_l = rpm_l-offset_speed;
        if(rpm_l<0){
            rpm_l = -1*rpm_l;
        }
        if(rpm_r<0){
            rpm_r = -1*rpm_r;
        }

        return std::make_tuple(rpm_l, rpm_r);
        
    }


}