/**
    CS-11 Asn 2: Calculates the total of 6 checks
    @file motor_driver.h
    @author Marzan Alam
    @version 1.0 9/01/22 
*/

#include<cmath>
#include<tuple>

namespace robot{
    class Motor_Driver{
        public:
            Motor_Driver();
            //calculate linear and angular velocity
            std::tuple <double, double> move_robot(double error_w1, double error_w2);
            //calculate linear and angular rpm
            std::tuple <int, int> diff_drive(double v, double w);


        private:
            double angular_vel_base;
            double linear_vel_base;
            double FACTOR_LINEAR_e1;
            double FACTOR_ANGULAR_e1;
            double FACTOR_ANGULAR_e2;

            double width;
            double diameter;
        public:
            int offset_speed;

    };
}