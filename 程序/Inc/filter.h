#ifndef __FILTER_H
#define __FILTER_H

/********ÉùÃ÷º¯Êý********/
float Kalman_Filter_x(float Accel,float Gyro);
float Complementary_Filter_x(float angle_m, float gyro_m);
float Kalman_Filter_y(float Accel,float Gyro);
float Complementary_Filter_y(float angle_m, float gyro_m);
#endif
