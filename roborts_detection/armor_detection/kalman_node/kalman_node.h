#pragma once 

#include<iostream>
#include<opencv2/opencv.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"
#include <vector>

#include <ros/ros.h>

#include "roborts_msgs/GimbalInfo.h"
#include "roborts_msgs/InfoFromCar.h"

class kalman_node
{
private:
    cv::RotatedRect armor_rect_;
    cv::RotatedRect hisRect;
    cv::RotatedRect hisprerect;
    cv::RotatedRect PreRect;

    int loop_cnt;
    int breaking_cnt;

    float vel_x;
    float vel_y;
    float his_vx;
    float max_vel_x;
    float min_vel_x;
    float op_max_vel_x;
    float op_min_vel_x;
    float his_max_vel_x;
    float his_min_vel_x;
    float his_vel_x;
    float old_his_vel_x;

    roborts_msgs::GimbalInfo gimbal_info_;
    
public:
    kalman_node()
    {
      loop_cnt = 0;
      breaking_cnt = 0;

      vel_x = 0;
      vel_y = 0;
      his_vx =0;
      max_vel_x =0;
      min_vel_x =100000;
      op_max_vel_x =0;
      op_min_vel_x =-100000;
      his_max_vel_x =0;
      his_min_vel_x =0;
      his_vel_x = 0;
    }

    float PreFilter(float Pre_vel);

    cv::RotatedRect KF(cv::RotatedRect armor_rect);

    void VelFliter(float &vel_x);

    ros::NodeHandle ros_nh_;
    ros::Subscriber ros_sub_gimbal_;

    float kalman_pitch_rate;
    float kalman_yaw_rate;
    float kalman_pitch_angle;
    float kalman_yaw_angle;
};

