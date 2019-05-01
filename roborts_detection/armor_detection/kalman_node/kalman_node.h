#pragma once 

#include<iostream>
#include<opencv2/opencv.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include "io/io.h"
#include "state/error_code.h"
#include "roborts_msgs/ArmorDetectionAction.h"
#include <roborts_msgs/GimbalAngle.h>
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/FricWhl.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "roborts_msgs/GimbalInfo.h"
#include "roborts_msgs/InfoFromCar.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//#include <gimbal_control.h>
//#include "proto/gimbal_control.pb.h"

// #include "roborts_msgs/GlobalPlannerAction.h"
// #include "roborts_msgs/LocalPlannerAction.h"

#include <thread>
#include <condition_variable>
#include <geometry_msgs/Twist.h>





class kalman_node
{
private:
    cv::RotatedRect armor_rect_;

    cv::RotatedRect PreRect;

    double y_gimbal =0;
    double p_gimbal =0;
    double his_y_gimbal =0;
    double his_p_gimbal =0;
    double vx_gimbal =0;
    double vy_gimbal =0;
    float  vel_x =0;
    float  vel_y =0;
    
    roborts_msgs::GimbalInfo gimbal_info_;
    
    

    //actionlib::SimpleActionClient<roborts_msgs::GimbalInfo> kalman_actionlib_client_;

    void ListenYaw(roborts_msgs::InfoFromCar msg)
    {
        // gimbal_info_.mode             = msg->mode;
        // gimbal_info_.pitch_ecd_angle  = msg->pitch_ecd_angle;
        // gimbal_info_.yaw_ecd_angle    = msg->yaw_ecd_angle;
        // gimbal_info_.pitch_gyro_angle = msg->pitch_gyro_angle;
        // gimbal_info_.yaw_gyro_angle   = msg->yaw_gyro_angle;
        // gimbal_info_.yaw_rate         = msg->yaw_rate;
        // gimbal_info_.pitch_rate       = msg->pitch_rate;
        gimbal_info_.yaw_ecd_angle = msg.yaw_angle;
        gimbal_info_.pitch_ecd_angle = msg.pitch_angle;
        // ROS_ERROR("gimbal_info_.mode: %d ", gimbal_info_.mode );

      ROS_ERROR("gimbal_info_.pitch_ecd_angle: %f ",  gimbal_info_.pitch_ecd_angle);
      ROS_ERROR("gimbal_info_.yaw_ecd_angle: %f ",    gimbal_info_.yaw_ecd_angle );
    //   ROS_ERROR("gimbal_info_.pitch_gyro_angle: %f ", gimbal_info_.pitch_gyro_angle);
    //   ROS_ERROR("gimbal_info_.yaw_gyro_angle: %f ",   gimbal_info_.yaw_gyro_angle);
    //   ROS_ERROR("gimbal_info_.yaw_rate: %f ",         gimbal_info_.yaw_rate);
    //   ROS_ERROR("gimbal_info_.pitch_rate: %f ",       gimbal_info_.pitch_rate);
      
    }



    
    ros::Rate s;
public:
    kalman_node():
    s(100)
    // kalman_actionlib_client_("car_info", true)
    {
        // init_K();
    }

    // bool comp(const float &a, const float &b);
    float PreFilter(float Pre_vel);
    cv::RotatedRect KF(cv::RotatedRect armor_rect);
    void init_K()   //useless
    {
        ros_nh_                      = ros::NodeHandle();
        ros_sub_gimbal_              = ros_nh_.subscribe("car_info", 1, &kalman_node::ListenYaw, this);  //useless
        ros::spinOnce();s.sleep();
        ros::spinOnce();s.sleep();
        ros::spinOnce();s.sleep();
        ros::spinOnce();s.sleep();
        

        // ros::AsyncSpinner async_spinner(1);
        // async_spinner.start();
        // ros::waitForShutdown();
        // s.sleep();
    }
      void CarInfoCallback(roborts_msgs::InfoFromCar msg)
  {
    // pass_pitch_angle = msg.pitch_angle;
    // pass_pitch_rate  = msg.pitch_rate;
    // pass_yaw_angle   = msg.yaw_angle;
    // pass_yaw_rate    = msg.yaw_rate;

    // ROS_ERROR("armor_node: pass_pitch_angle: %f", pass_pitch_angle);
    // ROS_ERROR("armor_node: pass_yaw_angle: %f", pass_yaw_angle);
    // ROS_ERROR("armor_node: pass_pitch_rate: %f", pass_pitch_rate);
    // ROS_ERROR("armor_node: pass_yaw_rate: %f", pass_yaw_rate);
    
    // Kl.yaw_rate = msg.yaw_rate;
    // Kl.pitch_rate = msg.pitch_rate;
    // ROS_ERROR("Kl.pitch_rate: %f ",  msg.pitch_rate);
    // ROS_ERROR("Kl.yaw_rate: %f ",    msg.yaw_rate);

  }

    ros::NodeHandle ros_nh_;
    ros::Subscriber ros_sub_gimbal_;

    float kalman_pitch_rate;
    float kalman_yaw_rate;
    float kalman_pitch_angle;
    float kalman_yaw_angle;
};

