/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include <ros/ros.h>
#include "serial_write_proto.h"
#include <vector>
// float angles[5] = {0};
// std::vector<float> angles(5);
// std::vector<float> Pre_vels_copy(5);
// #include <detect/armor_goal.h>
// #include <decision/shoot_info.h>
// roborts_msgs::ArmorDetectionFeedbackConstPtr feedback;
// detect::armor_goal vision_data;


  bool comp(const float &a, const float &b)
  {
    //   return abs(a) > abs(b);
    return a>b;
  }

/* maybe i should flite it zai shi jue zheli */
  float PreFilter(float Fangle)
  {
    float angles[2] = {0.0};
    // std::vector<float> angles(3);
    
    // std::vector<float> Pre_vels_copy(3);
    // angles[0] = angles[1];
    // angles[1] = angles[2];
    // angles[2] = angles[3];
    angles[0] = angles[1];
    angles[1] = Fangle;
    // angles[5] = angles[6];
    // angles[6] = angles[7];
    // angles[7] = angles[8];
    // angles[8] = Fangle;

    // Pre_vels_copy = angles;
    // sort(Pre_vels_copy.begin(), Pre_vels_copy.end(), comp);

    return (angles[0]+angles[1])/2;
    
    
  }

void vision_callback(const roborts_msgs::ArmorDetectionFeedbackConstPtr & armor_data)
{
    std::vector<geometry_msgs::Point>::const_iterator it = armor_data->enemy_pos.begin() ;

    cout <<"armor_data->enemy_pos.size(): "<< armor_data->enemy_pos.size() << endl;

    for (int i = 0; it != armor_data->enemy_pos.end(); ++it,++i)
    {
        double yaw   = it->x; // yaw
        double pitch = it->y; // pitch
        double z     = it->z; // distance

        cout << yaw << "\t" << pitch << "\t" << z << "\t" << i << endl;
    }
}

void GimbalAngleCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg)
{
    // ROS_ERROR("yaw: %f  pitch: %f", msg->yaw_angle, msg->pitch_angle);
    
        s2c.yaw_angle   = msg->yaw_angle ;
        s2c.pitch_angle = -(msg->pitch_angle) ;
        if(s2c.pitch_angle > 0)
        s2c.pitch_angle = s2c.pitch_angle +2;
        if(s2c.pitch_angle > 12 || s2c.pitch_angle < -2.5)
        s2c.pitch_angle = PreFilter(s2c.pitch_angle);
        // ROS_ERROR("%f", s2c.pitch_angle);
    
    // ROS_ERROR("%d", serial_cnt);
    
        //TODO: other member in the struct

}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "serial_Write");

    ros::NodeHandle nh;
    ros::Subscriber ros_sub_cmd_gimbal_angle_ = nh.subscribe("cmd_gimbal_angle", 1, GimbalAngleCallback);
    // ros::Subscriber sub_armor = nh.subscribe("armor_info", 5, vision_callback); // update data
    // ros::Subscriber sub_shoot = nh.subscribe("shoot_info", 5, shoot_callback); // update data
    ros::Rate loopRate(100);
    
    // init serial
    serial_mul::serial_write serial;
    
    while(1)
    {
        ros::spinOnce();
        serial.write_To_serial(s2c); // pub data
        loopRate.sleep();
    }

  return 0;
}
