/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include <ros/ros.h>
#include "serial_read_proto.h"
// #include <serial/car_info.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "serial_Read");
    ros::NodeHandle nh;
    // ros::Publisher pub = nh.advertise<const roborts_msgs::GimbalAngle::ConstPtr>("car_info", 100);
    // ros::Publisher pub = nh.advertise<const roborts_msgs::
    ros::Publisher pub = nh.advertise<roborts_msgs::InfoFromCar>("car_info", 100);
    
    ros::Rate loopRate(50);

    // init serial
    serial_mul::serial_read serial;
    
    while(nh.ok() )
    {
        ros::spinOnce();
        serial.read_from_serial();
        // ROS_ERROR("Is't ok 2");
        pub.publish(serial.info);
        
        loopRate.sleep();
    }
    return 0;
}
