/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#ifndef _SERIAL_READ_PROTO_H_
#define _SERIAL_READ_PROTO_H_

#include <ros/ros.h>
// #include <serial/car_info.h>

#include "roborts_msgs/ArmorDetectionAction.h"
#include <roborts_msgs/GimbalAngle.h>
#include "roborts_msgs/InfoFromCar.h"


#include <stdint.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "LinuxSerial.hpp"

using namespace std;
using namespace boost::asio;

namespace serial_mul
{


class serial_read
{
    unsigned char data[10];
    int     data_len;
    int16_t init_yaw;

public:
    
    roborts_msgs::InfoFromCar info;
    

public:
    serial_read() {}
    ~serial_read() { if(pSerialPort) delete pSerialPort; }

public:
	void read_from_serial()
    {
        serial.ReadData(data, 10);
        if (data[0] == 0xDA && data[9] == 0xDB)
        {
            info.pitch_angle = int16_t((data[3] << 8) | (data[4]) )/10.0;   //slove it ....
            info.yaw_angle   = int16_t((data[1] << 8) | (data[2]) )/10.0;
            info.pitch_rate  = int16_t((data[7] << 8) | (data[8]) )/10.0;
            info.yaw_rate    = int16_t((data[5] << 8) | (data[6]) )/10.0;

            // ROS_ERROR("yaw_angle: %f", info.yaw_angle);
            // ROS_ERROR("pitch_angle: %f", info.pitch_angle);
            // ROS_ERROR("yaw_rate: %f", info.yaw_rate);
            // ROS_ERROR("pitch_rate: %f", info.pitch_rate);

            // pass_yaw_rate = info.pitch_rate;
            // pass_pitch_rate = info.yaw_rate;
            // pass_yaw_angle = info.yaw_angle;
            // pass_pitch_angle = info.pitch_angle;
        }

    }


private:
	bool init_port( const std::string port, const unsigned int char_size = 8)
    {
        if (!pSerialPort) return false;

        pSerialPort->open( port, ec );
        
        pSerialPort->set_option( serial_port::baud_rate( 115200 ), ec );
        pSerialPort->set_option( serial_port::flow_control( serial_port::flow_control::none ), ec );
        pSerialPort->set_option( serial_port::parity( serial_port::parity::none ), ec );
        pSerialPort->set_option( serial_port::stop_bits( serial_port::stop_bits::one ), ec);
        pSerialPort->set_option( serial_port::character_size( char_size ), ec);
    
        return true;
    }

    io_service m_ios;				// io_service Object
	serial_port *pSerialPort;		// Serial port Object
	std::string port_id;	    	// For save com name
	boost::system::error_code ec;	// Serial_port function exception
    CLinuxSerial serial;
};

} // namespace serial_mul

#endif