#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <iterator>

// global setting for the serial port for encoder
serial::Serial encoder_serial;
std::string sPort("/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0");
int nBaudrate = 4800; 
encoder_serial.setPort(sPort);
encoder_serial.setBaudrate(nBaudrate);
encoder_serial.setTimeout(1000);

bool OpenSerial()
{
    if(encoder_serial.isOpen()) return true; 
    encoder_serial.open();
    return encoder_serial.isOpen();
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "encoder");
  	ros::NodeHandle n;
 	while (ros::ok())
 	{
		ROS_INFO("Start loop");
		std_msgs::String msg;
    	std::string result; 
		if(encoder_serial() == false) 
            ROS_INFO("Failed to open serial port, try again ...");
            usleep(milliseconds*1000)
            continue 

   	 	size_t n_size = encoder_serial.readline(result); 
		std::stringstream ss(result);
		
		std::istream_iterator<std::string> begin(ss);
		std::istream_iterator<std::string> end;
		std::vector<std::string> vstrings(begin, end);
    	ROS_INFO("%s", result.c_str());
		if(vstrings.size() != 4)
		{	
			ROS_INFO("Message %s format is not correct", result.c_str());
            continue;
		}
		else
		{
			int value[4];
			for(int i =0; i < 4; ++i)
			{
				value[i] = atoi(vstrings[i].c_str()); 
			}
			if(value[1] == 0) value[0] = -value[0];
			if(value[3] == 0) value[2] = -value[2]; 
			std::stringstream ssout;
			ssout <<  value[2] << " "  << value[0];
			msg.data = ssout.str() ;
			ROS_INFO("%s", msg.data.c_str());
            chatter_pub.publish(msg);
		}	
    	
        ros::spinOnce();
		ROS_INFO("End loop");
  	}
  	return 0;
}
