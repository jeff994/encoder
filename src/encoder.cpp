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

serial::Serial my_serial; 

void initSerial()
{
	my_serial.setPort("/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75533353637351616171-if00");
 	serial::Timeout::simpleTimeout(1000);
 	serial::Timeout timeout	= serial::Timeout::simpleTimeout(1000);
 	my_serial.setTimeout(timeout);
 	my_serial.setBaudrate (4800);
}

bool openSerial()
{
	if(my_serial.isOpen())
		return true; 
	my_serial.open();
	return my_serial.isOpen();
} 

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "encoder");
  	ros::NodeHandle n;
 	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("encoder", 1000);

 	initSerial(); 

 	while (ros::ok())
 	{
		ROS_INFO("Start loop");
		if(openSerial() == false)
			ROS_INFO("Serial port is not connected, trying to open agin...")
			usleep(100000);
			continue; 
		std_msgs::String msg;


    	std::string result; 
		
   	 	size_t n_size = my_serial.readline(result); 
		std::stringstream ss(result);
		
		std::istream_iterator<std::string> begin(ss);
		std::istream_iterator<std::string> end;
		std::vector<std::string> vstrings(begin, end);
    		 ROS_INFO("%s", result.c_str());
		if(vstrings.size() != 4)
		{	
			ROS_ERROR("Messge %s format is not correct",result.c_str());
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
			ssout <<  value[0] << " "  << value[2];
			msg.data = ssout.str() ;
			ROS_INFO("%s", msg.data.c_str());
			chatter_pub.publish(msg);

		}	
    	ros::spinOnce();

    		//loop_rate.sleep();
		ROS_INFO("End loop");
  	}
  	return 0;
}
