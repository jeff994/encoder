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

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "encoder");
  	ros::NodeHandle n;
 	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("encoder", 1000);
	serial::Serial my_serial("/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75533353637351616171-if00", 4800, serial::Timeout::simpleTimeout(1000));

   	ros::Rate loop_rate(12);

   	if(my_serial.isOpen())
    		std::cout << " Yes." << std::endl;
  	else
	{
    		std::cout << " The serial port is not correct." << std::endl;
		return 0; 
	}

  	int count = 0;
 

 	while (true)
 	{
		ROS_INFO("Start loop");
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
			std::cout << "message is not correct" << std::endl;
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
   		 ++count;
		ROS_INFO("End loop");
  	}
  	return 0;
}
