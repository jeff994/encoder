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
#include <math.h>
#include <bitset>

serial::Serial my_serial; 
void initSerial()
{
	my_serial.setPort("/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75630313536351217041-if00");
 	serial::Timeout::simpleTimeout(1000);
 	serial::Timeout timeout	= serial::Timeout::simpleTimeout(1000);
 	my_serial.setTimeout(timeout);
 	my_serial.setBaudrate (115200);
}

bool openSerial()
{
	if(my_serial.isOpen())
		return true; 
	my_serial.open();
	return my_serial.isOpen();
}


// Convert a hex string to interfer 
// true if conversion is successful 
// false if the input length is 0 
bool HexStringToInt(std::string input, int &n_x)
{
	if(input.size() != 1) 
		return false; 
	std::stringstream ss;
	std::bitset<8> bits(input[0]);
	n_x = bits.to_ulong();
	return true; 
}

bool GetBatteryLevel(std::string input, int & n_x)
{
	if(input.size() != 1) 
		return false; 
	bool bRet = HexStringToInt(input, n_x);
	if(!bRet) 
		return false; 
	std::bitset<8> foo(n_x);
	if(foo.test(7) == 1)
	{
		foo.set(7, 0);
	}
	// Otherwise forward 
	n_x = foo.to_ulong(); 
	return true; 
}

int GetDirection(std::bitset<16> foo)
{
	for(int i = 3; i < 15; i++)
	{
		foo.set(i, 0);
	}
	int n_x = foo.to_ulong(); 
	return n_x;
}

bool GetOtherFlags(std::string input, std::bitset<16> &foo)
{
	if(input.size() != 2) 
		return false; 
	int n_x = 0; 
	bool bRet = HexStringToInt(input, n_x); 
	std::bitset<8> lower(input[1]);
	std::bitset<8> upper(input[0]);
	for(int i = 0; i < 8; ++i)
	{
	 	foo.set(i, lower[i]);
		foo.set(i + 8, upper[i]);
	}
	return true;
}

bool GetEncoderValue(std::string input, int & n_x)
{
	if(input.size()!= 2) 
		return false; 
	
	std::bitset<8> lower(input[1]);
	std::bitset<8> upper(input[0]);

	std::bitset<16> foo(n_x);
	for(int i = 0; i < 8; ++i)
	{
	 	foo.set(i, lower[i]);
		foo.set(i + 8, upper[i]);
	}
	
	// If first bit is 1 then the value should be negative (backward )
	if(foo.test(15) == 1)
	{
		foo.set(15, 0);
		n_x = foo.to_ulong(); 
		n_x = n_x;
	}
	// Otherwise forward 
	else
	{
		n_x = -foo.to_ulong(); 
	}
	return true; 
} 

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "encoder");
  	ros::NodeHandle n;
	ros::Publisher encoder_pub = n.advertise<std_msgs::String>("encoder", 1000);

 	initSerial(); 

 	while (ros::ok())
 	{
		ROS_INFO("Start loop");
		if(openSerial() == false)
		{
			ROS_INFO("Serial port is not connected, trying to open agin...");
			usleep(100000);
			continue; 
		}

		std_msgs::String msg;
    		std::string result; 
		
   	 	size_t n_size = my_serial.readline(result); 
		
		ROS_INFO("value have size: %d", n_size);

		if(n_size >12) 
		{
			int left_encode, right_encode, yaw, pitch, roll; 
			bool yaw_direction, pictch_direction, roll_direction; 
			bool motor_1_ok, motor_2_ok, encoder_ok, giro_scope_ok, reverse_senor_ok,  distance_senor_ok;   
			bool is_manual_mode; 
			bool is_obstacle_avoidence_mode; 
			bool has_obstancle;
			bool is_interaction_mode;
			int direction;
		
			std::stringstream ss(result);
		
			std::string sVlaue = ss.str();
			ROS_INFO("value have: %s", sVlaue.c_str());
			std::bitset<16> bitSetParameters; 
			bool bRet		= GetEncoderValue	(sVlaue.substr(1, 2),	left_encode);
			bRet			= GetEncoderValue	(sVlaue.substr(3, 2),	right_encode); 
			bRet			= HexStringToInt	(sVlaue.substr(5, 1),	yaw);
			bRet			= HexStringToInt	(sVlaue.substr(6, 1),	pitch);
			bRet			= HexStringToInt	(sVlaue.substr(7, 1),	roll);
			bRet			= GetOtherFlags		(sVlaue.substr(8, 2),	bitSetParameters);


		// Reading the parameters from bitset parameters 

			yaw_direction				= bitSetParameters.test(15); 
			pictch_direction			= bitSetParameters.test(14); 
			roll_direction				= bitSetParameters.test(13); 
			motor_1_ok				= bitSetParameters.test(12); 
			motor_2_ok				= bitSetParameters.test(11); 
			is_manual_mode				= bitSetParameters.test(10); 
			is_obstacle_avoidence_mode		= bitSetParameters.test(9); 
			has_obstancle				= bitSetParameters.test(8); 
			encoder_ok				= bitSetParameters.test(7); 
			giro_scope_ok				= bitSetParameters.test(6); 
			reverse_senor_ok			= bitSetParameters.test(5); 
			distance_senor_ok			= bitSetParameters.test(4); 
			is_interaction_mode			= bitSetParameters.test(3); 
		
			direction					= GetDirection(bitSetParameters); 


			// after getting all the data from input, now it's publishing the data to tos 

			// Step1: publish the encoder data 
			std::stringstream ssout;
			ssout <<  left_encode << " "  << right_encode;
			msg.data = ssout.str() ;
			ROS_INFO("%s", msg.data.c_str());
        		encoder_pub.publish(msg);
		}
	
    		ros::spinOnce();

    		//loop_rate.sleep();
		ROS_INFO("End loop");
  	}
  	return 0;
}
