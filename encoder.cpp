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


// global setting for the serial port for encoder
serial::Serial encoder_serial;
std::string sPort("/dev/serial/by-id/uusb-Arduino__www.arduino.cc__Arduino_Uno_854383630393512042A0-if00");
int nBaudrate = 4800; 
encoder_serial.setPort(sPort);
encoder_serial.setBaudrate(nBaudrate);
encoder_serial.setTimeout(1000);
ros::Publisher encoder_pub = n.advertise<std_msgs::String>("encoder", 1000);

bool OpenSerial()
{
    if(encoder_serial.isOpen()) return true; 
    encoder_serial.open();
    return encoder_serial.isOpen();
}

// Convert a hex string to interfer 
// true if conversion is successful 
// false if the input length is 0 
bool HexStringToInt(std::string input, int &n_x)
{
	if(input.size() == 0) 
		return false; 
	std::stringstream ss;
	ss << std::hex << input.c_str();
	ss >> n_x;
	return true; 
}

bool GetBatteryLevel(std::string input, int & n_x)
{
	if(input.size() != 2) 
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
	if(input.size()!=4) 
		return false; 
	int n_x = 0; 
	bool bRet = HexStringToInt(input, n_x); 
	foo = std::bitset<16>(n_x);
	return true;
}

bool GetEncoderValue(std::string input, int & n_x)
{
	if(input.size()!=4) 
		return false; 
	
	bool bRet = HexStringToInt(input, n_x); 
	
	if(!bRet) return false; 

	std::bitset<16> foo(n_x);
	
	// If first bit is 1 then the value should be negative (backward )
	if(foo.test(15) == 1)
	{
		foo.set(15, 0);
		n_x = foo.to_ulong(); 
		n_x = -n_x;
	}
	// Otherwise forward 
	else
	{
		n_x = foo.to_ulong(); 
	}
	return true; 
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "encoder");
  	ros::NodeHandle n;
 	while (ros::ok())
		ROS_INFO("Start loop");
		std_msgs::String msg;
    	std::string result; 
		if(encoder_serial() == false) 
		{
            ROS_INFO("Failed to open serial port, try again ...");
            usleep(milliseconds*1000);
            continue;
        }

        // list of parameters need to be parsed from the serial port 
		int left_encode, right_encoe, yaw, pitch, roll; 
		bool yaw_direction, pictch_direction, roll_direction; 
		bool motor_1_ok, motor_2_ok, encoder_ok, giro_scope_ok, reverse_senor_ok,  distance_senor_ok;   
		bool is_manual_mode; 
		bool is_obstacle_avoidence_mode; 
		bool has_obstancle;
		bool is_interaction_mode;
		int direction;


   	 	size_t n_size = encoder_serial.readline(result); 
		std::stringstream ss(result);
		
		std::string sVlaue = ss.str(); 
		
		std::bitset<16> bitSetParameters; 
		bool bRet		= GetEncoderValue	(sVlaue.substr(2,4),	left);
		bRet			= GetEncoderValue	(sVlaue.substr(6,4),	right); 
		bRet			= HexStringToInt	(sVlaue.substr(10,2),	yaw);
		bRet			= HexStringToInt	(sVlaue.substr(12,2),	pitch);
		bRet			= HexStringToInt	(sVlaue.substr(14,2),	roll);
		bRet			= GetOtherFlags		(sVlaue.substr(16,4),	bitSetParameters);


		// Reading the parameters from bitset parameters 

		yaw_direction				= bitSetParameters.test(15); 
		pictch_direction			= bitSetParameters.test(14); 
		roll_direction				= bitSetParameters.test(13); 
		motor_1_ok					= bitSetParameters.test(12); 
		motor_2_ok					= bitSetParameters.test(11); 
		is_manual_mode				= bitSetParameters.test(10); 
		is_obstacle_avoidence_mode	= bitSetParameters.test(9); 
		has_obstancle				= bitSetParameters.test(8); 
		encoder_ok					= bitSetParameters.test(7); 
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

	/*    	
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
		}	*/
    	
        ros::spinOnce();
		ROS_INFO("End loop");
  	}
  	return 0;
}
