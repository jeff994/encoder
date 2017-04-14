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
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
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

int main(int argc, char **argv)
{
        ros::init(argc, argv, "send_unlock");
        ros::NodeHandle n;
        ros::Publisher encoder_pub = n.advertise<std_msgs::String>("send_unlock", 1000);

        initSerial();

        if (ros::ok())
        {
		if(openSerial() == false)
                {
                        ROS_INFO("Serial port is not connected, trying to open agin...");
                        usleep(100000);
                        continue;
                }
		std_string test_string	= "iap_jump_app\r\n\0";
		//send commands and rest
		size_t test_bool = my_serial.write(test_string);
		usleep(100000);
		size_t test_bool = my_serial.write(test_string);
                usleep(100000);
		size_t test_bool = my_serial.write(test_string);
                usleep(100000);
                size_t test_bool = my_serial.write(test_string);
                usleep(100000);
		size_t test_bool = my_serial.write(test_string);
                usleep(100000);
                size_t test_bool = my_serial.write(test_string);
                usleep(100000);
                size_t test_bool = my_serial.write(test_string);
                usleep(100000);
                size_t test_bool = my_serial.write(test_string);
                usleep(100000);

	}
	return 0;
}
