/*
 * epuck_driver_my.cpp
 *
 *  Created on: Oct 13, 2016
 *      Author: josl
 */
#include <iostream>
#include <ros/ros.h>

#include <BluetoothConnection.h>
#include <RosWrapper.h>
#include <BasicCppDriver.h>

#define DEBUG_ROS_PARAMS 1

int main(int argc,char *argv[])
{


	    ros::init(argc, argv, "epuck_driver_cpp");

	    ros::NodeHandle np("~"); // Private.

		int epuck_id;
		std::string epuck_address;
		std::string epuck_name;

		double init_xpos;
		double init_ypos;
		double init_theta;
		bool enabled_sensors[SENSORS_NUM];
		int cam_width;
		int cam_height;
		int cam_zoom;
		int cam_mode;
		int cam_x_offset;
		int cam_y_offset;

	    np.getParam("epuck_id", epuck_id);
	    np.param<std::string>("epuck_address", epuck_address, "");
	    np.param<std::string>("epuck_name", epuck_name, "epuck");
	    np.param("xpos", init_xpos, 0.0);
	    np.param("ypos", init_ypos, 0.0);
	    np.param("theta", init_theta, 0.0);
	    np.param("accelerometer", enabled_sensors[ACCELEROMETER], false);
	    np.param("motor_speed", enabled_sensors[MOTOR_SPEED], false);
	    np.param("floor", enabled_sensors[FLOOR], false);
	    np.param("proximity", enabled_sensors[PROXIMITY], false);
	    np.param("motor_position", enabled_sensors[MOTOR_POSITION], false);
	    np.param("microphone", enabled_sensors[MICROPHONE], false);
	    np.param("camera", enabled_sensors[CAMERA], false);
	    np.param("cam_width", cam_width, 160);
	    np.param("cam_height", cam_height, 2);
	    np.param("cam_zoom", cam_zoom, 1);
	    np.param("cam_mode", cam_mode, 0);
	    np.param("cam_x_offset", cam_x_offset, -1);
	    np.param("cam_y_offset", cam_y_offset, -1);


	    if(DEBUG_ROS_PARAMS) {
	        std::cout << "[" << epuck_name << "] " << "epuck id: " << epuck_id<< std::endl;
	        std::cout << "[" << epuck_name << "] " << "epuck address: " << epuck_address << std::endl;
	        std::cout << "[" << epuck_name << "] " << "epuck name: " << epuck_name << std::endl;
	        std::cout << "[" << epuck_name << "] " << "init pose: " << init_xpos << ", " << init_ypos << ", " << init_theta << std::endl;
	        std::cout << "[" << epuck_name << "] " << "accelerometer enabled: " << enabled_sensors[ACCELEROMETER] << std::endl;
	        std::cout << "[" << epuck_name << "] " << "motor speed enabled: " << enabled_sensors[MOTOR_SPEED] << std::endl;
	        std::cout << "[" << epuck_name << "] " << "floor enabled: " << enabled_sensors[FLOOR] << std::endl;
	        std::cout << "[" << epuck_name << "] " << "proximity enabled: " << enabled_sensors[PROXIMITY] << std::endl;
	        std::cout << "[" << epuck_name << "] " << "motor position enabled: " << enabled_sensors[MOTOR_POSITION] << std::endl;
	        std::cout << "[" << epuck_name << "] " << "microphone enabled: " << enabled_sensors[MICROPHONE] << std::endl;
	        std::cout << "[" << epuck_name << "] " << "camera enabled: " << enabled_sensors[CAMERA] << std::endl;
	        std::cout << "[" << epuck_name << "] " << "image size: " << cam_width << " x " << cam_height << std::endl;
	        std::cout << "[" << epuck_name << "] " << "image zoom: " << cam_zoom << std::endl;
	        std::cout << "[" << epuck_name << "] " << "image mode: " << (cam_mode?"RGB":"GRAY") << std::endl;
	        std::cout << "[" << epuck_name << "] " << "image offset: " << cam_x_offset << ", " << cam_y_offset << std::endl;
	    }

	    epuck_driver::BluetoothConnection epuck_bluetooth_connection(epuck_name, epuck_address.c_str());

	    epuck_driver::BasicCppDriver epuck_cpp_driver(
	    		epuck_name,
	    		epuck_bluetooth_connection,
				enabled_sensors,
				cam_width,
				cam_height,
				cam_zoom,
				cam_mode,
				cam_x_offset,
				cam_y_offset);

	    epuck_driver::RosWrapper epuck(
	    		epuck_name,
	    		epuck_cpp_driver,
				init_xpos,
				init_ypos,
				init_theta);

	    epuck.run();

//		C++ implementation (no ros)
//	    BasicCppDriver epuck(config);
//	    epuck.setMotorSpeed(200, 0);
//
//	    int i = 0;
//	    while(true)
//	    {
//	    	epuck.updateSensorValues();
//	    	ros::Duration(0.2).sleep();
//	    	epuck.setLed(i, 2);
//	    	i = (i+1) % 10;
//	    	epuck.setMotorPosition(i*100, i*100);
//	    //		epuck.setMotorSpeed(i*100, i*-100);
//	    }

}



//TODO
// - consecutive timeouts drops connection
// - image revice timeout
