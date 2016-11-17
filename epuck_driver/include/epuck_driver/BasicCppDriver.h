/*
 * BasicCppDriver.h
 *
 *  Created on: Oct 13, 2016
 *      Author: josl
 */

#ifndef EPUCK_DRIVER_MY_SRC_BASICCPPDRIVER_H_
#define EPUCK_DRIVER_MY_SRC_BASICCPPDRIVER_H_

#define DEBUG_CAMERA_INIT 0
#define DEBUG_UPDATE_SENSORS_DATA 0

#define CAM_MAX_BUFFER_SIZE 3203
#define CAM_MODE_GRAY 0
#define CAM_MODE_RGB 1

#define SENSORS_NUM 7
#define ACCELEROMETER 0
#define MOTOR_SPEED 1
#define FLOOR 2
#define PROXIMITY 3
#define MOTOR_POSITION 4
#define MICROPHONE 5
#define CAMERA 6

#define ACTUATORS_NUM 3
#define MOTORS 0
#define LEDS 1
#define MOTORS_POS 2


#include "BluetoothConnection.h"

namespace epuck_driver {

class BasicCppDriver {
public:
	BasicCppDriver(std::string epuck_name,
			BluetoothConnection & epuck_bt_connection,
			bool enabled_sensors[SENSORS_NUM],
			int cam_width,
			int cam_height,
			int cam_zoom,
			int cam_mode,
			int cam_x_offset = -1,
			int cam_y_offset = -1);
	virtual ~BasicCppDriver();

	void setMotorSpeed(int speedLeft, int speedRight);
	void setLed(unsigned char ledNum, unsigned char ledState);
	void setMotorPosition(int stepsLeft, int stepsRight);

	void updateCameraImage();
	void updateSelector();
	void updateSensorValues();

private:
	void initSensors();
	void initCamera();


public:
	int accData[3];
	int motorSpeedData[2];
	int floorData[5];
	int proxData[8];
	int motorPositionData[2];
	int micData[3];
	unsigned char *camData;
	int imageSize;
	int selector;


	std::string epuck_name;
	bool enabled_sensors[SENSORS_NUM];

	int cam_width;
	int cam_height;
	int cam_zoom;
	int cam_mode;
	int cam_x_offset;
	int cam_y_offset;

private:
	BluetoothConnection & _epuck_bt_connnection;

	char _pcToRobotBuff[10];
	unsigned int _bytesToSend;
	unsigned int _bytesToReceive;




};

} /* namespace epuck_driver */

#endif /* EPUCK_DRIVER_MY_SRC_BASICCPPDRIVER_H_ */
