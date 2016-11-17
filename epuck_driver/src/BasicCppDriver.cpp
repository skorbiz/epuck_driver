/*
 * BasicCppDriver.cpp
 *
 *  Created on: Oct 13, 2016
 *      Author: josl
 */

#include <BasicCppDriver.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace epuck_driver {

BasicCppDriver::BasicCppDriver(
		std::string epuck_name,
		BluetoothConnection & epuck_bt_connection,
		bool enabled_sensors[SENSORS_NUM],
		int cam_width,
		int cam_height,
		int cam_zoom,
		int cam_mode,
		int cam_x_offset,
		int cam_y_offset) :
	_epuck_bt_connnection(epuck_bt_connection),
	cam_width(cam_width),
	cam_height(cam_height),
	cam_zoom(cam_zoom),
	cam_mode(cam_mode),
	cam_x_offset(cam_x_offset),
	cam_y_offset(cam_y_offset)
{
	for(int i = 0; i < SENSORS_NUM; i++)
		this->enabled_sensors[i] = enabled_sensors[i];

	for(int i = 0; i < 3; i++)
		accData[i] = 0;

	motorSpeedData[0] = 0;
	motorSpeedData[2] = 0;

	for(int i = 0; i < 5; i++)
		floorData[i] = 0;

	for(int i = 0; i < 8; i++)
		proxData[i] = 0;

	motorPositionData[0] = 0;
	motorPositionData[1] = 0;

	for(int i = 0; i < 3; i++)
		micData[i] = 0;

	selector = 0;



	unsigned int _bytesToSend = 0;
	unsigned int _bytesToReceive = 0;

	//epuck_bt_connection.initConnection();
	initSensors();
	initCamera();
}

BasicCppDriver::~BasicCppDriver()
{
	//_epuck_bt_connnection.closeConnection();
	if(enabled_sensors[CAMERA])
		free(camData);

}

void BasicCppDriver::setMotorSpeed(int speedLeft, int speedRight)
{
    char buff[6];
    buff[0] = -'D';
    buff[1] = speedLeft&0xFF;
    buff[2] = (speedLeft>>8)&0xFF;
    buff[3] = speedRight&0xFF;
    buff[4] = (speedRight>>8)&0xFF;
    buff[5] = 0;
    _epuck_bt_connnection.writeToConnection(buff, 6);

}

void BasicCppDriver::setLed(unsigned char ledNum, unsigned char ledState)
{
    char buff[6];
    buff[0] = -'L';
    buff[1] = ledNum;
    buff[2] = ledState;
    buff[3] = 0;
    _epuck_bt_connnection.writeToConnection(buff, 6);
}

void BasicCppDriver::setMotorPosition(int stepsLeft, int stepsRight)
{
    char buff[6];
    buff[0] = -'P';
    buff[1] = stepsLeft&0xFF;
    buff[2] = (stepsLeft>>8)&0xFF;
    buff[3] = stepsRight&0xFF;
    buff[4] = (stepsRight>>8)&0xFF;
    buff[5] = 0;
    _epuck_bt_connnection.writeToConnection(buff, 6);
}

void BasicCppDriver::updateSensorValues()
{
	char robotToPcBuff[_bytesToReceive];
	unsigned int bytesRead = _epuck_bt_connnection.readFromConnection(_pcToRobotBuff,_bytesToSend, robotToPcBuff, _bytesToReceive);
	int bufIndex = 0;

    if(bytesRead == _bytesToReceive) {
        if(enabled_sensors[ACCELEROMETER]) {
            accData[0] = (unsigned char)robotToPcBuff[bufIndex] | robotToPcBuff[bufIndex+1]<<8;
            accData[1] = (unsigned char)robotToPcBuff[bufIndex+2] | robotToPcBuff[bufIndex+3]<<8;
            accData[2] = (unsigned char)robotToPcBuff[bufIndex+4] | robotToPcBuff[bufIndex+5]<<8;
            bufIndex += 6;
            if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuck_name << "] " << "acc: " << accData[0] << "," << accData[1] << "," << accData[2] << std::endl;
        }
        if(enabled_sensors[MOTOR_SPEED]) {
            motorSpeedData[0] = (unsigned char)robotToPcBuff[bufIndex] | robotToPcBuff[bufIndex+1]<<8;
            motorSpeedData[1] = (unsigned char)robotToPcBuff[bufIndex+2] | robotToPcBuff[bufIndex+3]<<8;
            bufIndex += 4;
            if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuck_name << "] " << "speed: " << motorSpeedData[0] << "," << motorSpeedData[1] << std::endl;
        }
        if(enabled_sensors[FLOOR]) {
            floorData[0] = (unsigned char)robotToPcBuff[bufIndex] | robotToPcBuff[bufIndex+1]<<8;
            floorData[1] = (unsigned char)robotToPcBuff[bufIndex+2] | robotToPcBuff[bufIndex+3]<<8;
            floorData[2] = (unsigned char)robotToPcBuff[bufIndex+4] | robotToPcBuff[bufIndex+5]<<8;
            floorData[3] = (unsigned char)robotToPcBuff[bufIndex+6] | robotToPcBuff[bufIndex+7]<<8;
            floorData[4] = (unsigned char)robotToPcBuff[bufIndex+8] | robotToPcBuff[bufIndex+9]<<8;
            bufIndex += 10;
            if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuck_name << "] " << "floor: " << floorData[0] << "," << floorData[1] << "," << floorData[2] << "," << floorData[3] << "," << floorData[4] << std::endl;
        }
        if(enabled_sensors[PROXIMITY]) {
            proxData[0] = (unsigned char)robotToPcBuff[bufIndex] | robotToPcBuff[bufIndex+1]<<8;
            proxData[1] = (unsigned char)robotToPcBuff[bufIndex+2] | robotToPcBuff[bufIndex+3]<<8;
            proxData[2] = (unsigned char)robotToPcBuff[bufIndex+4] | robotToPcBuff[bufIndex+5]<<8;
            proxData[3] = (unsigned char)robotToPcBuff[bufIndex+6] | robotToPcBuff[bufIndex+7]<<8;
            proxData[4] = (unsigned char)robotToPcBuff[bufIndex+8] | robotToPcBuff[bufIndex+9]<<8;
            proxData[5] = (unsigned char)robotToPcBuff[bufIndex+10] | robotToPcBuff[bufIndex+11]<<8;
            proxData[6] = (unsigned char)robotToPcBuff[bufIndex+12] | robotToPcBuff[bufIndex+13]<<8;
            proxData[7] = (unsigned char)robotToPcBuff[bufIndex+14] | robotToPcBuff[bufIndex+15]<<8;
            bufIndex += 16;
            if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuck_name << "] " << "prox: " << proxData[0] << "," << proxData[1] << "," << proxData[2] << "," << proxData[3] << "," << proxData[4] << "," << proxData[5] << "," << proxData[6] << "," << proxData[7] << std::endl;
        }
        if(enabled_sensors[MOTOR_POSITION]) {
            motorPositionData[0] = (unsigned char)robotToPcBuff[bufIndex] | robotToPcBuff[bufIndex+1]<<8;
            motorPositionData[1] = (unsigned char)robotToPcBuff[bufIndex+2] | robotToPcBuff[bufIndex+3]<<8;
            bufIndex += 4;
            if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuck_name << "] " << "position: " << motorPositionData[0] << "," << motorPositionData[1] << std::endl;
        }
        if(enabled_sensors[MICROPHONE]) {
            micData[0] = (unsigned char)robotToPcBuff[bufIndex] | robotToPcBuff[bufIndex+1]<<8;
            micData[1] = (unsigned char)robotToPcBuff[bufIndex+2] | robotToPcBuff[bufIndex+3]<<8;
            micData[2] = (unsigned char)robotToPcBuff[bufIndex+4] | robotToPcBuff[bufIndex+5]<<8;
            bufIndex += 6;
            if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuck_name << "] " << "mic: " << micData[0] << "," << micData[1] << "," << micData[2] << std::endl;
        }
    } else {
        if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuck_name << "] " << "discard the sensors data" << std::endl;
    }
}

void BasicCppDriver::updateCameraImage()
{
    if(enabled_sensors[CAMERA])
    {
		char msg[2];
		int imageBytesToReceive = imageSize;
		msg[0] = -'I';
		msg[1] = 0;

		char robotToPcBuff[imageBytesToReceive];
		unsigned int bytesRead = _epuck_bt_connnection.readFromConnection(msg,2, (char *)camData, imageBytesToReceive);
    }
}

void BasicCppDriver::updateSelector()
{
		char msg[2];
		msg[0] = 'C';
		msg[1] = '\r';
		int bytesToReceive = 5;
		char robotToPcBuff[bytesToReceive];
		unsigned int bytesRead = _epuck_bt_connnection.readFromConnection(msg,2, robotToPcBuff, bytesToReceive);
		selector = robotToPcBuff[2] - '0';
}




void BasicCppDriver::initSensors()
{
	unsigned int bufIndex = 0;
	_bytesToReceive = 0;

	if(enabled_sensors[ACCELEROMETER]){
		_pcToRobotBuff[bufIndex] = -'a';
		bufIndex++;
		_bytesToReceive += 6;
	}

	if(enabled_sensors[MOTOR_SPEED]){
		_pcToRobotBuff[bufIndex] = -'E';
		bufIndex++;
		_bytesToReceive += 4;
	}

	if(enabled_sensors[FLOOR]){
		_pcToRobotBuff[bufIndex] = -'M';
		bufIndex++;
		_bytesToReceive += 10;
	}

	if(enabled_sensors[PROXIMITY]){
		_pcToRobotBuff[bufIndex] = -'N';
		bufIndex++;
		_bytesToReceive += 16;
	}

	if(enabled_sensors[MOTOR_POSITION]){
		_pcToRobotBuff[bufIndex] = -'Q';
		bufIndex++;
		_bytesToReceive += 4;
	}
	if(enabled_sensors[MICROPHONE]) {
		_pcToRobotBuff[bufIndex] = -'u';
		bufIndex++;
		_bytesToReceive += 6;
	}

	_pcToRobotBuff[bufIndex] = 0;        // Terminate the command sequence; the camera image will be handled separately.
	_bytesToSend = bufIndex + 1;

}


void BasicCppDriver::initCamera()
{
	assert(cam_width > 0 && cam_width <= 640);
	assert(cam_height > 0 && cam_height <= 480);
	assert(cam_zoom >= 0 && cam_zoom <= 8);
	assert(cam_mode >= 0 && cam_mode <= 1);

	if(cam_x_offset >= 0 && cam_x_offset <= 640)
        cam_x_offset = (640-cam_width*cam_zoom)/2;  // Center the slice.

	if(cam_y_offset >= 0 && cam_y_offset <= 480);
		cam_y_offset = (480-cam_height*cam_zoom)/2;  // Center the slice.

	assert(cam_width*cam_height*(cam_mode+1) <= CAM_MAX_BUFFER_SIZE && "parameters exceed the maximum buffer");

	if(enabled_sensors[CAMERA])
	{
		imageSize = cam_width*cam_height*(cam_mode+1)+3; // The image data header contains "mode", "width", "height" in the first 3 bytes.
		camData = (unsigned char *) malloc (imageSize);

		// Configure camera params.
		char buff[30];
		char buff_ret[3];
		memset(buff, 0x0, 30);
		sprintf(buff,"J,%d,%d,%d,%d,%d,%d\r", cam_mode, cam_width, cam_height, cam_zoom, cam_x_offset, cam_y_offset);
		unsigned int bytesRecived = _epuck_bt_connnection.readFromConnection(buff, strlen(buff),buff_ret, 3);

		if(bytesRecived == 3)
		{
			if(DEBUG_CAMERA_INIT)std::cout << "camera init correctly (" << buff_ret[0] << buff_ret[1] << buff_ret[2] << ")" << std::endl;
		}
		else
		{
			if(DEBUG_CAMERA_INIT)std::cout  << "cannot init camera" << std::endl;
		}
	}


}

} /* namespace epuck_driver */
