/*
 * BluetoothConnection.h
 *
 *  Created on: Oct 13, 2016
 *      Author: josl
 */

#ifndef EPUCK_DRIVER_MY_SRC_BLUETOOTHCONNECTION_H_
#define EPUCK_DRIVER_MY_SRC_BLUETOOTHCONNECTION_H_

#define READ_TIMEOUT_SEC 10    // 10 seconds, keep it high to avoid desynchronize when there are communication delays due to Bluetooth.
#define READ_TIMEOUT_USEC 0


#include <iostream>

namespace epuck_driver {

class BluetoothConnection
{

public:
	BluetoothConnection(std::string epuckname, int robotId);
	BluetoothConnection(std::string epuckname, const char *address);
	virtual ~BluetoothConnection();

//	int initConnection();
//	void closeConnection();

	void writeToConnection(char * msg, unsigned int length);
	unsigned int readFromConnection(char * msg, unsigned int length, char * ret_msg, unsigned int ret_msg_size);


private:
	void clearCommunicationBuffer();
	int initConnectionWithRobotId(int robotId);
	int initConnectionWithRobotAddress(const char *address);

private:
	int rfcommSock;
	int sock;
	int devId;
	std::string epuckname;
//	int robotId;
//	const char* address;

};

} /* namespace epuck_driver */

#endif /* EPUCK_DRIVER_MY_SRC_BLUETOOTHCONNECTION_H_ */
