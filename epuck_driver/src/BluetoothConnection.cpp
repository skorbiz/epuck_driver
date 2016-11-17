/*
 * BluetoothConnection.cpp
 *
 *  Created on: Oct 13, 2016
 *      Author: josl
 */

#define DEBUG_CONNECTION_INIT 1
#define DEBUG_UPDATE_SENSORS_TIMING 0
#define DEBUG_UPDATE_SENSORS_DATA 0
#define DEBUG_COMMUNICATION_ERROR 1

#include <BluetoothConnection.h>

#include <unistd.h>
#include <sstream>
#include <cerrno>
#include <sys/time.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/rfcomm.h>

namespace epuck_driver {

BluetoothConnection::BluetoothConnection(std::string epuckname, int robotId) :
	sock(-1),
	rfcommSock(-1),
	devId(-1),
	epuckname(epuckname)
//	,robotId(robotId)
//	,address("")
{
	initConnectionWithRobotId(robotId);
}

BluetoothConnection::BluetoothConnection(std::string epuckname, const char* address) :
	sock(-1),
	rfcommSock(-1),
	devId(-1),
	epuckname(epuckname)
//	,robotId(-1)
//	,address(address)
{
	initConnectionWithRobotAddress(address);
}

BluetoothConnection::~BluetoothConnection()
{
	std::stringstream ss;

	if(close(rfcommSock) < 0) {
		ss.str("");
		ss << "[" << epuckname << "] " << "Can't close rfcomm socket";
		if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
	}

}


void BluetoothConnection::clearCommunicationBuffer()
{

	char buffer[64];
	struct timeval timeout;
	fd_set readfds;
	int retval;
	int trials = 0;

	if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "sending enter..." << std::endl;
	buffer[0] = '\r';
	FD_ZERO(&readfds);
	FD_SET(rfcommSock, &readfds);
	write(rfcommSock, buffer, 1);
	while(1) {
		timeout.tv_sec=0; // The timeout need to be set every time because the "select" may modify it.
		timeout.tv_usec=500000;
		retval = select(rfcommSock+1,&readfds,NULL,NULL,&timeout);
		if (retval!=0) {
			int n = read(rfcommSock, buffer, 64);
			if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "read " << n << " bytes" << std::endl;
			if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "content: " << buffer << std::endl;
			memset(buffer, 0x0, 64);
		} else {
			break;
		}
	}

	if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "requesting version..." << std::endl;
	buffer[0] = 'V';
	buffer[1] = '\r';
	FD_ZERO(&readfds);
	FD_SET(rfcommSock, &readfds);
	write(rfcommSock, buffer, 2);
	trials = 0;
	while(1) {
		timeout.tv_sec=0; // The timeout need to be set every time because the "select" may modify it.
		timeout.tv_usec=500000;
		retval = select(rfcommSock+1, &readfds, NULL, NULL, &timeout);
		if (retval!=0) {
			int n = read(rfcommSock, buffer, 64);
			if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "read " << n << " bytes" << std::endl;
			if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "content: " << buffer << std::endl;
			memset(buffer, 0x0, 64);
		} else {
			trials++;
			if(trials >= 1) {
				break;
			}
		}
	}

}

//int BluetoothConnection::initConnection()
//{
//	if(address == "")
//		return initConnectionWithRobotId(robotId);
//	else
//		return initConnectionWithRobotAddress(address);
//}

int BluetoothConnection::initConnectionWithRobotId(int robotId) {

	if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "Connecting via bluetooth with robotID: " << robotId << std::endl;
	std::stringstream ss;

	// open device
	devId = hci_get_route(NULL);
	if (devId < 0) {
		ss.str("");
		ss << "[" << epuckname << "] " << "Error, can't get bluetooth adapter ID";
		if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
		return -1;
	}

	// open socket
	sock = hci_open_dev(devId);
	if (sock < 0) {
		ss.str("");
		ss << "[" << epuckname << "] " << "Error, can't open bluetooth adapter";
		if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
		return -1;
	}

	int trials = 3;     // Try looking for the robot 3 times before giving up.
	while(trials) {
		// query
		if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "Scanning bluetooth:" << std::endl;
		//int length  = 8; /* ~10 seconds */
		int length  = 4; /* ~5 seconds */
		inquiry_info *info = NULL;
		int devicesCount = 0;
		while(1) {
			// device id, query length (last 1.28 * length seconds), max devices, lap ??, returned array, flag
			devicesCount = hci_inquiry(devId, length, 255, NULL, &info, IREQ_CACHE_FLUSH);
			if (devicesCount < 0) {
				ss.str("");
				ss << "[" << epuckname << "] " << "Error, can't query bluetooth";
				if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
				if(errno!=EBUSY) {      // EBUSY means the Bluetooth device is currently used by another process (this happens when
					close(sock);        // we want to connect to multiple robots simultaneously); in this case wait a little and retry.
					return -1;          // All others errors are treated normally.
				} else {
					usleep(1000000);
				}
			} else {
				break;
			}
		}

		bool found = false;
		for (int i = 0; i < devicesCount; i++) {
			char addrString[19];
			char addrFriendlyName[256];
			ba2str(&(info+i)->bdaddr, addrString);
			if (hci_read_remote_name(sock, &(info+i)->bdaddr, 256, addrFriendlyName, 0) < 0) {
				strcpy(addrFriendlyName, "[unknown]");
			}
			if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "\t" <<  addrString << " " << addrFriendlyName << std::endl;
			if (strncmp("e-puck_", addrFriendlyName, 7) == 0) {
				int id;
				if (sscanf(addrFriendlyName + 7, "%d", &id) && (id == robotId)) {
					if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "Contacting e-puck " << id << std::endl;

					// set the connection parameters (who to connect to)
					struct sockaddr_rc addr;
					addr.rc_family = AF_BLUETOOTH;
					addr.rc_channel = (uint8_t) 1;
					addr.rc_bdaddr = (info+i)->bdaddr;

					// allocate a socket
					rfcommSock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

					// connect to server
					int status = ::connect(rfcommSock, (struct sockaddr *)&addr, sizeof(addr));

					if (status == 0) {
						clearCommunicationBuffer();
						found = true;
					} else {
						ss.str("");
						ss << "[" << epuckname << "] " << "Error, can't connect to rfcomm socket";
						if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
						return -1;
					}
					break;
				}
			}
		}
		if(found) {
			if(hci_close_dev(sock) < 0) {
				ss.str("");
				ss << "[" << epuckname << "] " << "Can't close HCI device";
				if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
			}
			return 0;
		} else {
			trials--;
		}
	}
	if(hci_close_dev(sock) < 0) {
		ss.str("");
		ss << "[" << epuckname << "] " << "Can't close HCI device";
		if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
	}
	return -1;

}

int BluetoothConnection::initConnectionWithRobotAddress(const char *address)
{
	if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "Connecting via bluetooth with address: " << address << std::endl;

	std::stringstream ss;
	struct sockaddr_rc addr;    // set the connection parameters (who to connect to)
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 1;
	str2ba(address, &addr.rc_bdaddr);

	// allocate a socket
	rfcommSock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	// connect to server
	int status = ::connect(rfcommSock, (struct sockaddr *)&addr, sizeof(addr));

	if (status == 0) {
		clearCommunicationBuffer();
		return 0;
	} else {
		ss.str("");
		ss << "[" << epuckname << "] " << "Error, can't connect to rfcomm socket";
		if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
		return -1;
	}

}

void BluetoothConnection::writeToConnection(char * msg, unsigned int length)
{
    write(rfcommSock, msg, length);
}

unsigned int BluetoothConnection::readFromConnection(char * msg, unsigned int length, char * ret_msg, unsigned int ret_msg_size)
{
    struct timeval timeout;
    struct timeval currentTime2, lastTime2;
    struct timeval currentTime3, lastTime3;
    int consecutiveReadTimeout = 0;
    fd_set readfds;
    int retval;
    int trials = 0;
    unsigned int bufIndex = 0;
    unsigned int bytesRead = 0;

    memset(ret_msg, 0x0, ret_msg_size);
    FD_ZERO(&readfds);
    FD_SET(rfcommSock, &readfds);
    write(rfcommSock, msg, 2);
    bytesRead = 0;

    if(DEBUG_UPDATE_SENSORS_TIMING)gettimeofday(&lastTime3, NULL);
    while(bytesRead < ret_msg_size)
    {
    	timeout.tv_sec=READ_TIMEOUT_SEC; // The timeout need to be set every time because the "select" may modify it.
    	timeout.tv_usec=READ_TIMEOUT_USEC;
    	if(DEBUG_UPDATE_SENSORS_TIMING)gettimeofday(&lastTime2, NULL);
    	retval = select(rfcommSock+1, &readfds, NULL, NULL, &timeout);
    	if(DEBUG_UPDATE_SENSORS_TIMING)gettimeofday(&currentTime2, NULL);
    	if(DEBUG_UPDATE_SENSORS_TIMING)std::cout << "[" << epuckname << "] " << "sensors data read in " << double((currentTime2.tv_sec*1000000 + currentTime2.tv_usec)-(lastTime2.tv_sec*1000000 + lastTime2.tv_usec))/1000000.0 << " sec" << std::endl;
    	if (retval>0)
    	{
    		int n = read(rfcommSock, &ret_msg[bytesRead], ret_msg_size-bytesRead);
    		//std::cout << "read " << n << " / " << bytesToReceive << " bytes" << std::endl;
    		bytesRead += n;
    		consecutiveReadTimeout = 0;
    	}
    	else if(retval==0)
    	{
    		if(DEBUG_COMMUNICATION_ERROR)std::cout << "[" << epuckname << "] " << "sensors read timeout" << std::endl;
    		consecutiveReadTimeout++;
    		break;
    	}
    	else
    	{
    		if(DEBUG_COMMUNICATION_ERROR)perror("sensors read error");
    		break;
    	}
    }
//	if(DEBUG_COMMUNICATION_ERROR)std::cout << "[" << epuckname << "] " << "read " << bytesRead << " bytes" << std::endl;
//	if(DEBUG_COMMUNICATION_ERROR)std::cout << "[" << epuckname << "] " << "Should read: " << ret_msg_size << " bytes" << std::endl;
//	if(DEBUG_COMMUNICATION_ERROR)std::cout << "[" << epuckname << "] " << "content send: " << msg << std::endl;
//	if(DEBUG_COMMUNICATION_ERROR)std::cout << "[" << epuckname << "] " << "content read: " << ret_msg << std::endl;
//	if(DEBUG_COMMUNICATION_ERROR)std::cout << "[" << epuckname << "] " << "content send size: " << length << std::endl;

    return bytesRead;
}

} /* namespace epuck_driver */

