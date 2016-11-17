/*
 * RosWrapper.h
 *
 *  Created on: Nov 3, 2016
 *      Author: josl
 */

#ifndef EPUCK_DRIVER_MY_SRC_ROSWRAPPER_H_
#define EPUCK_DRIVER_MY_SRC_ROSWRAPPER_H_

#include <sstream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt32MultiArray.h>

#include "BasicCppDriver.h"

#define WHEEL_DIAMETER 4        // cm.
#define WHEEL_SEPARATION 5.3    // Separation between wheels (cm).
#define WHEEL_DISTANCE 0.053    // Distance between wheels in meters (axis length); it's the same value as "WHEEL_SEPARATION" but expressed in meters.
#define WHEEL_CIRCUMFERENCE ((WHEEL_DIAMETER*M_PI)/100.0)    // Wheel circumference (meters).
#define MOT_STEP_DIST (WHEEL_CIRCUMFERENCE/1000.0)      // Distance for each motor step (meters); a complete turn is 1000 steps (0.000125 meters per step (m/steps)).
#define ROBOT_RADIUS 0.035 // meters.

#define DEBUG_ODOMETRY 0
#define DEBUG_ACCELEROMETER 0
#define DEBUG_SPEED_RECEIVED 0

namespace epuck_driver {

class RosWrapper
{

public:
	RosWrapper(std::string epuck_name, BasicCppDriver & epuck_driver, double xPos_init = 0, double yPos_init = 0, double theta_init = 0);
	virtual ~RosWrapper();

	void run();

private:
	void RGB565toRGB888(int width, int height, unsigned char *src, unsigned char *dst);

	void updateSensors();
	void updateProximity();
	void updateMotorPosition();
	void updateAccelerometer();
	void updateMotorSpeed();
	void updateFloor();
	void updateMicrophone();
	void updateCamera();
	void updateSelector();

	void updateActuators();
	void handlerVelocity(const geometry_msgs::Twist::ConstPtr& msg);
	void handleLeds(const std_msgs::UInt32MultiArray::ConstPtr& msg);

	template<class T>
	std::string t_to_string(T i)
	{
	    std::stringstream ss;
	    std::string s;
	    ss << i;
	    s = ss.str();

	    return s;
	}

private:
	BasicCppDriver & epuck;
	std::string epuck_name;

	tf::TransformBroadcaster br;
	ros::NodeHandle n;

	ros::Subscriber cmdVelSubscriber;
	ros::Subscriber ledsSubscriber;

	ros::Publisher proxPublisher[8];
	ros::Publisher laserPublisher;
	ros::Publisher odomPublisher;
	ros::Publisher imagePublisher;
	ros::Publisher accelPublisher;
	ros::Publisher motorSpeedPublisher;
	ros::Publisher microphonePublisher;
	ros::Publisher floorPublisher;

	bool changedActuators[ACTUATORS_NUM];
	int speedLeft;
	int speedRight;

    std::vector<uint32_t> ledCurrent;
    std::vector<uint32_t> ledNext;

	double theta;
	double xPos;
	double yPos;

	double leftStepsDiff;
	double rightStepsDiff;
	double leftStepsPrev;
	double rightStepsPrev;
	signed long int leftStepsRawPrev;
	signed long int rightStepsRawPrev;
	ros::Time lastTime;
	int overflowCountLeft;
	int overflowCountRight;




};

} /* namespace epuck_driver */


#endif /* EPUCK_DRIVER_MY_SRC_ROSWRAPPER_H_ */
