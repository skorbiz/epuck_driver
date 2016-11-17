/*
 * RosWrapper.cpp
 *
 *  Created on: Nov 3, 2016
 *      Author: josl
 */

#include <RosWrapper.h>

#include <string.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>

namespace epuck_driver {

RosWrapper::RosWrapper(std::string epuck_name, BasicCppDriver & epuck_driver, double xPos_init, double yPos_init, double theta_init) :
	epuck_name(epuck_name),
	epuck(epuck_driver),
	ledCurrent(10,0),
	ledNext(10,0),
	speedLeft(0),
	speedRight(0),
	theta(theta_init),
	xPos(xPos_init),
	yPos(yPos_init)
{
    if(epuck.enabled_sensors[PROXIMITY])
        for(int i = 0; i < 8; i++)
            proxPublisher[i] = n.advertise<sensor_msgs::Range>("proximity" + t_to_string(i), 10);
    if(epuck.enabled_sensors[ACCELEROMETER])
        accelPublisher = n.advertise<sensor_msgs::Imu>("accel", 10);
    if(epuck.enabled_sensors[MOTOR_SPEED])
    	motorSpeedPublisher = n.advertise<visualization_msgs::Marker>("motor_speed", 10);
    if(epuck.enabled_sensors[FLOOR])
        floorPublisher = n.advertise<visualization_msgs::Marker>("floor", 10);
    if(epuck.enabled_sensors[PROXIMITY])
        laserPublisher = n.advertise<sensor_msgs::LaserScan>("scan", 10);
    if(epuck.enabled_sensors[MOTOR_POSITION])
        odomPublisher = n.advertise<nav_msgs::Odometry>("odom", 10);
    if(epuck.enabled_sensors[MICROPHONE])
        microphonePublisher = n.advertise<visualization_msgs::Marker>("microphone", 10);
    if(epuck.enabled_sensors[CAMERA])
		imagePublisher = n.advertise<sensor_msgs::Image>("camera", 1);

    cmdVelSubscriber = n.subscribe("mobile_base/cmd_vel", 10, &RosWrapper::handlerVelocity, this);
    ledsSubscriber = n.subscribe("leds", 10, &RosWrapper::handleLeds, this);

    for(int i = 0; i < ACTUATORS_NUM; i++)
    	changedActuators[i] = false;

	leftStepsDiff = 0;
	rightStepsDiff = 0;
	leftStepsPrev = 0;
	rightStepsPrev = 0;
	leftStepsRawPrev = 0;
	rightStepsRawPrev = 0;
	ros::Time lastTime = ros::Time::now();
	overflowCountLeft = 0;
	overflowCountRight = 0;

}

RosWrapper::~RosWrapper()
{
}

void RosWrapper::RGB565toRGB888(int width, int height, unsigned char *src, unsigned char *dst)
{
	int index_src = 0;
	int index_dst = 0;

	for (int line = 0; line < height; ++line)
	{
		for (int column = 0; column < width; ++column)
		{
			dst[index_dst++] = (unsigned char)(src[index_src] & 0xF8);
			dst[index_dst++] = (unsigned char)((src[index_src]&0x07)<<5) | (unsigned char)((src[index_src+1]&0xE0)>>3);
			dst[index_dst++] = (unsigned char)((src[index_src+1]&0x1F)<<3);
			index_src+=2;
		}
	}
}

void RosWrapper::updateSensors()
{
	epuck.updateSensorValues();
	epuck.updateCameraImage();
	epuck.updateSelector();
	updateProximity();
	updateMotorPosition();
	updateAccelerometer();
	updateMotorSpeed();
	updateFloor();
	updateMicrophone();
	updateCamera();
}

void RosWrapper::updateProximity()
{

    if(epuck.enabled_sensors[PROXIMITY]) {
        for(int i = 0; i < 8; i++)
        {
        	sensor_msgs::Range proxMsg;

            std::stringstream ss;
            ss.str("");
            ss << "proximity" << i;
            proxPublisher[i] = n.advertise<sensor_msgs::Range>(ss.str(), 10);
            //proxMsg[i] = new sensor_msgs::Range();
            proxMsg.radiation_type = sensor_msgs::Range::INFRARED;
            ss.str("");
            ss << epuck_name << "/base_prox" << i;
            proxMsg.header.frame_id =  ss.str();
            proxMsg.field_of_view = 0.26;    // About 15 degrees...to be checked!
            proxMsg.min_range = 0.005;       // 0.5 cm.
            proxMsg.max_range = 0.05;        // 5 cm.

            proxMsg.range = 0.5/sqrt(epuck.proxData[i]);
            // Transform the analog value to a distance value in meters (given from field tests).

            if(proxMsg.range > proxMsg.max_range)
                proxMsg.range = proxMsg.max_range;
            if(proxMsg.range < proxMsg.min_range)
                proxMsg.range = proxMsg.min_range;
            proxMsg.header.stamp = ros::Time::now();

            proxPublisher[i].publish(proxMsg);
        }

        // e-puck proximity positions (cm), x pointing forward, y pointing left
        //           P7(3.5, 1.0)   P0(3.5, -1.0)
        //       P6(2.5, 2.5)           P1(2.5, -2.5)
        //   P5(0.0, 3.0)                   P2(0.0, -3.0)
        //       P4(-3.5, 2.0)          P3(-3.5, -2.0)
        //
        // e-puck proximity orentations (degrees)
        //           P7(10)   P0(350)
        //       P6(40)           P1(320)
        //   P5(90)                   P2(270)
        //       P4(160)          P3(200)

        tf::Vector3 proxy_position [] = {
        		tf::Vector3( 0.035, -0.010, 0.034),
				tf::Vector3( 0.025, -0.025, 0.034),
				tf::Vector3( 0.000, -0.030, 0.034),
				tf::Vector3(-0.035, -0.020, 0.034),
				tf::Vector3(-0.035,  0.020, 0.034),
				tf::Vector3( 0.000,  0.030, 0.034),
				tf::Vector3( 0.025,  0.025, 0.034),
				tf::Vector3( 0.035,  0.010, 0.034)};

        double proxy_orientation_degree [] = {350, 320, 270, 200, 160, 90, 40, 10};

        for(int i = 0; i < 8; i++)
        {
            std::stringstream parent;
            std::stringstream child;
            tf::Transform transform;
            tf::Quaternion q;

            transform.setOrigin( proxy_position[i] );
            q.setRPY(0, 0, (proxy_orientation_degree[i] / 180.0) * M_PI );
            transform.setRotation(q);
            parent << epuck_name << "/base_prox" << i;
            child << epuck_name << "/base_link";
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
        }



        //populate the LaserScan message
        sensor_msgs::LaserScan laserMsg;
        laserMsg.header.stamp = ros::Time::now();
        laserMsg.header.frame_id = epuck_name + "/base_laser";
        laserMsg.angle_min = -M_PI/2.0;
        laserMsg.angle_max = M_PI/2.0;
        laserMsg.angle_increment = M_PI/18.0; // 10 degrees.
        //laserMsg.time_increment = (currentTimeMap-lastTimeMap).toSec()/180; //0.003; //(1 / laser_frequency) / (num_readings);
        //laserMsg.scan_time = (currentTimeMap-lastTimeMap).toSec();
        // The laser is placed in the center of the robot, but the proximity sensors are placed around the robot thus add "ROBOT_RADIUS" to get correct values.
        laserMsg.range_min = 0.005+ROBOT_RADIUS; // 0.5 cm + ROBOT_RADIUS.
        laserMsg.range_max = 0.05+ROBOT_RADIUS; // 5 cm + ROBOT_RADIUS.
        laserMsg.ranges.resize(19);
        laserMsg.intensities.resize(19);

        // We use the information from the 6 proximity sensors on the front side of the robot to get 19 laser scan points. The interpolation used is the following:
        // -90 degrees: P2
        // -80 degrees: 4/5*P2 + 1/5*P1
        // -70 degrees: 3/5*P2 + 2/5*P1
        // -60 degrees: 2/5*P2 + 3/5*P1
        // -50 degrees: 1/5*P2 + 4/5*P1
        // -40 degrees: P1
        // -30 degrees: 2/3*P1 + 1/3*P0
        // -20 degrees: 1/3*P1 + 2/3*P0
        // -10 degrees: P0
        // 0 degrees: 1/2*P0 + 1/2*P7
        // 10 degrees: P7
        // 20 degrees: 1/3*P6 + 2/3*P7
        // 30 degrees: 2/3*P6 + 1/3*P7
        // 40 degrees: P6
        // 50 degrees: 1/5*P5 + 4/5*P6
        // 60 degrees: 2/5*P5 + 3/5*P6
        // 70 degrees: 3/5*P5 + 2/5*P6
        // 80 degrees: 4/5*P5 + 1/5*P6
        // 90 degrees: P5

        double tempProx[19];
        tempProx[0] = epuck.proxData[2];
        tempProx[1] = epuck.proxData[2]*4/5 + epuck.proxData[1]*1/5;
        tempProx[2] = epuck.proxData[2]*3/5 + epuck.proxData[1]*2/5;
        tempProx[3] = epuck.proxData[2]*2/5 + epuck.proxData[1]*3/5;
        tempProx[4] = epuck.proxData[2]*1/5 + epuck.proxData[1]*4/5;
        tempProx[5] = epuck.proxData[1];
        tempProx[6] = epuck.proxData[1]*2/3 + epuck.proxData[0]*1/3;
        tempProx[7] = epuck.proxData[1]*1/3 + epuck.proxData[0]*2/3;
        tempProx[8] = epuck.proxData[0];
        tempProx[9] = epuck.proxData[0]*1/2 + epuck.proxData[7]*1/2;
        tempProx[10] = epuck.proxData[7];
        tempProx[11] = epuck.proxData[7]*2/3 + epuck.proxData[6]*1/3;
        tempProx[12] = epuck.proxData[7]*1/3 + epuck.proxData[6]*2/3;
        tempProx[13] = epuck.proxData[6];
        tempProx[14] = epuck.proxData[6]*4/5 + epuck.proxData[5]*1/5;
        tempProx[15] = epuck.proxData[6]*3/5 + epuck.proxData[5]*2/5;
        tempProx[16] = epuck.proxData[6]*2/5 + epuck.proxData[5]*3/5;
        tempProx[17] = epuck.proxData[6]*1/5 + epuck.proxData[5]*4/5;
        tempProx[18] = epuck.proxData[5];

        for(int i = 0; i < 19; i++)
        {
            if(tempProx > 0)
            { // Transform the analog value to a distance value in meters (given from field tests).
                laserMsg.ranges[i] = (0.5/sqrt(tempProx[i]))+ROBOT_RADIUS;
                laserMsg.intensities[i] = tempProx[i];
            }
            else
            { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
                laserMsg.ranges[i] = laserMsg.range_max;
                laserMsg.intensities[i] = 0;
            }

            if(laserMsg.ranges[i] > laserMsg.range_max)
                laserMsg.ranges[i] = laserMsg.range_max;
            if(laserMsg.ranges[i] < laserMsg.range_min)
                laserMsg.ranges[i] = laserMsg.range_min;
        }

        std::stringstream parent;
        std::stringstream child;
        tf::Transform transform;
        tf::Quaternion q;

        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.034) );
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        parent.str("");
        child.str("");
        parent << epuck_name << "/base_laser";
        child << epuck_name << "/base_link";
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));

        laserPublisher.publish(laserMsg);

    }
}

void RosWrapper::updateMotorPosition()
{
    if(epuck.enabled_sensors[MOTOR_POSITION]) {
    	signed long int motorPositionDataCorrect[2];
    	double deltaSteps = 0;
    	double deltaTheta = 0;


        // The encoders values coming from the e-puck are 2 bytes signed int thus we need to handle the overflows otherwise the odometry will be wrong after a while (about 4 meters).
        if((leftStepsRawPrev>0) && (epuck.motorPositionData[0]<0) && (abs(epuck.motorPositionData[0]-leftStepsRawPrev)>30000)) {     // Overflow detected (positive).
            overflowCountLeft++;
        }
        if((leftStepsRawPrev<0) && (epuck.motorPositionData[0]>0) && (abs(epuck.motorPositionData[0]-leftStepsRawPrev)>30000)) {     // Overflow detected (negative).
            overflowCountLeft--;
        }
        motorPositionDataCorrect[0] = (overflowCountLeft*65536) + epuck.motorPositionData[0];

        if((rightStepsRawPrev>0) && (epuck.motorPositionData[1]<0) && (abs(epuck.motorPositionData[1]-rightStepsRawPrev)>30000)) {     // Overflow detected (positive).
            overflowCountRight++;
        }
        if((rightStepsRawPrev<0) && (epuck.motorPositionData[1]>0) && (abs(epuck.motorPositionData[1]-rightStepsRawPrev)>30000)) {     // Overflow detected (negative).
            overflowCountRight--;
        }
        motorPositionDataCorrect[1] = (overflowCountRight*65536) + epuck.motorPositionData[1];

        leftStepsRawPrev = epuck.motorPositionData[0];
        rightStepsRawPrev = epuck.motorPositionData[1];

        if(DEBUG_ODOMETRY)std::cout << "[" << epuck_name << "] " << "left, right raw: " << epuck.motorPositionData[0] << ", " << epuck.motorPositionData[1] << std::endl;
        if(DEBUG_ODOMETRY)std::cout << "[" << epuck_name << "] " << "left, right raw corrected: " << motorPositionDataCorrect[0] << ", " << motorPositionDataCorrect[1] << std::endl;

        // Compute odometry.
        leftStepsDiff = motorPositionDataCorrect[0]*MOT_STEP_DIST - leftStepsPrev; // Expressed in meters.
        rightStepsDiff = motorPositionDataCorrect[1]*MOT_STEP_DIST - rightStepsPrev;   // Expressed in meters.
        if(DEBUG_ODOMETRY)std::cout << "[" << epuck_name << "] " << "left, right steps diff: " << leftStepsDiff << ", " << rightStepsDiff << std::endl;

        deltaTheta = (rightStepsDiff - leftStepsDiff)/WHEEL_DISTANCE;   // Expressed in radiant.
        deltaSteps = (rightStepsDiff + leftStepsDiff)/2;        // Expressed in meters.
        if(DEBUG_ODOMETRY)std::cout << "[" << epuck_name << "] " << "delta theta, steps: " << deltaTheta << ", " << deltaSteps << std::endl;

        xPos += deltaSteps*cos(theta + deltaTheta/2);   // Expressed in meters.
        yPos += deltaSteps*sin(theta + deltaTheta/2);   // Expressed in meters.
        theta += deltaTheta;    // Expressed in radiant.
        if(DEBUG_ODOMETRY)std::cout << "[" << epuck_name << "] " << "x, y, theta: " << xPos << ", " << yPos << ", " << theta << std::endl;

        leftStepsPrev = motorPositionDataCorrect[0]*MOT_STEP_DIST;     // Expressed in meters.
        rightStepsPrev = motorPositionDataCorrect[1]*MOT_STEP_DIST;    // Expressed in meters.

        // Publish the odometry message over ROS.
        nav_msgs::Odometry odomMsg;
        odomMsg.header.stamp = ros::Time::now();
        odomMsg.header.frame_id = "odom";
        std::stringstream ss;
        ss << epuck_name << "/base_link";
        odomMsg.child_frame_id = ss.str();
        odomMsg.pose.pose.position.x = xPos;
        odomMsg.pose.pose.position.y = yPos;
        odomMsg.pose.pose.position.z = 0;
        // Since all odometry is 6DOF we'll need a quaternion created from yaw.
        geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(theta);
        odomMsg.pose.pose.orientation = odomQuat;
        ros::Time currentTime = ros::Time::now();
        odomMsg.twist.twist.linear.x = deltaSteps / ((currentTime-lastTime).toSec());   // "deltaSteps" is the linear distance covered in meters from the last update (delta distance);
                                                                                        // the time from the last update is measured in seconds thus to get m/s we multiply them.
        odomMsg.twist.twist.angular.z = deltaTheta / ((currentTime-lastTime).toSec());  // "deltaTheta" is the angular distance covered in radiant from the last update (delta angle);
                                                                                        // the time from the last update is measured in seconds thus to get rad/s we multiply them.
        if(DEBUG_ODOMETRY)std::cout << "[" << epuck_name << "] " << "time elapsed = " << (currentTime-lastTime).toSec() << " seconds" << std::endl;
        lastTime = ros::Time::now();

        odomPublisher.publish(odomMsg);

        // Publish the transform over tf.
        geometry_msgs::TransformStamped odomTrans;
        odomTrans.header.stamp = odomMsg.header.stamp;
        odomTrans.header.frame_id = odomMsg.header.frame_id;
        odomTrans.child_frame_id = odomMsg.child_frame_id;
        odomTrans.transform.translation.x = xPos;
        odomTrans.transform.translation.y = yPos;
        odomTrans.transform.translation.z = 0.0;
        odomTrans.transform.rotation = odomQuat;
        br.sendTransform(odomTrans);
    }
}

void RosWrapper::updateAccelerometer()
{
    if(epuck.enabled_sensors[ACCELEROMETER]) {
        std::stringstream ss;
        ss << epuck_name << "/base_link";
        sensor_msgs::Imu accelMsg;
        accelMsg.header.frame_id = ss.str();
        accelMsg.header.stamp = ros::Time::now();
        accelMsg.linear_acceleration.x = (epuck.accData[1]-2048.0)/800.0*9.81; // 1 g = about 800, then transforms in m/s^2.
        accelMsg.linear_acceleration.y = (epuck.accData[0]-2048.0)/800.0*9.81;
        accelMsg.linear_acceleration.z = (epuck.accData[2]-2048.0)/800.0*9.81;
        accelMsg.linear_acceleration_covariance[0] = 0.01;
        accelMsg.linear_acceleration_covariance[1] = 0.0;
        accelMsg.linear_acceleration_covariance[2] = 0.0;
        accelMsg.linear_acceleration_covariance[3] = 0.0;
        accelMsg.linear_acceleration_covariance[4] = 0.01;
        accelMsg.linear_acceleration_covariance[5] = 0.0;
        accelMsg.linear_acceleration_covariance[6] = 0.0;
        accelMsg.linear_acceleration_covariance[7] = 0.0;
        accelMsg.linear_acceleration_covariance[8] = 0.01;
        if(DEBUG_ACCELEROMETER)std::cout << "[" << epuck_name << "] " << "accel raw: " << epuck.accData[0] << ", " << epuck.accData[1] << ", " << epuck.accData[2] << std::endl;
        if(DEBUG_ACCELEROMETER)std::cout << "[" << epuck_name << "] " << "accel (m/s2): " << ((epuck.accData[0]-2048.0)/800.0*9.81) << ", " << ((epuck.accData[1]-2048.0)/800.0*9.81) << ", " << ((epuck.accData[2]-2048.0)/800.0*9.81) << std::endl;
        accelMsg.angular_velocity.x = 0;
        accelMsg.angular_velocity.y = 0;
        accelMsg.angular_velocity.z = 0;
        accelMsg.angular_velocity_covariance[0] = 0.01;
        accelMsg.angular_velocity_covariance[1] = 0.0;
        accelMsg.angular_velocity_covariance[2] = 0.0;
        accelMsg.angular_velocity_covariance[3] = 0.0;
        accelMsg.angular_velocity_covariance[4] = 0.01;
        accelMsg.angular_velocity_covariance[5] = 0.0;
        accelMsg.angular_velocity_covariance[6] = 0.0;
        accelMsg.angular_velocity_covariance[7] = 0.0;
        accelMsg.angular_velocity_covariance[8] = 0.01;
        geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(0);
        accelMsg.orientation = odomQuat;
        accelMsg.orientation_covariance[0] = 0.01;
        accelMsg.orientation_covariance[1] = 0.0;
        accelMsg.orientation_covariance[2] = 0.0;
        accelMsg.orientation_covariance[3] = 0.0;
        accelMsg.orientation_covariance[4] = 0.01;
        accelMsg.orientation_covariance[5] = 0.0;
        accelMsg.orientation_covariance[6] = 0.0;
        accelMsg.orientation_covariance[7] = 0.0;
        accelMsg.orientation_covariance[8] = 0.01;
        accelPublisher.publish(accelMsg);
    }
}

void RosWrapper::updateMotorSpeed()
{
	if(epuck.enabled_sensors[MOTOR_SPEED]) {
		std::stringstream ss;
		ss << epuck_name << "/base_link";
		visualization_msgs::Marker motorSpeedMsg;
		motorSpeedMsg.header.frame_id = ss.str();
		motorSpeedMsg.header.stamp = ros::Time::now();
		motorSpeedMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		motorSpeedMsg.pose.position.x = 0.15;
		motorSpeedMsg.pose.position.y = 0;
		motorSpeedMsg.pose.position.z = 0.15;
		geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(0);
		motorSpeedMsg.pose.orientation = odomQuat;
		motorSpeedMsg.scale.z = 0.01;
		motorSpeedMsg.color.a = 1.0;
		motorSpeedMsg.color.r = 1.0;
		motorSpeedMsg.color.g = 1.0;
		motorSpeedMsg.color.b = 1.0;
		ss.str("");
		ss << "speed: [" << epuck.motorSpeedData[0] << ", " << epuck.motorSpeedData[1] << "]";
		motorSpeedMsg.text = ss.str();
		motorSpeedPublisher.publish(motorSpeedMsg);
	}
}



void RosWrapper::updateFloor()
{
	if(epuck.enabled_sensors[FLOOR]) {
		std::stringstream ss;
		ss << epuck_name << "/base_link";
		visualization_msgs::Marker floorMsg;
		floorMsg.header.frame_id = ss.str();
		floorMsg.header.stamp = ros::Time::now();
		floorMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		floorMsg.pose.position.x = 0.15;
		floorMsg.pose.position.y = 0;
		floorMsg.pose.position.z = 0.13;
		geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(0);
		floorMsg.pose.orientation = odomQuat;
		floorMsg.scale.z = 0.01;
		floorMsg.color.a = 1.0;
		floorMsg.color.r = 1.0;
		floorMsg.color.g = 1.0;
		floorMsg.color.b = 1.0;
		ss.str("");
		ss << "floor: [" << epuck.floorData[0] << ", " << epuck.floorData[1] << ", " << epuck.floorData[2] << ", " << epuck.floorData[3] << ", " << epuck.floorData[4] << "]";
		floorMsg.text = ss.str();
		floorPublisher.publish(floorMsg);
	}
}

void RosWrapper::updateMicrophone()
{
	if(epuck.enabled_sensors[MICROPHONE]) {
		std::stringstream ss;
		ss << epuck_name << "/base_link";
		visualization_msgs::Marker microphoneMsg;
		microphoneMsg.header.frame_id = ss.str();
		microphoneMsg.header.stamp = ros::Time::now();
		microphoneMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		microphoneMsg.pose.position.x = 0.15;
		microphoneMsg.pose.position.y = 0;
		microphoneMsg.pose.position.z = 0.11;
		geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(0);
		microphoneMsg.pose.orientation = odomQuat;
		microphoneMsg.scale.z = 0.01;
		microphoneMsg.color.a = 1.0;
		microphoneMsg.color.r = 1.0;
		microphoneMsg.color.g = 1.0;
		microphoneMsg.color.b = 1.0;
		ss.str("");
		ss << "mic: [" << epuck.micData[0] << ", " << epuck.micData[1] << ", " << epuck.micData[2] << "]";
		microphoneMsg.text = ss.str();
		microphonePublisher.publish(microphoneMsg);
	}
}

void RosWrapper::updateCamera()
{
	if(epuck.enabled_sensors[CAMERA]) {
			cv::Mat rgb888;
			cv_bridge::CvImage out_msg;
			out_msg.header.stamp = ros::Time::now();; // Same timestamp and tf frame as input image
			if(epuck.cam_mode == CAM_MODE_RGB) {
				rgb888 = cv::Mat(epuck.cam_height, epuck.cam_width, CV_8UC3);
				RGB565toRGB888(epuck.cam_width, epuck.cam_height, &epuck.camData[3], rgb888.data);
				out_msg.encoding = sensor_msgs::image_encodings::RGB8;
			} else {
				rgb888 = cv::Mat(epuck.cam_height, epuck.cam_width, CV_8UC1);
				rgb888.data = &epuck.camData[3];
				out_msg.encoding = sensor_msgs::image_encodings::MONO8;
			}
			out_msg.image = rgb888;
			imagePublisher.publish(out_msg.toImageMsg());
	}
}

void RosWrapper::updateActuators()
{
    if(changedActuators[MOTORS])
    {
        changedActuators[MOTORS] = false;
        epuck.setMotorSpeed(speedLeft, speedRight);
    }

    if(changedActuators[LEDS])
    {
    	for(int i = 0; i < ledNext.size(); i++)
    		if(ledCurrent[i] != ledNext[i])
    			epuck.setLed(i, ledNext[i]);

    	changedActuators[LEDS] = false;
    	ledCurrent = ledNext;
    }

    if(changedActuators[MOTORS_POS])
    {
        changedActuators[MOTORS_POS] = false;
    }
}

void RosWrapper::handlerVelocity(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Controls the velocity of each wheel based on linear and angular velocities.
    double linear = msg->linear.x/3;    // Divide by 3 to adapt the values received from the rviz "teleop" module that are too high.
    double angular = msg->angular.z/3;

    // Kinematic model for differential robot.
    double wl = (linear - (WHEEL_SEPARATION / 2.0) * angular) / WHEEL_DIAMETER;
    double wr = (linear + (WHEEL_SEPARATION / 2.0) * angular) / WHEEL_DIAMETER;

    // At input 1000, angular velocity is 1 cycle / s or  2*pi/s.
    speedLeft = int(wl * 1000.0);
    speedRight = int(wr * 1000.0);
    changedActuators[MOTORS] = true;

    if(DEBUG_SPEED_RECEIVED)std::cout << "[" << epuck_name << "] " << "new speed: " << speedLeft << ", " << speedRight << std::endl;
}

void RosWrapper::handleLeds(const std_msgs::UInt32MultiArray::ConstPtr& msg)
{
	if(msg->data.size() != 10)
	{
		ROS_WARN("led_array.size() != 10, data was droped");
		return;
	}

	ledNext = msg->data;
	changedActuators[LEDS] = true;
}



void RosWrapper::run()
{
    while (ros::ok())
    {
        updateSensors();
        updateActuators();
        ros::spinOnce();
    }
}

} /* namespace epuck_driver */

