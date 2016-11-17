/*
 * epuck_driver_my.cpp
 *
 *  Created on: Oct 13, 2016
 *      Author: josl
 */
#include <iostream>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
//ROS Message types to include
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt32MultiArray.h>



sensor_msgs::Range msg_prox0;
sensor_msgs::Range msg_prox1;
sensor_msgs::Range msg_prox2;
sensor_msgs::Range msg_prox3;
sensor_msgs::Range msg_prox4;
sensor_msgs::Range msg_prox5;
sensor_msgs::Range msg_prox6;
sensor_msgs::Range msg_prox7;

sensor_msgs::Imu msg_accel;
visualization_msgs::Marker msg_light;
visualization_msgs::Marker msg_selector;
visualization_msgs::Marker msg_motor_speed;
visualization_msgs::Marker msg_microphone;

void callback_prox0(const sensor_msgs::Range& msg){ msg_prox0 = msg; }
void callback_prox1(const sensor_msgs::Range& msg){ msg_prox1 = msg; }
void callback_prox2(const sensor_msgs::Range& msg){ msg_prox2 = msg; }
void callback_prox3(const sensor_msgs::Range& msg){ msg_prox3 = msg; }
void callback_prox4(const sensor_msgs::Range& msg){ msg_prox4 = msg; }
void callback_prox5(const sensor_msgs::Range& msg){ msg_prox5 = msg; }
void callback_prox6(const sensor_msgs::Range& msg){ msg_prox6 = msg; }
void callback_prox7(const sensor_msgs::Range& msg){ msg_prox7 = msg; }

void callback_accel(const sensor_msgs::Imu& msg){ msg_accel = msg; }
void callback_selector(const visualization_msgs::Marker& msg){ msg_selector = msg; }
void callback_light(const visualization_msgs::Marker& msg){ msg_light = msg; }
void callback_motor_speed(const visualization_msgs::Marker& msg){ msg_motor_speed = msg; }
void callback_microphone(const visualization_msgs::Marker& msg){ msg_microphone = msg; }



void sensor_read_test(ros::NodeHandle * n)
{
	int queue_size = 10;
    ros::Subscriber sub0 = n->subscribe("/proximity0", queue_size, callback_prox0);
    ros::Subscriber sub1 = n->subscribe("/proximity1", queue_size, callback_prox1);
    ros::Subscriber sub2 = n->subscribe("/proximity2", queue_size, callback_prox2);
    ros::Subscriber sub3 = n->subscribe("/proximity3", queue_size, callback_prox3);
    ros::Subscriber sub4 = n->subscribe("/proximity4", queue_size, callback_prox4);
    ros::Subscriber sub5 = n->subscribe("/proximity5", queue_size, callback_prox5);
    ros::Subscriber sub6 = n->subscribe("/proximity6", queue_size, callback_prox6);
    ros::Subscriber sub7 = n->subscribe("/proximity7", queue_size, callback_prox7);

	ros::Subscriber subA = n->subscribe("/accel", queue_size, callback_accel);
	ros::Subscriber subB = n->subscribe("/light", queue_size, callback_light);
	ros::Subscriber subC = n->subscribe("/selector", queue_size, callback_selector);
	ros::Subscriber subD = n->subscribe("/motor_speed", queue_size, callback_motor_speed);
	ros::Subscriber subE = n->subscribe("/microphone", queue_size, callback_microphone);


    while(ros::ok())
    {
        system("clear");
		std::cout << "Accel:";
		std::cout << " x: " << msg_accel.linear_acceleration.x;
		std::cout << " y: " << msg_accel.linear_acceleration.y;
		std::cout << " z: " << msg_accel.linear_acceleration.z;
		std::cout << std::endl;

		std::cout << "Prox:";
		std::cout << "  " << msg_prox0.range;
		std::cout << ", " << msg_prox1.range;
		std::cout << ", " << msg_prox2.range;
		std::cout << ", " << msg_prox3.range;
		std::cout << ", " << msg_prox4.range;
		std::cout << ", " << msg_prox5.range;
		std::cout << ", " << msg_prox6.range;
		std::cout << ", " << msg_prox7.range;
		std::cout << std::endl;

		std::cout << msg_motor_speed.text << std::endl;
		std::cout << msg_microphone.text << std::endl;
		std::cout << msg_light.text << std::endl;
		std::cout << msg_selector.text << std::endl;
        ros::Duration(0.1).sleep();
    }
}



void motor_speed_test(ros::NodeHandle * n)
{
	ros::Publisher motor_vel_publisher = n->advertise<geometry_msgs::Twist>("mobile_base/cmd_vel", 1);

    while(ros::ok())
    {
    	geometry_msgs::Twist msg1;
        msg1.linear.x = 2;
        motor_vel_publisher.publish(msg1);
        ros::Duration(0.5).sleep();

    	geometry_msgs::Twist msg2;
        msg2.angular.z = 0.35;
        motor_vel_publisher.publish(msg2);
        ros::Duration(2).sleep();
    }
}


void leds_test(ros::NodeHandle * n)
{
    ros::Publisher pub = n->advertise<std_msgs::UInt32MultiArray>("leds", 1);

    int run_index = 0;
    std::vector<uint32_t> led_array(10,0);

    while(ros::ok())
    {
        std_msgs::UInt32MultiArray msg;
//        for(int i = 0; i < 10; i++)
//        	led_array[i]=rand() % 3;
		led_array[run_index] = (led_array[run_index] + 1) % 2;
		run_index = (run_index + 1) % 10;
        msg.data = led_array;
        pub.publish(msg);
        ros::Duration(0.5).sleep();
    }
}


void image_view_test()
{
    system("rosrun image_view image_view image:=/camera");
}



int main(int argc,char *argv[])
{
	std::cout << "Starting epuck interface demo" << std::endl;
    ros::init(argc, argv, "epuck_interface_demo");
    ros::NodeHandle n;

    boost::thread thread_sensor_read_test(sensor_read_test, &n);
    boost::thread thread_motor_speed(motor_speed_test, &n);
    boost::thread thread_led(leds_test, &n);
    //boost::thread thread_img_view(image_view_test);

    ros::spin();

    thread_sensor_read_test.join();
    thread_motor_speed.join();
    thread_led.join();
    //thread_img_view.join();
}


