/*
 * @author: Andrew Simpson
 * @createdOn: June 20, 2015
 * @brief: Node for serial communications between Arduino controller and Pi.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <serial/serial.h>

#include <sstream>

#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>

// Project headers
#include "car_serial_comms/MessageParser.h"

// Custom message type
#include "car_serial_comms/ThrottleAndSteering.h"

#define BUFFER_SIZE 20
/*
 * Main - connect to Arduino on specified comm port.
 */
int main(int argc, char **argv)
{
  //****************************************************************************
  // Initialize ROS
  //
  // Pass args and node name to ROS for processing...
  ros::init(argc, argv, "serial_comms");
  //ros::param::param<std::string>("~port", port, "/dev/ttyUSB0");
  //ros::param::param<int>("~baud", baud, 115200);
  // NodeHandle is how the ROS communications system is accessed.
  ros::NodeHandle nh;

  // Advertise topic for outputting messages from the Arduino (name, buf_size)
  ros::Publisher serial_comms_pub = nh.advertise<car_serial_comms::ThrottleAndSteering>("arduino_comms",100);

  // Run at 60 Hz - twice that of the camera
  // This basically means no lag from the camera's perspective.
  ros::Rate loop_rate(60);
  //****************************************************************************

  //****************************************************************************
  // Initialize serial
  //
  // Open serial port
  std::string port = "/dev/ttyUSB0"; //Eg. "/dev/ttyUSB0"
  unsigned long baud = 115200;
  serial::Serial serial_port(port, baud,serial::Timeout::simpleTimeout(1000));
  if (!serial_port.isOpen())
    return 1;
  //****************************************************************************

  // For now, we only recieve, and never send.

  // Reused variables
  uint8_t buffer[BUFFER_SIZE];
  int8_t cnt = 0;
  int steering, throttle;
  MessageParser mp = MessageParser();
  // Initialize buffer
  for (int i=0; i<BUFFER_SIZE; ++i)
    buffer[i] = 0;
  // Main loop
  while (ros::ok())
  {
    // Check for available data
    if (serial_port.available() > 0)
    {
      // Load up the buffer
      while (serial_port.available() && cnt < BUFFER_SIZE)
      {
        // Get one char at a time because lazy
        serial_port.read(&buffer[cnt++], 1);
      }

      // Hand over the message and see if new values are produced
      if (mp.parse_message(buffer, BUFFER_SIZE, cnt))
      {
        // Get updated values
        steering = mp.get_steering();
        throttle = mp.get_throttle();

        // Make message, load with data, then publish
        car_serial_comms::ThrottleAndSteering msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "/world";
        msg.steering = steering;
        msg.throttle = throttle;

        // // Package into crappy message
        // std::stringstream ss;
        // ss << steering << "," << throttle;
        // msg.data = ss.str();

        // // This is just a notification for debug - not a ROS message
        // ROS_INFO("%s", msg.data.c_str());

        // Publish to send out
        serial_comms_pub.publish(msg);
      }
    }

    // Let ROS do stuff
    ros::spinOnce();

    // Wait for next iteration
    loop_rate.sleep();
  }

  // Return 0 even if not OK
  return 0;
}
