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

class Serial_Manager
{
private:
  //----------------------------------------------------------------------------
  // Member objects
  serial::Serial serial_port_;
  ros::NodeHandle nh_;
  ros::Publisher comms_pub_;
  ros::Subscriber comms_sub_;
  MessageParser mp_;

  // Reused variables for parsing serial input
  uint8_t buffer_[BUFFER_SIZE];
  int8_t cnt_;

public:
  //----------------------------------------------------------------------------
  // Member functions
  // constructor
  Serial_Manager(std::string port, unsigned long baud)
    : serial_port_(port, baud,serial::Timeout::simpleTimeout(1000))
  {
    // Deal with topics
    comms_pub_ = nh_.advertise<car_serial_comms::ThrottleAndSteering>(
      "arduino_comms", 10);
    comms_sub_ = nh_.subscribe("vision_controller/drive_cmd", 10,
      &Serial_Manager::send_serial_callback, this);

    // Initialize buffer
    cnt_ = 0;
    for (int i=0; i<BUFFER_SIZE; ++i)
      buffer_[i] = 0;
  }

  // Don't need a destructor for now, nothing to do

  /*
   * send_serial_callback - send a received ROS message to the com port.
   */
  void send_serial_callback(const car_serial_comms::ThrottleAndSteering& msg)
  {
    // Build string to send
    std::stringstream ss;
    ss << "<" << msg.steering << "," << msg.throttle << ">";

    // Write out over serial port
    serial_port_.write(ss.str());
  }

  /*
   * check_serial - check if new data has been received on the serial port
   */
  void check_serial()
  {
    // Check for available data
    if (serial_port_.available() > 0)
    {
      // Load up the buffer
      while (serial_port_.available() && cnt_ < BUFFER_SIZE)
      {
        // Get one char at a time because lazy
        serial_port_.read(&buffer_[cnt_++], 1);
      }

      // Hand over the message and see if new values are produced
      if (mp_.parse_message(buffer_, BUFFER_SIZE, cnt_))
      {
        // Get updated values
        int steering = mp_.get_steering();
        int throttle = mp_.get_throttle();

        // Make message, load with data, then publish
        car_serial_comms::ThrottleAndSteering msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "/chassis";
        msg.steering = steering;
        msg.throttle = throttle;

        // // This is just a notification for debug - not a ROS message
        // ROS_INFO("%s", msg.data.c_str());

        // Publish to send out
        comms_pub_.publish(msg);
      }
    }
  }
};


/*
 * main - connect to Arduino on specified com port.
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
  Serial_Manager sm(port, baud);
  //****************************************************************************

  // Main loop
  while (ros::ok())
  {
    // Check for new data
    sm.check_serial();

    // Let ROS do stuff
    ros::spinOnce();

    // Wait for next iteration
    loop_rate.sleep();
  }

  // Return 0 even if not OK
  return 0;
}
