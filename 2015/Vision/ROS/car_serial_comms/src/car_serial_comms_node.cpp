/*
 * @author: Andrew Simpson
 * @createdOn: June 20, 2015
 * @brief: Node for serial communications between Arduino controller and Pi.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <serial.h>

#include <sstream>

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

  // NodeHandle is how the ROS communications system is accessed.
  ros::NodeHandle nh;

  // Advertise topic for outputting messages from the Arduino (name, buf_size)
  ros::Publisher serial_comms_pub = nh.advertise<std_msgs::String>("arduino_comms",100);

  // Run at 60 Hz - twice that of the camera
  // This basically means no lag from the camera's perspective.
  ros::Rate loop_rate(60);
  //****************************************************************************

  //****************************************************************************
  // Initialize serial
  //
  // Open serial port
  String port="???";
  unsigned long baud = 115200;
  serial::Serial serial_port(port, baud,serial::Timeout::simpleTimeout(1000));
  if (!serial_port.isOpen())
    exit EXIT_ERROR;
  //****************************************************************************

  // For now, we only recieve, and never send.

  // Reused variables
  uint8_t buffer[10];
  int8_t idx = 0;
  int steering, throttle;
  MessageParser mp;
  // Main loop
  while (ros::ok())
  {
    // Check for available data
    if (serial_port.available() > 0)
    {
      // Load up the buffer
      while (serial_port.available() && idx < 9)
      {
        serial_port.read(&buffer[idx++]);
      }

      // Hand over the message and see if new values are produced
      if (mp.parse_message(buffer,10))
      {
        steering = mp.getSteering();
        throttle = mp.getThrottle();

        // Make message, load with data, then publish
        std_msgs::String msg;

        std::stringstream ss;
        ss << steering << "," << throttle;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

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
