#2015 Code - Vision

This is the development code for the new vision system for the 2015 car. This year's main goals are to perform tracking better, actually look for the staring light, and be MUCH easier to understand (C++, proper organization).

There are two planned stages to development. The first is remote controlling the car, while capturing steering and throttle readings in addition to the video feed. The second stage is to apply machine learning to the readings to retune our system from last year to re-create the recorded steering and throttle as closely as possible when running against the recorded video.

##Note
The ROS implementation discussed in this section is replacing the C++ Raspberry Pi Camera libraries that have been left in this directory.

Look at the code in the "ROS Source Code" directory instead. That directory (\*cough\* *will* \*cough\*) consists of the packages that we are making. There is no markdown in the directory to prevent making a mess of the ROS workspace on the Pi.

##Hardware
There are three hardware systems that concern this sub-team: an Android phone, an Arduino microcontroller with Bluetooth, and a Raspberry Pi with a camera module running ROS. The phone connects to the Arduino over Bluetooth, and the Arduino and Raspberry Pi communicate via serial comms over a short USB cable.

##Stage 1
![Diagram 1](http://i.imgur.com/xt48jcH.png "Vision Comms Stage 1")

In the first stage, a person uses the phone to control the car and to tell the Pi when to record. The phone connects to the Arduino using Bluetooth. The Arduino controls the steering and throttle, and passes those values onto the Pi so that they can be recorded in time with the video. The Arduino also passes along the recording command to the Pi so that it knows when to record.

The Arduino and Pi will communicate via serial over a short USB cable. The Pi records video, steering angle, and throttle using ROS's 'rosbag' functionality. This allows the data to be played back on another computer as if it were happening in real time.

This should help with development because anyone on the team will be able to work on it whenever, and without the actual car. Also looking into getting the ROS bag into MATLAB because it has some nice stuff for doing machine learning.

### To Do List
- Create ROS Launch script to get roscore, raspicam, and carcomms nodes running
  - Should be configured to run in background at startup
- Create carcomms node
  - Just needs to receive from the Arduino and broadcast into a ROS topic
  - Broadcasts steering, throttle, and recording flag
- Setup test track
- Setup hardware


##Stage 2
![Diagram 2](http://i.imgur.com/03uP9tb.png "Vision Comms Stage 2")

In the second stage, the phone is just used as a remote kill-switch for the car. The flow of data between the Pi and Arduino is reversed. The Pi will use the learned parameters to create throttle and steering commands, and send them over serial to the Arduino.
