# IMU and GPS Module
## Hardware
This module uses a Teensy 3.1 because of its speed, ease of use, and (primarilly) two I2C busses. This allows the Teensy to have a slave IMU while being a slave itself to the brain board.

The GPS unit oprates over serial.

## Software
Included here are the libraries the Teensy will need to use this code, namely the rewritten I2C library that uses the Teensy's hardware interupts correctly, and the GPS library that it uses to parse the strings from the GPS unit.

## IMU
The IMU consists of a 3 axis accelerometer, gyroscope, and magnetometer. I have reused SparkFun's IMU code for their Razor IMU board, which uses the same accelerometer and magnetometer, though I have repalaced their hardware specific code by something more flexible written by LCushman. We really just kept the math.

This really just needs a little tuning and should then be good to go for next year.

## GPS
The GPS is a Paralax __. It is not labeled, so DO NOT lose this number. If lost, contact ahfergus1, who probably still has the baggie somewhere in his room.