/*
*	Author:		Kevin McLean
*	Project:	Project C.A.R. Brain board
*	Board:		MediaTek LinkIt™ ONE
*	Pinouts:	D9-Drive motor
*				D3-Stearing servo
*				Serial1(D0,D1)-Bluetooth
*				Serial(USB)-PI
*				D18-I2C SDA
*				D19-I2C SCL
*
*/

//uncomment for control type
//#define AUTONOMOUS
#define REMOTE_CONTROL
#define DEBUG

//uncomment for debugging communications active
//#define I2C
//#define USB
#define BLUETOOTH

#ifndef DEBUG
#define I2C
#define USB
#define BLUETOOTH
#endif

#include <Wire.h>
#include <Servo.h> 

Servo motor;
Servo steer;

const int encoders[4] = { 0, 1, 2, 3 };		//encoder board addresses
const int power_board = 4;

int steering_angle = 0;
int base_speed = 0;
int speed_error = 0;
int real_speed = 0;
int accumulated_speed_error = 0;
boolean emergency_stop = true;
unsigned long timer = 0;
int battery_voltage = 10;
int battery_current = 4;

void setup()
{
#ifdef DEBUG
	Serial.begin(9600);
	Serial.println("Debugging started.");
#endif

#ifdef I2C
	Wire.begin();                 //start I2C
#endif

#ifdef USB
	Serial.begin(9600);			//Start serial communications 
#endif

#ifdef BLUETOOTH
	Serial1.begin(9600);			//Start bluetooth communications 
#endif

	motor.attach(9);     // range 1000-2000		1500=stop
	steer.attach(3);    // range 50-120   below 90 turns right
	while (millis() < 2000)
	{
		motor.writeMicroseconds(1500);
		steer.write(90);
	}
	timer = millis();
}

void loop()
{
#ifdef I2C
	get_data();
#endif

#ifndef AUTONOMOUS
#ifdef BLUETOOTH
	if (Serial1.available() > 0)
	{
		base_speed = (Serial1.parseInt() - 500);
		steering_angle = (Serial1.parseInt() / 10 - 90);
		Serial1.println(battery_voltage + " " + battery_current);
		emergency_stop = false;
		timer = millis();
	}
	if ((millis() - timer) >= 700)
	{
		emergency_stop = true;
	}
#endif
#endif

#ifdef USB
	//put pi communication stuff here
#endif

#ifndef REMOTE_CONTROL
#ifdef BLUETOOTH
	if (Serial1.available() > 0)
	{
		emergency_stop = Serial1.parseInt();    // test this is still working.
		timer = millis();
	}
	if ((millis() - timer) >= 700)
	{
		emergency_stop = true;
	}
#endif
#endif

	//emergency_stop=false;
	if (emergency_stop)
	{
		motor.writeMicroseconds(1500);
		steer.write(90);
	}
	else
	{
		motor.writeMicroseconds(1500 - constrain(base_speed + speed_error, -500, 500)); //add conversion for pulse period to servo control
		steer.write(90 - constrain(steering_angle, -30, 40));
	}
}


//This needs to be fixed???
#ifdef I2C
void get_data()
{
	//read real speed and find the speed error
	for (int i = 0; i < 4; i++)
	{
		Wire.requestFrom(encoders[i], 1);
		if (Wire.available())
		{
			real_speed += Wire.parseInt();
		}
	}
	real_speed /= 4;
	if (!emergency_stop)
	{
		accumulated_speed_error += base_speed - real_speed;
	}
	speed_error = constrain(0.5*(base_speed - real_speed) + 0.5*accumulated_speed_error, -100, 100);

	//get data from power board
	Wire.requestFrom(power_board, 4);
	if (Wire.available())
	{
		battery_voltage = Wire.parseInt();
		battery_current = Wire.parseInt();
	}
	Serial1.write(battery_voltage + " " + battery_current);
}
#endif