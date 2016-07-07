/*
*	Author:		Kevin McLean
*	Project:	Project C.A.R. Brain board
*	Board:		MediaTek LinkIt� ONE
*	Pinouts:	D9-Drive motor
*				D3-Stearing servo
*				Serial1(D0,D1)-Bluetooth
*				Serial(USB)-PI
*				D18-I2C SDA
*				D19-I2C SCL
*				D2-Collision avoidance interrupt
*				D4-Right IR
*				D5-Left IR
*				D6-Start button
*				D7-Drag race mode indicator LED
*				D13-Circuit race mode indicator LED
*				D12-Start button light
*	Discription:Handles decision making and communication between components of the car. Programed in Arduino.
*/

//uncomment for control type
//#define AUTONOMOUS
#define REMOTE_CONTROL
#define DEBUG

//uncomment for active communications while debugging
//#define I2C
//#define USB
#define BLUETOOTH
#define MOTORS

#ifndef DEBUG
#define I2C
#define USB
#define BLUETOOTH
#define MOTORS
#endif

#include <Wire.h>
#include <Servo.h>

#ifdef MOTORS
Servo motor;
Servo steer;
#endif

const int encoders[4] = { 8, 9, 10, 11 };		//encoder board addresses

int steering_angle = 0;
int base_speed = 0;
int speed_error = 0;
int real_speed = 0;
int accumulated_speed_error = 0;
boolean emergency_stop = true;
boolean stop = false;
unsigned long timer = 0;
int battery_voltage = 10;
int battery_current = 50;
int old_encoder_distance[4] = { 0, 0, 0, 0 };
unsigned long distance = 0;
int race_mode = 0;
unsigned long new_timer[4] = { 0, 0, 0, 0 };
unsigned long old_timer[4] = { 0, 0, 0, 0 };

#ifdef USB
static boolean serial_get_value(uint8_t &id, int16_t &value);
static boolean serial_send_value(int angle, int speed);
#endif

#ifdef I2C
boolean wire_get_value(int &data, int address);
#endif

void pause();

void setup()
{
	//Collision avoidance config
	pinMode(2, INPUT);
	//attachInterrupt(0, pause, CHANGE);
	pinMode(4, INPUT);
	pinMode(5, INPUT);

#ifdef I2C
	Wire.begin();                 //start I2C
#endif

#ifdef USB
	Serial.begin(115200);			//Start serial communications 
#endif

#ifdef BLUETOOTH
	Serial1.begin(9600);			//Start bluetooth communications 
#endif

#ifdef AUTONOMOUS
	//Start button config
	pinMode(6, INPUT_PULLUP);
	pinMode(7, OUTPUT);
	pinMode(13, OUTPUT);
	pinMode(12, OUTPUT);

	//start button selection
	while (!race_mode)
	{
		digitalWrite(7, LOW);
		digitalWrite(13, LOW);
		digitalWrite(12,LOW);
		if (!digitalRead(6))
		{
			race_mode = 1;
			digitalWrite(7, HIGH);
			digitalWrite(12,HIGH);
			timer = millis();
			while ((millis() - timer) < 2000)
			{
				if ((!digitalRead(6)) && ((millis() - timer) > 1000))
				{
					race_mode = 2;
					digitalWrite(13, HIGH);
					break;
				}
			}
		}
	}
#endif

#ifdef MOTORS
	motor.attach(9);     // range 1000-2000		1500=stop
	steer.attach(3);    // range 50-120   below 90 turns right		(this will need to be recalibrated when the car chassis is rebuilt)
	timer = millis();
	while ((millis()-timer) < 12000)
	{
		motor.writeMicroseconds(1500);
		steer.write(90);
	}
#endif
	timer = millis();
}

void loop()
{
	//set speed and steering angle when in autonomous mode
#ifdef AUTONOMOUS
	if (race_mode == 1)
	{
#ifdef USB
		//put pi communication stuff here
		serial_get_value(steering_angle, base_speed);
#endif
			base_speed = 2000;
		steering_angle = 2;
		if ((distance % 100) < 15)
			steering_angle = 1;
		if (distance > 1690)	//need to figure out what this value should actually be
		{
			base_speed = 0;
		}
	}
	else if (race_mode == 2)
	{
		if ((millis()%1000)<500)
			digitalWrite(12,HIGH);
		else
			digitalWrite(12,LOW);
#ifdef USB
		//put pi communication stuff here
		serial_get_value(battery_voltage, battery_current);
		serial_send_value(battery_voltage, battery_current);
		//if (base_speed != 0)
		steering_angle = battery_voltage;
			base_speed = 2000;
			if (base_speed < 4000)
				base_speed++;
#endif
	}
#endif

	//get data over I2C
#ifdef I2C
	//this needs to be fixed????
	//read real speed and find the speed error
	real_speed = 0;
	if (!(emergency_stop||stop||base_speed==0))
	{
		for (int i = 0; i < 4; i++)
		{
			int encoder_distance = 0;
			new_timer[i] = micros();
			while (!wire_get_value(encoder_distance, encoders[i]))
			{
				delay(1);
				new_timer[i] = micros();
			}
			//check for rollover
			if(encoder_distance<(old_encoder_distance[i]%256))
			{
				//there was roll over
				real_speed += (encoder_distance - (old_encoder_distance[i] % 256) + 256)*52500/(new_timer[i]-old_timer[i]);
				old_timer[i] = new_timer[i];
				old_encoder_distance[i]+=encoder_distance-(old_encoder_distance[i]%256)+256;
			}
			else
			{
				//no rollover
				real_speed += (encoder_distance - (old_encoder_distance[i] % 256)) *52500/ (new_timer[i] - old_timer[i]);
				old_timer[i] = new_timer[i];
				old_encoder_distance[i]+=encoder_distance-(old_encoder_distance[i]%256);
			}
		}
		//average values
		real_speed = real_speed/4;		//speed in mm/s
		distance = (old_encoder_distance[0]+old_encoder_distance[1]+old_encoder_distance[2]+old_encoder_distance[3])/4;

		//calculate speed error
		accumulated_speed_error += base_speed - real_speed;
		speed_error = constrain(0.9*(base_speed - real_speed) + 0.1*accumulated_speed_error, -1000, 1000);
	}

	//get data from power board
	//while (!wire_get_value(battery_voltage, battery_current, power_board)){delay(10);}
#endif

	//Remote control code
#ifdef REMOTE_CONTROL
#ifdef BLUETOOTH
	if (Serial1.available() > 1)
	{
		base_speed = (Serial1.parseInt() / 5 - 100);
		if ((base_speed < -100) || (base_speed>100))
		{
			base_speed = 0;
		}
		steering_angle = (Serial1.parseInt() / 10 - 90);
		Serial1.print(battery_voltage);
		Serial1.print(",");
		Serial1.print(battery_current);
		Serial1.print(",");
		Serial1.print(base_speed);
		Serial1.print(",");
		Serial1.println(steering_angle);
		emergency_stop = false;
		timer = millis();
	}
	if ((millis() - timer) >= 300)
	{
		emergency_stop = true;
	}
#endif

#ifdef USB
	//put pi communication stuff here
	//only for getting the training video
	serial_send_value(constrain(steering_angle, -30, 40), base_speed);
#endif
#endif

#ifdef AUTONOMOUS
	//collision avoidance steering
	if (digitalRead(4))
	{
		steering_angle += 10;
	}
	if (digitalRead(5))
	{
		steering_angle -= 10;
	}

	//communicate with the remote emergency stop app
#ifdef BLUETOOTH
	if (Serial1.available() > 0)
	{
		emergency_stop = Serial1.parseInt();    // test this is still working.
		Serial1.print(battery_voltage);
		Serial1.print(",");
		Serial1.print(battery_current);
		Serial1.print(",");
		Serial1.print(real_speed);
		Serial1.print(",");
		Serial1.println(distance);
		timer = millis();
	}
	if ((millis() - timer) >= 300)
	{
		emergency_stop = true;
	}
#endif
#endif

	//emergency_stop=false;
	//drive the motors
#ifdef MOTORS
	if (emergency_stop||stop||base_speed==0)
	{
		motor.writeMicroseconds(1500);
		steer.write(90);
	}
	else
	{
		motor.writeMicroseconds(1500 + constrain((base_speed), -100, 100)); //add conversion for pulse period to servo control
		steer.write(90 + constrain(steering_angle, -30, 40));
	}
#endif
}

//collision avoidance ISR
void pause()
{
	stop = digitalRead(2);
}

/*
class Car_Comms
/brief
-Sends and receives serial id-value pairs.
-Uses and deals with 16-bit checksums. In this case it's practically sending
data twice.
-All methods are static. Do not instantiate.
*/
//class Car_Comms {
//public:

#ifdef USB
/*
function serial_get_value
/brief
-Attempts to read an id and value using our comms protocol.
-Reference arguments are only updated if a new package is completed -
they will be unmodified if a complete, valid entry is not obtained.

/params
-uint8_t &id: Reference to id variable to read into
-int16_t &value: Reference to value variable to read into

/return
-bool: True if a id-value pair was obtained. False otherwise.
*/
boolean serial_get_value(int &first, int &second)
{
	// Static variables remain between calls
	int buffer[2] = { 0, 0 };
	char buf16[4]={'0','0','0','\0'};
	boolean started = false;
	boolean success = false;
	// Read chars until there is no data in the buffer or we find a terminator
	while (Serial.available() >= 5)
	{
		// Check for start of package
		if (Serial.peek() == '<')
		{
			Serial.read(); // Eat the byte
			started = true;
		}
		else
		{
			// Skip this char, go to next until a packet opener is found.
			Serial.read(); // Eat byte.
			continue;
		}

		//read the first int
		for (int i = 0; i < 3 && Serial.peek() != ','; i++)
			buf16[i] = Serial.read();
		buffer[0] = atoi(buf16);

		//check for parser
		if (Serial.peek() == ',')
		{
			Serial.read(); // Eat the byte
		}
		else
		{
			started = false; // Throw away this packet
			continue;
		}

		//read the second int
		for (int i = 0; i < 3 && Serial.peek() != '>'; i++)
			buf16[i] = Serial.read();
		buffer[1] = atoi(buf16);

		//check for terminator
		if (Serial.peek() == '>')
		{
			Serial.read(); // Eat the byte
			//pass the variables back to the caller
			first = buffer[0];
			second = buffer[1];
			success = true;
		}
		else
		{
			started = false; // Throw away this packet
			continue;
		}
	}
	return success;
}

/*
function serial_send_value
/brief
-Sends steering angle and real speed over serial in our format.
-Always returns true because I can't think of a failure mechanism.

/params
-int angle: The steering angle
-int speed: The real speed.

/return
-bool: True if transmission was successful. Always true.
*/
boolean serial_send_value(int angle, int speed)
{
	Serial.print('<');
	Serial.print(angle);
	Serial.print(',');
	Serial.print(speed);
	Serial.print('>');
	return true;
}
#endif

#ifdef I2C
/*
function wire_get_value
/brief
-Attempts to read a uint16_t and a uint8_t using our comms protocol.
-Reference arguments are only updated if a new package is completed -
they will be unmodified if a complete, valid entry is not obtained.

/params
-int &first: Reference to uint16_t variable to read into
-int &second: Reference to uint8_t variable to read into

/return
-bool: True if a packet was obtained. False otherwise.
*/
boolean wire_get_value(int &data, int address)
{
	Wire.requestFrom(address, 1);
			if (Wire.available() > 0)
			{
				data=Wire.read();
				return true;
			}
			else
			{
				return false; //return false; //didn't receive the correct number of bytes
			}
}

/*
function wire_send_value
/brief
-Sends an id-value pair over serial in our format.
-Always returns true because I can't think of a failure mechanism.

/params
-uint8_t id: The id to send.
-int16_t value: The value to send.

/return
-bool: True if transmission was successful.
*/
/*
static boolean wire_send_value(uint8_t id, int16_t value, int address) {
Wire.beginTransmission(address);
Wire.print('<');
Wire.print(id);
Wire.print(',');
Wire.print(value);
Wire.print(',');
Wire.print(id + value);
Wire.print('>');
return !Wire.endTransmission();
}*/
#endif
//};
