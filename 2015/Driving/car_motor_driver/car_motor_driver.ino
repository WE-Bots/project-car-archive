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
int battery_current = 50;

void setup()
{
	Serial.begin(9600);
#ifdef I2C
	Wire.begin();                 //start I2C
#endif

#ifdef USB
	Serial.begin(9600);			//Start serial communications 
#endif

#ifdef BLUETOOTH
	Serial1.begin(9600);			//Start bluetooth communications 
#endif

#ifdef MOTORS
	motor.attach(9);     // range 1000-2000		1500=stop
	steer.attach(3);    // range 50-120   below 90 turns right		(this will need to be recalibrated when the car chassis is rebuilt)
	while (millis() < 2000)
	{
		motor.writeMicroseconds(1500);
		steer.write(90);
	}
#endif

	timer = millis();
}

void loop()
{
	//get data over I2C
#ifdef I2C
	//this needs to be fixed????
	//read real speed and find the speed error
	real_speed = 0;
	for (int i = 0; i < 4; i++)
	{
		int16_t encoder_period = 0;
		uint8_t id = 0;
		while (!wire_get_value(id, encoder_period, encoders[i])){}
		real_speed += encoder_period;
	}
	real_speed /= 4;
	if (!emergency_stop)
	{
		accumulated_speed_error += base_speed - real_speed;
	}
	speed_error = constrain(0.5*(base_speed - real_speed) + 0.5*accumulated_speed_error, -100, 100);

	//get data from power board
	uint8_t id = 0;
	int16_t value = 0;
	do
	{
		wire_send_value(1, 0, power_board);
	} while (!wire_get_value(id, value, power_board) && id != 1);
	battery_voltage = value;
	do
	{
		wire_send_value(2, 0, power_board);
	} while (!wire_get_value(id, value, power_board) && id != 2);
	battery_current = value;
#ifdef BLUETOOTH
	Serial1.write(battery_voltage + "," + battery_current);
#endif
#endif

	//Remote control code
#ifndef AUTONOMOUS
#ifdef BLUETOOTH
	if (Serial1.available() > 0)
	{
		base_speed = (Serial1.parseInt()/10 - 50);
		steering_angle = (Serial1.parseInt() / 10 - 90);
		//Serial.print("Speed: ");
		//Serial.print(base_speed);
		//Serial.print("     Angle: ");
		//Serial.println(steering_angle);
		Serial1.print(battery_voltage);
		Serial1.print(",");
		Serial1.println(battery_current);
		emergency_stop = false;
		timer = millis();
	}
	if ((millis() - timer) >= 700)
	{
		emergency_stop = true;
	}
#endif

#ifdef USB
	//put pi communication stuff here
	//only for getting the training video
	Serial.print('<');
	Serial.print(90 - constrain(steering_angle, -30, 40));
	Serial.print(',');
	Serial.print(1500 - constrain(base_speed + speed_error, -500, 500));
	Serial.print('>');
#endif
#endif

	//Autonomous code
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

#ifdef USB
	//put pi communication stuff here
#endif
#endif

	//emergency_stop=false;
	//drive the motors
#ifdef MOTORS
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
#endif
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
static boolean serial_get_value(uint8_t &id, int16_t &value) {
	// Static variables remain between calls
	static char buffer[3][7]; // Three parameters, length 6 + '\0' each
	static unsigned int charIdx[3] = { 0, 0, 0 }; // Index for each char container
	static unsigned int buffIdx = 0; // Current container receiving data
	static boolean started = false;
	// Read chars until there is no data in the buffer or we find a terminator
	while (Serial.available() > 0) {
		// Check for start of package
		if (Serial.peek() == '<') {
			Serial.read(); // Eat the byte
			started = true;
			buffIdx = 0;
			for (int i = 0; i < 3; ++i)
				charIdx[i] = 0;
		}
		else if (!started) {
			// Skip this char, go to next until a packet opener is found.
			Serial.read(); // Eat byte.
			continue;
		}
		else if (Serial.peek() == ',') {
			Serial.read(); // Eat the byte
			if ((charIdx[buffIdx] == 0) || (buffIdx >= 2)) {
				// Current is still empty or encountered a seventh char - bad.
				started = false; // Throw away this packet
				continue;
			}
			buffer[buffIdx][charIdx[buffIdx]++] = '\0'; // Add null terminator
			buffIdx++;
		}
		else if (Serial.peek() == '>') {
			if (!started || charIdx[0] == 0 || charIdx[1] == 0 || charIdx[2] == 0) {
				// No valid beginning of package was found, or a buffer
				// is still empty.
				// Buffers likely contain garbage. Toss the packet.
				Serial.read(); // Eat char
				started = false;
				continue;
			}
			// Don't eat the byte - leave it for a check outside of the loop
			buffer[buffIdx][charIdx[buffIdx]++] = '\0'; // Add null terminator
			break;
		}
		else // Must be a digit. Maybe check just in case?
			if (charIdx[buffIdx] == 5) {
				// Too long to be an int16_t integer.
				Serial.read(); // Eat char
				started = false; // Packet is invalid
				continue;
			}
		buffer[buffIdx][charIdx[buffIdx]++] = Serial.read();
	}

	// Check if buffer was emptied (ALWAYS check before peeking)
	if (Serial.available() == 0)
		return false; // Processed whole buffer without receiving complete packet

	// Check if terminator was reached
	if (Serial.peek() != '>')
		return false; // Not sure how you could even get here.
	// Maybe came in after the check in the loop.

	// Reserve return value
	uint8_t retid;
	int16_t retval;
	int16_t chksum;

	// Get id TODO: verify that they're all valid integers (atoi does not)
	retid = atoi(buffer[0]);
	// Get value
	retval = atoi(buffer[1]);
	// Get checksum
	chksum = atoi(buffer[2]);

	// Compare checksum
	if ((id + retval) != chksum)
		return false;

	// Checksum was valid, pass the variables back to the caller, return true
	id = retid;
	value = retval;
	return true;
}

/*
function serial_send_value
/brief
-Sends an id-value pair over serial in our format.
-Always returns true because I can't think of a failure mechanism.

/params
-uint8_t id: The id to send.
-int16_t value: The value to send.

/return
-bool: True if transmission was successful. Always true.
*/
static boolean serial_send_value(uint8_t id, int16_t value) {
	Serial.print('<');
	Serial.print(id);
	Serial.print(',');
	Serial.print(value);
	Serial.print(',');
	Serial.print(id + value);
	Serial.print('>');
	return true;
}
#endif

#ifdef I2C
/*
function wire_get_value
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
static boolean wire_get_value(uint8_t &id, int16_t &value, int address) {
	Wire.requestFrom(address, 9);
	// Static variables remain between calls
	static char buffer[3][7]; // Three parameters, length 6 + '\0' each
	static unsigned int charIdx[3] = { 0, 0, 0 }; // Index for each char container
	static unsigned int buffIdx = 0; // Current container receiving data
	static boolean started = false;
	// Read chars until there is no data in the buffer or we find a terminator
	while (Wire.available() > 0) {
		// Check for start of package
		if (Wire.peek() == '<') {
			Wire.read(); // Eat the byte
			started = true;
			buffIdx = 0;
			for (int i = 0; i < 3; ++i)
				charIdx[i] = 0;
		}
		else if (!started) {
			// Skip this char, go to next until a packet opener is found.
			Wire.read(); // Eat byte.
			continue;
		}
		else if (Wire.peek() == ',') {
			Wire.read(); // Eat the byte
			if ((charIdx[buffIdx] == 0) || (buffIdx >= 2)) {
				// Current is still empty or encountered a seventh char - bad.
				started = false; // Throw away this packet
				continue;
			}
			buffer[buffIdx][charIdx[buffIdx]++] = '\0'; // Add null terminator
			buffIdx++;
		}
		else if (Wire.peek() == '>') {
			if (!started || charIdx[0] == 0 || charIdx[1] == 0 || charIdx[2] == 0) {
				// No valid beginning of package was found, or a buffer
				// is still empty.
				// Buffers likely contain garbage. Toss the packet.
				Wire.read(); // Eat char
				started = false;
				continue;
			}
			// Don't eat the byte - leave it for a check outside of the loop
			buffer[buffIdx][charIdx[buffIdx]++] = '\0'; // Add null terminator
			break;
		}
		else // Must be a digit. Maybe check just in case?
			if (charIdx[buffIdx] == 5) {
				// Too long to be an int16_t integer.
				Wire.read(); // Eat char
				started = false; // Packet is invalid
				continue;
			}
		buffer[buffIdx][charIdx[buffIdx]++] = Wire.read();
	}

	// Check if buffer was emptied (ALWAYS check before peeking)
	if (Wire.available() == 0)
		return false; // Processed whole buffer without receiving complete packet

	// Check if terminator was reached
	if (Wire.peek() != '>')
		return false; // Not sure how you could even get here.
	// Maybe came in after the check in the loop.

	// Reserve return value
	uint8_t retid;
	int16_t retval;
	int16_t chksum;

	// Get id TODO: verify that they're all valid integers (atoi does not)
	retid = atoi(buffer[0]);
	// Get value
	retval = atoi(buffer[1]);
	// Get checksum
	chksum = atoi(buffer[2]);

	// Compare checksum
	if ((id + retval) != chksum)
		return false;

	// Checksum was valid, pass the variables back to the caller, return true
	id = retid;
	value = retval;
	return true;
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
}
#endif
//};
