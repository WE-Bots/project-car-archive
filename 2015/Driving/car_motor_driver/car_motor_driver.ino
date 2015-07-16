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
*				D2-Collision avoidance interrupt
*				D4-Right IR
*				D5-Left IR
*				D6-Start button
*				D7-Drag race mode indicator LED
*				D13-Circuit race mode indicator LED
*	Discription:Handles decision making and communication between components of the car. Programed in Arduino.
*/

//uncomment for control type
#define AUTONOMOUS
//#define REMOTE_CONTROL
#define DEBUG

//uncomment for active communications while debugging
//#define I2C
//#define USB
//#define BLUETOOTH
//#define MOTORS

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
const int power_board = 12;

int steering_angle = 0;
int base_speed = 0;
int speed_error = 0;
int real_speed = 0;
int accumulated_speed_error = 0;
boolean emergency_stop = true;
unsigned long timer = 0;
int battery_voltage = 10;
int battery_current = 50;
int old_encoder_distance[4] = { 0, 0, 0, 0 };
unsigned long distance = 0;
int race_mode = 0;

#ifdef USB
static boolean serial_get_value(uint8_t &id, int16_t &value);
static boolean serial_send_value(int angle, int speed);
#endif

#ifdef I2C
boolean wire_get_value(int &first, int &second, int address);
#endif

void setup()
{
	//Collision avoidance config
	//attachInterrupt(0, pause, CHANGE);

	Serial.begin(9600);
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

	//start button selection
	while (!race_mode)
	{
		digitalWrite(7, LOW);
		digitalWrite(13, LOW);
		if (!digitalRead(6))
		{
			race_mode = 1;
			digitalWrite(7, HIGH);
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
	while ((millis()-timer) < 2000)
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
	if (!emergency_stop)
	{
		for (int i = 0; i < 4; i++)
		{
			int encoder_period = 0;
			int encoder_distance = 0;
			while (!wire_get_value(encoder_period, encoder_distance, 8))
			{
				delay(10);
			}
			real_speed += encoder_period;
			//check for rollover
			if(encoder_distance<(old_encoder_distance[i]%256))
			{
				//there was roll over
				old_encoder_distance[i]+=encoder_distance-(old_encoder_distance[i]%256)+256;
			}
			else
			{
				//no rollover
				old_encoder_distance[i]+=encoder_distance-(old_encoder_distance[i]%256);
			}
		}
		//average values
		real_speed /= 4;
		distance = (old_encoder_distance[0]+old_encoder_distance[1]+old_encoder_distance[2]+old_encoder_distance[3])/4;

		//calculate speed error
		accumulated_speed_error += base_speed - real_speed;
		speed_error = constrain(0.5*(base_speed - real_speed) + 0.5*accumulated_speed_error, -100, 100);
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
		Serial1.print(real_speed);
		Serial1.print(",");
		Serial1.pringln(distance);
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
	serial_send_value(constrain(steering_angle, -30, 40), real_speed);
#endif
#endif

	//Autonomous code
#ifdef AUTONOMOUS
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
		Serail1.println(distance);
		timer = millis();
	}
	if ((millis() - timer) >= 300)
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
		motor.writeMicroseconds(1500 + constrain(base_speed + speed_error, -500, 500)); //add conversion for pulse period to servo control
		steer.write(90 + constrain(steering_angle, -30, 40));
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
static boolean serial_get_value(uint8_t &id, int16_t &value) 
{
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
-Sends steering angle and real speed over serial in our format.
-Always returns true because I can't think of a failure mechanism.

/params
-int angle: The steering angle
-int speed: The real speed.

/return
-bool: True if transmission was successful. Always true.
*/
static boolean serial_send_value(int angle, int speed) 
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
boolean wire_get_value(int &first, int &second, int address)
{
	if (Wire.requestFrom(address, 7) != 7)
		return false; //didn't receive the correct number of bytes

	// Static variables remain between calls
	char buf16[3] = { 0, 0, '\0' }; //2 bytes +'\0'
	char buf8 = 0;
	char bufnibble[7];

	// Read data
	Wire.readBytes(bufnibble, 7);
	buf16[0]=((bufnibble[0]&0x0f)<<4)|(bufnibble[1]&0x0f);
	buf16[2]=((bufnibble[2]&0x0f)<<4)|(bufnibble[3]&0x0f);
	buf8=((bufnibble[4]&0x0f)<<4)|(bufnibble[5]&0x0f);
	Serial.print("one: ");
	Serial.println((byte)buf16[0]);
	Serial.print("two: ");
	Serial.println((byte)buf16[1]);
	Serial.print("three: ");
	Serial.println((byte)buf8);
	Serial.print("xor: ");
	Serial.println((byte)(bufnibble[6]&0x0f));
	Serial.println((byte)(bufnibble[0] ^ bufnibble[1] ^ bufnibble[2]^ bufnibble[3]^ bufnibble[4]^ bufnibble[5]));

	//check XOR
	if ((bufnibble[6]&0x0f) != bufnibble[0] ^ bufnibble[1] ^ bufnibble[2]^ bufnibble[3]^ bufnibble[4]^ bufnibble[5])
		return false;	//invalid XOR

	// XOR was valid, pass the variables back to the caller, return true
	first = atoi(buf16);
	second = buf8;
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
