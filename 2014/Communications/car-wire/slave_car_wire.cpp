#include "slave_car_wire.h"

void slave_car_wire::receiveEvent(){

	if (Wire.available()){}
		_cmd = Wire.readByte();
	}

}

void slave_car_wire::requestEvent(){

	//You kinda need to write this one yourself

}