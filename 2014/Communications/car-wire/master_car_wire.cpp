#include "master_car_wire.h"

bool master_car_wire::requestFrom(void* buffer, unsigned char bytes, unsigned char devAddr, unsigned char register){
	_sum=0;

	Wire.beginTransmission(register);
	Wire.write(register);
	Wire.endTranmission();

	Wire.requestFrom(devAddr, bytes+2);
	while(Wire.available() < (bytes+2){}
	for (int i=0; i<bytes; i++){
		buffer[i] = Wire.read();
		_sum += buffer[i];
	}
	void * checkSumAddr = &_checkSum;
	checkSumAddr[0]=Wire.read();
	checkSumAddr[1]=Wire.read();
	if(_sum!=_checkSum)
		return false;
	else
		return true;
}

bool master_car_wire::write(void* buffer, unsigned char bytes, unsigned char devAddr, unsigned char register){
	_sum=0;
	Wire.beginTransmission(devAddr);
	Wire.write(register);
	for (int i=0; i<bytes; i++){
		Wire.write(buffer[i]);
		_sum+=buffer[i];
	}
	Wire.endTranmission();

	//Get checksum
	Wire.beginTransmission(devAddr);
	Wire.write(CHECKSUM_ADDR);
	Wire.endTranmission();

	Wite.requestFrom(devAddr, 2);
	while(Wire.available() < 2) {}
	void * checkSumAddr = &_checkSum;
	checkSumAddr[0]=Wire.read();
	checkSumAddr[1]=Wire.read();
	if(_sum!=_checkSum)
		return false;
	else
		return true;
}
