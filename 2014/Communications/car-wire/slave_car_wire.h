
static class slave_car_wire{

private:
	uint16_t _checkSum;
	unsigned char _cmd;

public:	
	void recieveEvent();
	void requestEvent();

};