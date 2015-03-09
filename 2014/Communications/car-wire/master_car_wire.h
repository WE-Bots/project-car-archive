
class master_car_wire {

private:
    uint16_t _sum, _checkSum;

public:
    bool request(void* buffer, unsigned char bytes, unsigned char devAddr, unsigned               char register);
    bool write(void* buffer, unsigned char bytes, unsigned char devAddr, unsigned                 char register);

}; 