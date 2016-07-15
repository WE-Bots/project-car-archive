#include "USB.h"

void USBInit()
{
    U1PWRCbits.USBPWR=1;    //power on the USB module
    U1IE=0;                 //enable interrupts (value needs to be set still)
    U1EIE=0;                //enable error interrupts (value needs to be set still)
    U1EP0bits.EPRXEN=1;     //enable receive on endpoint 0
    U1EP0bits.EPTXEN=1;     //enable transmit on endpoint 0
    U1EP0bits.EPHSHK=1;     //enable handshaking on endpoint 0

    //Endpoint Buffer Description Table Start Address
    U1BDTP1=0;              //value needed
    U1BDTP2=0;              //value needed
    U1BDTP3=0;              //value needed

    U1OTGCONbits.OTGEN=0;   //not an OTG device
    U1CONbits.HOSTEN=0;     //set as USB device
    U1CONbits.USBEN=1;      //enable the module to connect to the bus
}
