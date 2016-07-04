/*
 * File:    GPS.c
 * Author:  Michael Buchel
 * 
 * GPS interface file
 * 
 * Created on July 3, 2016, 7:55 PM
 */

#include "GPS.h"

int i; //Keeps track of letter
int good; //Used to check if the command string is the same
int commaNumber, counter;
char receivedChars[300]; //Keeps track of chars brought in
char command[7] = "$GPRMC"; //Command string
char Time[9], Status, Latitude[7], DirectionNS,
        Longitude[8], DirectionEW, Speed[3], Heading[3],
        Date[6], Magnetic[3], MagneticEW, Mode,
        Checksum;

void init() {
    RPINR18 = 0x003E; //Sets pin 48 as UART receiver
    
    U1MODEbits.STSEL = 0; //1 stop bit
    U1MODEbits.PDSEL = 0; //No parity 8 data bits
    U1MODEbits.ABAUD = 0; //Auto baud disabled
    U1MODEbits.BRGH = 0; //Standard speed mode
    
    U1BRG = BRGVAL; //Baud rate setting for 4800 
    U1STAbits.URXISEL = 0; //Interrupt after 1 character is recieved
    U1MODEbits.UARTEN = 1; //Enable UART
    
    for (i = 0; i < 300; i++) { //Clear received array
        receivedChars[i] = 0x0;
    }
    
    i = 0; //Set counter back to 0
    
    return;
}

int main(void) {
    init();
    
    while(1) {
        if (U1STAbits.FERR == 1) {
            continue;
        }
        
        if (U1STAbits.OERR == 1) {
            U1STAbits.OERR = 0;
            continue;
        }
        
        if (U1STAbits.URXDA == 1) {
            receivedChars[i++] = U1RXREG;
            
            if (U1RXREG == 0x0D) {
                good = 0;
                for (i = 1; i < 7; i++) { //Because <LF> is the first character in receivedChars
                    if (receivedChars[i] == command[i-1]) {
                        good++;
                    }
                }
                
                if (good == 6) { //Command starts with $GPRMC command string we are looking for
                    commaNumber = 0;
                    counter = 0;
                    for (i = 0; i < 300; i++) {
                        if (receivedChars[i] == ',' || receivedChars[i] == '*') {
                            commaNumber++;
                            continue;
                        }
                        switch (commaNumber) {
                            case 0:
                                break;
                            case 1:
                                Time[counter++] = receivedChars[i];
                                break;
                            case 2:
                                counter = 0;
                                Status = receivedChars[i];
                                break;
                            case 3:
                                Latitude[counter++] = receivedChars[i];
                                break;
                            case 4:
                                counter = 0;
                                DirectionNS = receivedChars[i];
                                break;
                            case 5:
                                Longitude[counter++] = receivedChars[i];
                                break;
                            case 6:
                                counter = 0;
                                DirectionEW = receivedChars[i];
                                break;
                            case 7:
                                Speed[counter++] = receivedChars[i];
                                if (counter == 3) {
                                    counter = 0;
                                }
                                break;
                            case 8:
                                Heading[counter++] = receivedChars[i];
                                if (counter == 3) {
                                    counter = 0;
                                }
                                break;
                            case 9:
                                Date[counter++] = receivedChars[i];
                                break;
                            case 10:
                                if (counter == 6) {
                                    counter = 0;
                                }
                                Magnetic[counter++] = receivedChars[i];
                                break;
                            case 11:
                                MagneticEW = receivedChars[i];
                                break;
                            case 12:
                                Mode = receivedChars[i];
                                break;
                            case 13:
                                Checksum = receivedChars[i];
                                break;
                        }
                    }
                }
            }
        }
    }
    
    return 0;
}