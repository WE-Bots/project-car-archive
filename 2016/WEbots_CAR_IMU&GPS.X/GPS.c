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
int commaNumber, counter; //Keeps track for parsing the data
char receivedChars[300]; //Keeps track of chars brought in
char command[7] = "$GPRMC"; //Command string
char Time[9], Status, Latitude[7], DirectionNS, //Parsed data
        Longitude[8], DirectionEW, Speed[3], Heading[3],
        Date[6], Magnetic[3], MagneticEW, Mode,
        Checksum;

void init() {
    RPINR18 = 0x003E; //Sets pin 48 as UART receiver
    RPINR26 = 0x0031; //Sets pin 61 for CAN receiver
    RPOR4 = 0x0F00; //Sets pin 60 for CAN transmitter
    
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
    init(); //Runs the setup function
    
    while(1) {
        if (U1STAbits.OERR == 1) { //Checks if buffer is overflowing
            U1STAbits.OERR = 0;
            continue;
        }
        
        if (U1STAbits.URXDA == 1) { //If byte is passed
            receivedChars[i++] = U1RXREG; //Put byte into array
            
            if (U1RXREG == 0x0D) { //If byte is end of transmission
                good = 0; //Uses this for a check
                
                for (i = 1; i < 7; i++) { //Because <LF> is the first character in receivedChars
                    if (receivedChars[i] == command[i-1]) {
                        good++;
                    }
                }
                
                if (good == 6) { //Command starts with $GPRMC command string we are looking for
                    commaNumber = 0; //Counter
                    counter = 0; //Counter
                    
                    for (i = 0; i < 300; i++) { //Loops through array
                        if (receivedChars[i] == ',' || receivedChars[i] == '*') { //Keeps track of the breaks in data
                            commaNumber++;
                            continue; //Skips the parsing code
                        }
                        
                        switch (commaNumber) { //Depending on which break in data fill a different array
                            case 0:
                                break;
                            case 1:
                                Time[counter++] = receivedChars[i]; //If after first comma fill Time array
                                break;
                            case 2:
                                counter = 0; //Clear array counter
                                Status = receivedChars[i]; //Fill Status
                                break;
                            case 3:
                                Latitude[counter++] = receivedChars[i]; //Fill latitude array
                                break;
                            case 4:
                                counter = 0; //Resets counter
                                DirectionNS = receivedChars[i]; //Sets N/S direction
                                break;
                            case 5:
                                Longitude[counter++] = receivedChars[i]; //Fills longitude array
                                break;
                            case 6:
                                counter = 0; //Clear counter
                                DirectionEW = receivedChars[i]; //Sets E/W direction
                                break;
                            case 7:
                                Speed[counter++] = receivedChars[i]; //Fills speed array
                                if (counter == 3) { //Checks counter to prevent more checks later on
                                    counter = 0;
                                }
                                break;
                            case 8:
                                Heading[counter++] = receivedChars[i]; //Fills heading array
                                if (counter == 3) { //Checks counter to prevent more checks later on
                                    counter = 0;
                                }
                                break;
                            case 9:
                                Date[counter++] = receivedChars[i]; //Fills date array
                                break;
                            case 10:
                                if (counter == 6) { //Clears counter after date
                                    counter = 0;
                                }
                                Magnetic[counter++] = receivedChars[i]; //Fills magnetic drift array
                                break;
                            case 11:
                                MagneticEW = receivedChars[i]; //Fills the direction of magnetic drift
                                break;
                            case 12:
                                Mode = receivedChars[i]; //Fills mode
                                break;
                            case 13:
                                Checksum = receivedChars[i]; //Fills checksum
                                break;
                        }
                    }
                }
            }
        }
    }
    return 0;
}