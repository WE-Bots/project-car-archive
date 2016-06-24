/* 
 * File:   structs.h
 * Author: Michael Buchel
 *
 * Just for balance offset typedefs no functions
 * 
 * Created on June 24, 2016, 5:05 PM
 */

#ifndef STRUCTS_H
#define	STRUCTS_H

typedef short int int16_t;
typedef unsigned char byte;

//Struct for the balance offset to replace the accelerometer based offset
struct balanceOffset {
    byte dataFormat;
    int16_t xBalanceVal, yBalanceVal, zBalanceVal;
    
    //Range of axis
    int16_t xRange, yRange, zRange;
};

typedef struct balanceOffset balance;

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* STRUCTS_H */

