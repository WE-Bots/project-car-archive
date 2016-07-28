 /* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef USERVARIABLES_H
#define	USERVARIABLES_H


#include <xc.h> // include processor files - each processor file is guarded.  
#include <p33Exxxx.h>

extern volatile int FLcaptime;
extern volatile int FLcaptime;
extern volatile int FRcaptime;
extern volatile int BLcaptime;
extern volatile int BRcaptime;

extern volatile unsigned long FLcount;
extern volatile unsigned long FRcount;
extern volatile unsigned long BLcount;
extern volatile unsigned long BRcount;

extern int motorDuty;
extern int servoDuty;

extern float desMotor;
extern float desServo;

extern char EStopRemote;



// TODO Insert appropriate #include <>

// TODO Insert C++ class definitions if appropriate

// TODO Insert declarations

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* USERVARIABLES_H */

