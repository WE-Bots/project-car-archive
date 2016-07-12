/**
  Debug LED Header File 

  @Company
    WE Bots

  @File Name
    debug_LED.h

  @Summary
    This is the header file for the debugging LED on the battery monitoring and 
    power distribution board

  @Description
    This header file provides APIs for the debug LED.
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.24
        MPLAB 	          :  MPLAB X v2.35 or v3.00
    
 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include <stdbool.h>

// TODO Insert appropriate #include <>

// Declarations
#define DEBUG_LED_ENABLE 1
#define DEBUG_LED_DISABLE 0
#define DEBUG_LED_STOP_BITS 3
#define DEBUG_LED_DATA_BITS 4

#define DEBUG_LED_NO_MESSAGE 0 
#define DEBUG_LED_MESSAGE_1 1
#define DEBUG_LED_MESSAGE_2 2
#define DEBUG_LED_MESSAGE_3 3
#define DEBUG_LED_MESSAGE_4 4
#define DEBUG_LED_MESSAGE_5 5
#define DEBUG_LED_MESSAGE_6 6
#define DEBUG_LED_MESSAGE_7 7

// Comment a function and leverage automatic documentation with slash star star
/**
 void
  
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

void Debug_Message_Initialize(void);
/**  
    @Summary
 Initializes the debug LED functionality

    @Description
 Initializes the debug LED functionality
 
    @Precondition
 None
 
    @Parameters
 None
 
    @Returns
 None           

    @Example
    <code>
 Debug_Message_Initialize();
    </code>

    @Remarks
 None
 */

void Enable_Debug_Message(void);
/**  
    @Summary
 Enables the debug LED

    @Description
 Enables the debug LED
 
    @Precondition
 None
 
    @Parameters
 None
 
    @Returns
 None           

    @Example
    <code>
 Enable_Debug_Message();
    </code>

    @Remarks
 None
 */

void Disable_Debug_Message(void);
/**  
    @Summary
  Disables the debug LED

    @Description
  Disables the debug LED
 
    @Precondition
  None
 
    @Parameters
 None
 
    @Returns
 None           
 
    @Example
    <code>
 Disable_Debug_Message();
    </code>

    @Remarks
 None
 */

void Set_Debug_Message(uint8_t message);
/**  
    @Summary
 Sets the desired debug blink message

    @Description
 This routine Sets the desired debug blink message to be displayed
 
    @Precondition
 None
 
    @Parameters
 Debug message as defined in the header file
 
    @Returns
 None     

    @Example
    <code>
 Set_Debug_Message( DEBUG_LED_MESSAGE_0);
    </code>

    @Remarks
 None
 */

void Debug_Message_Update(void);
/**  
    @Summary
 Handles flashing the debug LED

    @Description
 Handles flashing the debug LED
 
    @Precondition
 This should only be used within a timer ISR that is set to generate an 
 interrupt at a fixed interval.
 
 This ISR should call this function every time it gets called
 
    @Parameters
 None
 
    @Returns
 None           

    @Example
    <code>
 void TMR5_CallBack(void) 
 {
    Debug_Message_Update();    
 }
    </code>

    @Remarks
 None
 */

#ifdef	__cplusplus
extern "C"
{
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

