/**
  Debug LED Driver Source File 

  @Company
    WE Bots

  @File Name
    debug_LED.h

  @Summary
    This is the driver file for the debugging LED on the battery monitoring and 
    power distribution board

  @Description
    This header file provides APIs for the debug LED.
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.24
        MPLAB 	          :  MPLAB X v2.35 or v3.00
    
 */

/**
  Section: Included Files
 */
#include "xc.h"
#include "debug_LED.h"
#include "pin_manager.h"
#include <math.h>

/**
  Section: Data Type Definitions
 */

/** Debug Message Instance Object

  @Summary
 Defines the object required for the maintainence of the debug message instance.

  @Description
 This defines the object required to use the debug message. It tracks if the 
 debug message is enabled and what message to display.

  @Remarks
 None
 */
typedef struct _DEBUG_LED_OBJ_STRUCT {
    bool debug_status; // Is the display message enable or disabled
    uint8_t debug_message; // Message currently being displayed
    uint8_t next_debug_message; // The next message to be displayed
    uint8_t current_bit; // The current bit being displayed
    bool display_bit; // Is the current being displayed or is it an off cycle
}
DEBUG_LED_OBJECT;

static DEBUG_LED_OBJECT debug_obj;

/**
  Section: Driver Interface
 */

void Debug_Message_Initialize(void) {
    debug_obj.debug_status = DEBUG_LED_ENABLE;
    debug_obj.debug_message = DEBUG_LED_NO_MESSAGE;
    debug_obj.next_debug_message = debug_obj.debug_message;
    debug_obj.current_bit = 0;
    debug_obj.display_bit = 1;
}

void Enable_Debug_Message(void) {
    debug_obj.debug_status = DEBUG_LED_ENABLE;
}

void Disable_Debug_Message(void) {
    debug_obj.debug_status = DEBUG_LED_DISABLE;
}

void Set_Debug_Message(uint8_t message) {
    // Ensure that a valid message is provided
    if ((message >= 0) && (message < pow(2, DEBUG_LED_DATA_BITS))) {
        debug_obj.next_debug_message = message;
    }
}

void Debug_Message_Update(void) {
    // First check if the debug LED is enabled 
    if (debug_obj.debug_status == DEBUG_LED_ENABLE) {
        // check if the bit is being displayed this cycle
        if (debug_obj.display_bit == 1) {
            // Stop bit handling
            //  Check if the stop bit is being displayed
            //  NOte: The stop bit isn't displayed between the stop bits
            if (debug_obj.current_bit >= DEBUG_LED_DATA_BITS) {
                LEDInd_SetHigh();

                debug_obj.current_bit++;

                // check if the last stop bit has been displayed
                if (debug_obj.current_bit == (DEBUG_LED_DATA_BITS + DEBUG_LED_STOP_BITS)) {
                    debug_obj.current_bit = 0;
                    debug_obj.display_bit = 0;
                }
                return;
            }

            // The next cycle will be an off cycle
            debug_obj.display_bit = 0;

            // Update the message at the beginning of the cycle
            if (debug_obj.current_bit == 0) {
                debug_obj.debug_message = debug_obj.next_debug_message;
            }

            // Set the next bit of the message
            if ((debug_obj.debug_message >> debug_obj.current_bit) & 0x01) {
                LEDInd_SetHigh();
            } else {
                LEDInd_SetLow();
            }

            // Increment the counter
            debug_obj.current_bit++;
        } else {
            LEDInd_SetLow();
            // display the bit on the next cycle
            debug_obj.display_bit = 1;
        }
    } else {
        LEDInd_SetLow();
    }
}

