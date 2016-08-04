/**
  ADC1 Generated Driver API Header File

  @Company
    Microchip Technology Inc.

  @File Name
    adc1.h

  @Summary
    This is the generated header file for the ADC1 driver using MPLAB® Code Configurator

  @Description
    This header file provides APIs for driver for ADC1.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC24FV32KA304
        Driver Version    :  0.5
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.24
        MPLAB 	          :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

#ifndef _ADC1_H
#define _ADC1_H

/**
  Section: Included Files
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C"
{

#endif

    /**
      Section: Data Types
     */

    /** ADC Channel Definition
 
     @Summary 
       Defines the channels available for conversion
 
     @Description
       This routine defines the channels that are available for the module to use.
 
     Remarks:
       None
     */
    typedef enum
    {
        ADC1_CHANNEL_AVDD = 0x1E,
        ADC1_CHANNEL_AVSS = 0x1D,
        ADC1_CHANNEL_UPPER_GUARDBAND_RAIL = 0x1C,
        ADC1_CHANNEL_LOWER_GUARDBAND_RAIL = 0x1B,
        ADC1_CHANNEL_VBG = 0x1A,
        ADC1_CHANNEL_CTMU = 0x11,
        ADC1_CHANNEL_CTMU_TEMPERATURE_SENSOR_INPUT = 0x10,
        ADC1_BATTCURRENT = 0x00,
        ADC1_MAX_CHANNEL_COUNT = 1
    } ADC1_CHANNEL;

    /**     
            ADC Channel Definition
     */
    typedef enum
    {
        DRV_ADC1_CHANNEL_AVDD = 0x1E,
        DRV_ADC1_CHANNEL_AVSS = 0x1D,
        DRV_ADC1_CHANNEL_UPPER_GUARDBAND_RAIL = 0x1C,
        DRV_ADC1_CHANNEL_LOWER_GUARDBAND_RAIL = 0x1B,
        DRV_ADC1_CHANNEL_VBG = 0x1A,
        DRV_ADC1_CHANNEL_CTMU = 0x11,
        DRV_ADC1_CHANNEL_CTMU_TEMPERATURE_SENSOR_INPUT = 0x10,
        DRV_ADC1_BATTCURRENT = 0x00,
        DRV_ADC1_MAX_CHANNEL_COUNT = 1
    } DRV_ADC1_CHANNEL;
    /**
      Section: Interface Routines
     */


    /**
      @Summary
        This function initializes ADC instance : 1

      @Description
        This routine initializes the ADC driver instance for : 1
        index, making it ready for clients to open and use it. It also initializes any
        internal data structures.
        This routine must be called before any other ADC routine is called. 

      @Preconditions
        None.

      @Param
        None.

      @Returns
        None.

      @Comment
    
 
      @Example
        <code>
            int conversion;
            ADC1_Initialize();
            ADC1_ChannelSelect(AN1_Channel);
            ADC1_Start();
            //Provide Delay
            for(int i=0;i <1000;i++)
            {
            }
            ADC1_Stop();
            while(!ADC1_IsConversionComplete())
            {
                ADC1_Tasks();   
            }
            conversion = ADC1_ConversionResultGet();
        </code>

     */

    void ADC1_Initialize(void);
    /**
        void DRV_ADC1_Initialize(void)
     */
    void DRV_ADC1_Initialize(void) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse ADC1_Initialize instead. ")));

    /**
      @Summary
        Starts sampling manually.

      @Description
        This routine is used to start the sampling manually.
 
      @Preconditions
        ADC1_Initializer function should have been called 
        before calling this function.

      @Param
        None.

      @Returns
        None.

      @Example
        Refer to ADC1_Initializer() for an example

     */

    void ADC1_Start(void);
    /**
        void DRV_ADC1_Start(void)
     */
    void DRV_ADC1_Start(void) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse ADC1_Start instead. ")));
    /**
      @Summary
        Stops sampling manually.

      @Description
        This routine is used to stop the sampling manually before conversion
        is triggered.
 
      @Preconditions
        ADC1_Initializer() function should have been 
        called before calling this function.

      @Param
        None.

      @Returns
        None.

      @Example
        Refer to ADC1_Initializer() for an example
     */

    void ADC1_Stop(void);
    /**
        void DRV_ADC1_Stop(void)
     */
    void DRV_ADC1_Stop(void) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse ADC1_Stop instead. ")));
    /**
      @Summary
        Gets the buffer loaded with conversion results.

      @Description
        This routine is used to get the analog to digital converted values in a
        buffer. This routine gets converted values from multiple channels.
 
      @Preconditions
        This routine returns the buffer containing the conversion values only after 
        the conversion is complete. Completion status conversion can be checked using 
        ADC1_IsConversionComplete() routine.
 
      @Param
        None.

      @Returns
        Returns the count of the buffer containing the conversion values.

      @Example
        <code>
            int count;
            //Initialize for channel scanning
            ADC1_Initializer();
            ADC1_Start();
            //Provide Delay
            for(int i=0;i <1000;i++)
            {
            }
            ADC1_Stop();
            while(!ADC1_IsConversionComplete())
            {
                count = ADC1_ConversionResultBufferGet();
            }

     */

    uint16_t ADC1_ConversionResultBufferGet(uint16_t *buffer);

    /**
        uint16_t DRV_ADC1_ConversionBufferGet(uint16_t *buffer)
     */
    uint16_t DRV_ADC1_ConversionBufferGet(uint16_t *buffer) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse ADC1_ConversionResultBufferGet instead. ")));
    /**
      @Summary
        Returns the ADC1 conversion value.

      @Description
        This routine is used to get the analog to digital converted value. This
        routine gets converted values from the channel specified.
 
      @Preconditions
        The channel required must be selected before calling this routine using
        ADC1_ChannelSelect(channel). This routine returns the 
        conversion value only after the conversion is complete. Completion status 
        conversion can be checked using ADC1_IsConversionComplete()
        routine.
   
      @Returns
        Returns the buffer containing the conversion value.

      @Param
        Buffer address
  
      @Example
        Refer to ADC1_Initializer}(); for an example
     */

    uint16_t ADC1_ConversionResultGet(void);
    /**
        uint16_t DRV_ADC1_ConversionResultGet(void)
     */
    uint16_t DRV_ADC1_ConversionGet(void) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse ADC1_ConversionResultGet instead. ")));
    /**
      @Summary
        Returns true when the conversion is completed

      @Description
        This routine is used to determine if conversion is completed. This routine
        returns the value of the DONE bit. When conversion is complete the routine
        returns 1. It returns 0 otherwise.
 
      @Preconditions
        ADC1_Initializer() function should have been 
        called before calling this function.
 
      @Returns
        Returns true if conversion is completed

      @Param
        None
  
      @Example
        Refer to ADC1_Initializer(); for an example
     */

    bool ADC1_IsConversionComplete(void);
    /**
        bool DRV_ADC1_IsConversionComplete(void)
     */
    bool DRV_ADC1_IsConversionComplete(void) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse ADC1_IsConversionComplete instead. ")));
    /**
      @Summary
        Allows selection of a channel for conversion

      @Description
        This routine is used to select desired channel for conversion.
  
      @Preconditions
        ADC1_Initializer() function should have been 
        called before calling this function.
 
      @Returns
        None

      @Param
        Pass in required channel from the ADC1_CHANNEL list
  
      @Example
        Refer to ADC1_Initializer(); for an example
 
     */

    void ADC1_ChannelSelect(ADC1_CHANNEL channel);
    /**
        void DRV_ADC1_ChannelSelect( DRV_ADC1_CHANNEL channel)
     */
    void DRV_ADC1_ChannelSelect(DRV_ADC1_CHANNEL channel) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse ADC1_ChannelSelect instead. ")));

    /**
      @Summary
        Polled implementation

      @Description
        This routine is used to implement the tasks for polled implementations.
  
      @Preconditions
        ADC1_Initializer() function should have been 
        called before calling this function.
 
      @Returns 
        None
 
      @Param
        None
 
      @Example
        Refer to ADC1_Initializer(); for an example
    
     */
    void ADC1_Tasks(void);
    /**
        void DRV_ADC1_Tasks(void)
     */
    void DRV_ADC1_Tasks(void) __attribute__((deprecated("\nThis will be removed in future MCC releases. \nUse ADC1_Tasks instead. ")));


#ifdef __cplusplus  // Provide C++ Compatibility

}

#endif

#endif //_ADC1_H

/**
 End of File
 */
