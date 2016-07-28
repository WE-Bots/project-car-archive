/* 
 * File:   PWM.h
 * Author: Kevin
 *
 * Created on July 14, 2016, 5:01 PM
 */

#ifndef PWM_H
#define	PWM_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <p33Exxxx.h>

    void PWMInit();
    void PWM1SetDutyCycleUS(int us);
    void PWM2SetDutyCycleUS(int us);
    void PWM1SetDutyCycle(int dutyCycle);
    void PWM2SetDutyCycle(int dutyCycle);

#ifdef	__cplusplus
}
#endif

#endif	/* PWM_H */

