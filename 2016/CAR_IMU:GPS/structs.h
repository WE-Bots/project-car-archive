//
//  structs.h
//  
//
//  Created by Darryl Murray on 2015-08-29.
//
//

#ifndef ____structs__
#define ____structs__

#include <stdio.h>
#include "Arduino.h"


struct balance { // This struct contains the balance offset to replace the accelerometer-based offset
  byte dataFormat;
  int16_t xBalanceVal;
  int16_t yBalanceVal;
  int16_t zBalanceVal;
  
  // This saves the range of the axis
  int16_t xRange;
  int16_t yRange;
  int16_t zRange;
};


#endif /* defined(____structs__) */
