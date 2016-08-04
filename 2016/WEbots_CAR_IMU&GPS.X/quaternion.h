/* 
 * File:   quaternion.h
 * Author: Michael Buchel
 *
 * quaternion header from Darryl in C
 * 
 * Created on June 24, 2016, 3:35 PM
 */

#ifndef QUATERNION_H
#define	QUATERNION_H

#include "vector3d.h"

struct quaternionStorage {
    double w, x, y, z;
};

typedef struct quaternionStorage quaternion;

//Constructor
void init1(quaternion* address);
void init2(quaternion* address, double a, double b, double c, double d);
void init3(quaternion* address, quaternion other);
void init4(quaternion* address, vector start, vector end);

//Helpful functions
quaternion conjugateQuat(quaternion start);
double magnitudeQuat(quaternion address);
void normalizeQuat(quaternion* address);

//Export function
vector exportVector(quaternion start);

//Utility functions
vector rotateVector(quaternion axis, vector start);
vector getEulerRepresentation(quaternion start);

//If this was C++ this would be an overloaded operator
quaternion multiplyQuat(quaternion start, quaternion end);

#ifdef	__cplusplus
extern "C" {
#endif


#ifdef	__cplusplus
}
#endif

#endif	/* QUATERNION_H */