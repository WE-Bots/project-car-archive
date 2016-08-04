/* 
 * File:   quaternion.c
 * Author: Michael Buchel
 *
 * quaternion file from Darryl in C
 * 
 * Created on June 24, 2016, 3:35 PM
 */

#include "quaternion.h"

//Init with only address
void init1(quaternion* address) {
    address->w = 1;
    address->x = 0;
    address->y = 0;
    address->z = 0;
}

//Init with a value
void init2(quaternion* address, double a, double b, double c, double d) {
    address->w = a;
    address->x = b;
    address->y = c;
    address->z = d;
}

//Init with another quaternion
void init3(quaternion* address, quaternion other) {
    address->w = other.w;
    address->x = other.x;
    address->y = other.y;
    address->z = other.z;
}

//Init with 2 vectors
void init4(quaternion* address, vector start, vector end) {
    //Optimized (not fully) using
    //http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
    vector axis = cross(start, end);
    
    address->x = axis.x;
    address->y = axis.y;
    address->z = axis.z;
    address->w = 1.0f + dot(start, end);
    normalizeQuat(address);
}

//Conjugate of the quaternion
quaternion conjugateQuat(quaternion start) {
    quaternion temp;
    init2(&temp, start.w, -start.x, -start.y, -start.z);
    return temp;
}

//Magnitude of a quaternion
double magnitudeQuat(quaternion address) {
    return sqrt(pow(address.w, 2)+pow(address.x, 2)+pow(address.y, 2)+pow(address.z, 2));
}

//Normalizes the quaternion
void normalizeQuat(quaternion* address) {
    double mag = magnitudeQuat(*address);
    
    address->w /= mag;
    address->x /= mag;
    address->y /= mag;
    address->z /= mag;
}

//Exports to vector form
vector exportVector(quaternion start) {
    vector temp;
    
    temp.x = start.x;
    temp.y = start.y;
    temp.z = start.z;
    
    return temp;
}

//Rotates vector
//http://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
vector rotateVector(quaternion axis, vector start) {
    quaternion pureQuat;
    
    pureQuat.w = 0;
    pureQuat.x = start.x;
    pureQuat.y = start.y;
    pureQuat.z = start.z;
    
    quaternion resultQuat = multiplyQuat(multiplyQuat(axis, pureQuat), conjugateQuat(axis));
    vector result = exportVector(resultQuat);
    return result;
}

//Euler representation
//https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
vector getEulerRepresentation(quaternion start) {
    vector temp;
    
    temp.x = atan2(2*(start.w * start.x + start.y * start.z), 1-2*(pow(start.x, 2)+pow(start.y, 2)));
    temp.y = asin(2*(start.w * start.y - start.z * start.x));
    temp.z = atan2(2*(start.w * start.z + start.x * start.y), 1-2*(pow(start.y, 2)+pow(start.z, 2)));
    
    return temp;
}

//Multiplying quaternions
//http://www.cprogramming.com/tutorial/3d/quaternions.html
quaternion multiplyQuat(quaternion start, quaternion end) {
    quaternion result;
    
    result.w = start.w * end.w - start.x * end.x - start.y * end.y - start.z * end.z;
    result.x = start.w * end.x + start.x * end.w + start.y * end.z - start.z * end.y;
    result.y = start.w * end.y - start.x * end.z + start.y * end.w + start.z * end.x;
    result.z = start.w * end.z + start.x * end.y - start.y * end.x + start.z * end.w;
    
    return result;
}