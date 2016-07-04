/*
 * File:   vector3d.c
 * Author: Michael Buchel
 *
 * vector3d code converted to C
 * 
 * Created on June 24, 2016, 4:20 AM
 */

#include "vector3d.h"

//Adding 2 vectors together
vector add(vector start, vector end) {
    vector result;
    result.x = start.x + end.x;
    result.y = start.y + end.y;
    result.z = start.z + end.z;
    return result;
}

//Subtracting 2 vectors
vector sub(vector start, vector end) {
    vector result;
    result.x = start.x - end.x;
    result.y = start.y - end.y;
    result.z = start.z - end.z;
    return result;
}

//Multiplying a vector by a scalar
vector multiply(vector start, double scalar) {
    vector result;
    result.x = start.x * scalar;
    result.y = start.y * scalar;
    result.z = start.z * scalar;
    return result;
}

//Dividing a vector by a scalar
vector divide(vector start, double scalar) {
    vector result;
    result.x = start.x / scalar;
    result.y = start.y / scalar;
    result.z = start.z / scalar;
    return result;
}

//Magnitude of a vector
double magnitude(vector start) {
    return sqrt(pow(start.x, 2) + pow(start.y, 2) + pow(start.z, 2));
}

//Normalizes the vector
vector normalize(vector start) {
    double mag = magnitude(start);
    vector result;
    result.x = start.x / mag;
    result.y = start.y / mag;
    result.z = start.z / mag;
    return result;
}

//Dot product of 2 vectors
double dot(vector start, vector end) {
    return (start.x * end.x + start.y * end.y + start.z * end.z);
}

//Cross product of 2 vectors
vector cross(vector start, vector end) {
    vector result;
    result.x = start.y * end.z - start.z * end.y;
    result.y = start.z * end.x - start.x * end.z;
    result.z = start.x * end.y - start.y * end.x;
    return result;
}