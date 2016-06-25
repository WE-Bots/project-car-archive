//
//  vector3d.h
//
//
//  Created by Darryl Murray on 2015-08-29.
//
//

#ifndef ____vector3d__
#define ____vector3d__

#include <stdio.h>
#include <cmath>
#include "Arduino.h"

//TODO: Figure out how this template function struct works
template <typename T = double>
struct vector3D {
  T x;
  T y;
  T z;
  template <typename U>
  operator vector3D<U>() { // Interestingly, this operator isn't stored when the struct is saved. Either way, now the internal format of vector3D is completely irrelevant (go ahead, use vector3D<> with reckless abandon!), only change it when necessary.
    return vector3D<U> {x, y, z};
  }
  vector3D<double> operator+(const vector3D<double> &that) {
    return vector3D<double> {double(x) + that.x,
                             double(y) + that.y,
                             double(z) + that.z};
  }
  vector3D<double> operator-(const vector3D<double> &that) {
    return vector3D<double> {double(x) - that.x,
                             double(y) - that.y,
                             double(z) - that.z};
  }
  vector3D<double> operator*(const double &that) {
    return vector3D<double> {double(x) * that,
                             double(y) * that,
                             double(z) * that};
  }
  vector3D<double> operator/(const double &that) {
    return vector3D<double> {double(x) / that,
                             double(y) / that,
                             double(z) / that};
  }
};

typedef vector3D<int16_t> accelOutput;

// ******** Vector functions ******** //
//PE: I assume this calculates the magnitude mathematically using components from the provided vector
double magnitude(vector3D<double> vector);  
//PE: Not entirely sure, seems like this function could either mean "find an orthoganol vector" or "straighten vector along axis"
vector3D<double> normalize(vector3D<double> vector);  
//PE: dot?  Calculate the dot product?
double dot(vector3D<double> start, vector3D<double> end); 
//PE: This must be for calculating cross product of two vectors then 
vector3D<double> cross(vector3D<double> start, vector3D<double> end);  


#endif /* defined(____vector3d__) */
