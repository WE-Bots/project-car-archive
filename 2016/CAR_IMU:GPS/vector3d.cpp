//
//  vector3d.cpp
//  
//
//  Created by Darryl Murray on 2015-08-29.
//
//

#include "vector3d.h"


// ******** Vector functions ******** //
double magnitude(vector3D<double> vector) {
  return sqrt(abs(pow(vector.x, 2) + pow(vector.y, 2) + pow(vector.z, 2)));
}
vector3D<double> normalize(vector3D<double> vector) {
  double mag = magnitude(vector);
  return vector3D<double>{double(vector.x)/mag, double(vector.y)/mag, double(vector.z)/mag};
}
double dot(vector3D<double> start, vector3D<double> end) {
  return double(start.x) * end.x + double(start.y) * end.y + double(start.z) * end.z;
}
vector3D<double> cross(vector3D<double> start, vector3D<double> end) {
  vector3D<double> result;
  result.x = double(start.y) * end.z - double(start.z) * end.y;
  result.y = -(double(start.x) * end.z - double(start.z) * end.x);
  result.z = double(start.x) * end.y - double(start.y) * end.x;
  return result;
}
