//
//  quaternion.cpp
//  
//
//  Created by Darryl Murray on 2015-08-29.
//
//

#include "quaternion.h"

// ******** Constructors ******** //
quaternion::quaternion() { // No rotation - default
  w = 1;
  x = 0;
  y = 0;
  z = 0;
}
quaternion::quaternion(vector3D<double> start, vector3D<double> end) {
  // Sourced from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm
  // Now sourced from: http://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
  //THIS CAN BE OPTIMIZED! http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
  
  vector3D<double> axis = cross(end, start);
  
  x = axis.x;
  y = axis.y;
  z = axis.z;
  w = sqrt(pow(::magnitude(start), 2) * pow(::magnitude(end), 2)) + dot(start, end);
  normalize();
}
quaternion::quaternion(quaternionStorage input) {
  w = input.w;
  x = input.x;
  y = input.y;
  z = input.z;
}


// ******** Private Functions ******** //
quaternion quaternion::conjugate() {
  return quaternion(quaternionStorage{w, -x, -y, -z});
}
double quaternion::magnitude() {
  return sqrt(pow(w,2) + pow(x,2) + pow(y,2) + pow(z,2));
}
void quaternion::normalize() {
  double mag = magnitude();
  w /= mag;
  x /= mag;
  y /= mag;
  z /= mag;
}


// ******** Export functions ******** //
quaternionStorage quaternion::exportData() {
  return quaternionStorage {w, x, y, z};
}
vector3D<double> quaternion::exportVector3D() {
  return vector3D<double> {x, y, z};
}


// ******** Utility functions ******** //
vector3D<double> quaternion::rotateVector(vector3D<double> vector) { // Sourced from: http://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
  quaternion pureQuat (quaternionStorage{0, vector.x, vector.y, vector.z});
  quaternion resultQuat = *this * pureQuat * this->conjugate();
  vector3D<double> resultVec = resultQuat.exportVector3D();
  return vector3D<double> {resultVec.x, resultVec.y, resultVec.z};
}


// ******** Operator overloads ******** //
quaternion quaternion::operator*(const quaternion& other) { // Sourced from: http://www.cprogramming.com/tutorial/3d/quaternions.html
  quaternion result;
  result.w = (w * other.w - x * other.x - y * other.y - z * other.z);
  result.x = (w * other.x + x * other.w + y * other.z - z * other.y);
  result.y = (w * other.y - x * other.z + y * other.w + z * other.x);
  result.z = (w * other.z + x * other.y - y * other.x + z * other.w);
  return result;
}
