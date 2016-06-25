//
//  quaternion.h
//  
//
//  Created by Darryl Murray on 2015-08-29.
//
//

#ifndef ____quaternion__
#define ____quaternion__

#include <stdio.h>
#include <cmath>
#include "vector3d.h"

struct quaternionStorage { // DO NOT USE! ONLY FOR STORAGE IN EEPROM!
  double w;
  double x;
  double y;
  double z;
};

class quaternion {
private:
  // ******** Variables ******** //
  double w;
  double x;
  double y;
  double z;
  
  
  // ******** Private Functions ******** //
  quaternion conjugate();
  double magnitude();
  void normalize();
  
public:
  // ******** Constructors ******** //
  quaternion();
  quaternion(vector3D<double> start, vector3D<double> end);
  quaternion(quaternionStorage input);
  
  // ******** Export functions ******** //
  quaternionStorage exportData();
  vector3D<double> exportVector3D();
  
  // ******** Utility functions ******** //
  vector3D<double> rotateVector(vector3D<double> vector);
  vector3D<double> getEulerRepresentation();
  
  // ******** Operator overloads ******** //
  quaternion operator*(const quaternion& other);
};

#endif /* defined(____quaternion__) */
