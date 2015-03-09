
// Local magnetic declination
// I use this web : http://www.ngdc.noaa.gov/geomagmodels/Declination.jsp
//#define MAGNETIC_DECLINATION -6.0    // not used now -> magnetic bearing

void Compass_Heading()
{
  float MAG_X;
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  // Tilt compensated Magnetic filed X:
  MAG_X = Mgnt_Vector[0]*cos_pitch+Mgnt_Vector[1]*sin_roll*sin_pitch+Mgnt_Vector[2]*cos_roll*sin_pitch;
  // Tilt compensated Magnetic filed Y:
  MAG_Y = Mgnt_Vector[1]*cos_roll-Mgnt_Vector[2]*sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-MAG_Y,MAG_X);
}
