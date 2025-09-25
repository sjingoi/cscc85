#include <math.h>

#include "Lander.h"
#include "Lander_Control.h"

void orientLander(double angle_rad, const struct LanderState *ls) {
  double epsilon = 1;
  double target_angle = fmod((angle_rad / (2.0 * PI)) * 360.0, 360.0); // Convert to degrees
  double angle_delta = fmod((ls->angle - target_angle + 540.0), 360.0) - 180.0;

  if (fabs(angle_delta) > epsilon) {
    Rotate(-1 * angle_delta);
  }
}

void thrustAngle(double angle_rad, double acceleration, const struct LanderState *ls) {
  angle_rad += (MT_OK) ? 0 : (RT_OK) ? PI / 2.0 : PI / -2.0;
  double epsilon = 2.0;
  double target_angle = fmod((angle_rad / (2.0 * PI)) * 360.0, 360.0); // Convert to degrees
  double angle_delta = fmod((ls->angle - target_angle + 540.0), 360.0) - 180.0;

  if (fabs(angle_delta) > epsilon) {
    Rotate(-1 * angle_delta);
  }
  
  if (fabs(angle_delta) > 4) return;
  
  printf("Burning at acceleration %f\n", acceleration);
  if (MT_OK) {
    Main_Thruster(acceleration / 35.0);
  } else if (RT_OK) {
    Right_Thruster(acceleration / 25.0);
  } else {
    Left_Thruster(acceleration / 25.0); 
  }
}

void thrustVector(double acc_x, double acc_y, const struct LanderState *ls) {
  double angle = atan2(acc_y, acc_x);
  double mag = sqrt(acc_y*acc_y + acc_x*acc_x);
  thrustAngle(angle + PI / 2, mag, ls);
}