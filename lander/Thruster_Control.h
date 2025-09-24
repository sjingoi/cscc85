#pragma once

#include "Lander.h"

/**
 * Function that takes in an acceleration vector [acc_x acc_y] and accelerates in that direction.
 * The acceleration magnitude will be capped to the max acceleration of the craft.
 * This will rotate the spacecraft, so do not call this at the same time as other rotation 
 * functions.
 * 
 * @param acc_x - the x component of acceleration
 * @param acc_y - the y component of acceleration
 * @param ls - the lander's state
 */
void thrustVector(double acc_x, double acc_y, const struct LanderState *ls);

/**
 * Function that takes in an acceleration magnitude acceleration and direction angle_rad and 
 * accelerates in that direction.
 * The acceleration magnitude will be capped to the max acceleration of the craft.
 * This will rotate the spacecraft, so do not call this at the same time as other rotation 
 * functions.
 * 
 * @param angle_rad - the acceleration direction in radians
 * @param acceleration - the magnitude of acceleration
 * @param ls - the lander's state
 */
void thrustAngle(double angle_rad, double acceleration, const struct LanderState *ls);

/**
 * Function that takes in an angle angle_rad and orients the lander in that direction
 * 
 * @param angle_rad - the direction to point the lander in radians
 * @param ls - the lander's state
 */
void orientLander(double angle_rad, const struct LanderState *ls);