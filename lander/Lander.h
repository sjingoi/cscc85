#pragma once

#include "stdio.h"

/**
 * Struct that holds the lander state so that it can be easily referenced
 */
struct LanderState {
 // Physical properties of the craft
 double altitude; // Vertical distance from the ground
 double pos_x, pos_y;
 double vel_x, vel_y;
 double angle; // Current spacecraft angle in degrees

 // Calculated properies of the craft
 double max_acc; // Maximum acceleration based on available thrusters
 double landing_acc; // Vertical acceleration required to stop the craft at the ground based on the current falling rate
};

enum LandingPhase {
  GAIN_ALTITUDE,
  GO_ABOVE_LANDING_SITE,
  FALLING,
  LANDING_BURN,
  LANDED
};