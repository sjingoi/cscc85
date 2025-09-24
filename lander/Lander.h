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

enum SensorMapping {
    VELOCITY_X = 1,
    VELOCITY_Y = 2,
    POSITION_X = 3,
    POSITION_Y = 4,
    ANGLE = 5,
    SONAR = 6, // There is no way to derive this one so return mode 2 if it isn't functioning
    RANGEDIST = 7 // This never fails so return mode 0
};

// --- TRACKING SENSOR STATUS ---
// structure for tracking sensor status
typedef struct {
    int velocity_x_ok;      // Horizontal velocity sensor
    int velocity_y_ok;      // Vertical velocity sensor
    int position_x_ok;      // Horizontal position sensor
    int position_y_ok;      // Vertical position sensor
    int angle_ok;          // Angle sensor
    int sonar_ok;          // Sonar sensor
} SensorStatus;

#define READINGS 50

// --- SENSOR HISTORY ---
// structure for sensor history
typedef struct {
    int current_index;
    int count; // number of valid readings

    double velocity_x_hist[READINGS];
    double velocity_y_hist[READINGS];
    double position_x_hist[READINGS];
    double position_y_hist[READINGS];
    double angle_hist[READINGS];
    double range_dist_hist[READINGS];
    double sonar_hist[READINGS][36]; // 36 sonar readings per time step
} SensorHistory;
