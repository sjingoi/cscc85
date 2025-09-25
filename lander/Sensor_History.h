#pragma once

#define READINGS 200
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

extern SensorHistory sensor_history;

void UpdateSensorHistory();
double GetHistoricalVelocityX(int steps_back);
double GetHistoricalVelocityY(int steps_back);
double GetHistoricalPositionX(int steps_back);
double GetHistoricalPositionY(int steps_back);
double GetHistoricalAngle(int steps_back);
double GetHistoricalRangeDist(int steps_back);
double GetHistoricalSonar(int steps_back, int sonar_index);