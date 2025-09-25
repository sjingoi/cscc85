#pragma once
#include "Lander.h"

extern SensorHistory sensor_history;

void UpdateSensorHistory(SensorStatus sensor_status);
double GetHistoricalVelocityX(int steps_back);
double GetHistoricalVelocityY(int steps_back);
double GetHistoricalPositionX(int steps_back);
double GetHistoricalPositionY(int steps_back);
double GetHistoricalAngle(int steps_back);
double GetHistoricalRangeDist(int steps_back);
double GetHistoricalSonar(int steps_back, int sonar_index);