#pragma once

#include "Sensor_History.h"

void UpdateSensorStatus(SensorStatus *sensor_status, SensorHistory *sensor_history, double sonar[36]);
void PrintSensorStatus(SensorStatus sensor_status);