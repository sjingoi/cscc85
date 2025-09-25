#pragma once

#include "Lander.h"
#include "Sensor_History.h"
#include <vector>

// Structure for sensor value return from wrapper function
typedef struct {
    double value;
    int mode; // 0 for sensor reading, 1 for derived, 2 for not possible to derive 
} SensorValue;

bool SensorInExclusionList(SensorMapping sensor, std::vector<int> exclusion_list);

SensorValue GetSensorValue(SensorMapping sensor, std::vector<int> exclusion_list, SensorStatus sensor_status, SensorHistory sensor_history);
SensorValue GetVelocityX(std::vector<int> exclusion_list, SensorStatus sensor_status, SensorHistory sensor_history);
SensorValue GetVelocityY(std::vector<int> exclusion_list, SensorStatus sensor_status, SensorHistory sensor_history);
SensorValue GetPositionX(std::vector<int> exclusion_list, SensorStatus sensor_status, SensorHistory sensor_history);
SensorValue GetPositionY(std::vector<int> exclusion_list, SensorStatus sensor_status, SensorHistory sensor_history);
SensorValue GetAngle(std::vector<int> exclusion_list, SensorStatus sensor_status, SensorHistory sensor_history);
SensorValue GetSonar(std::vector<int> exclusion_list, SensorStatus sensor_status, SensorHistory sensor_history);
SensorValue GetRangeDist(std::vector<int> exclusion_list, SensorStatus sensor_status, SensorHistory sensor_history);
