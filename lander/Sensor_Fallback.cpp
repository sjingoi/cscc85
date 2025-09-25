#include "Sensor_Fallback.h"
#include "Sensor_History.h"
#include "Lander.h"
#include "Lander_Control.h"
#include <vector>

// Helper function to check if a sensor is in the exclusion list
bool SensorInExclusionList(SensorMapping sensor, std::vector<int> exclusion_list) {
    for (int i = 0; i < exclusion_list.size(); i++) {
      if (exclusion_list[i] == sensor) {
        return true;
      }
    }
    return false;
  }

// Functions for getting sensor values even during failures
// We need to pass the exclusion list to ensure no cycles are created (we take away sensors and see if we can continue deriving values)
SensorValue GetSensorValue(SensorMapping sensor, std::vector<int> exclusion_list, SensorStatus sensor_status, const SensorHistory *sensor_history) {
    switch (sensor) {
        // Velocity X
        case VELOCITY_X:
            return GetVelocityX(exclusion_list, sensor_status, sensor_history);
        // Velocity Y
        case VELOCITY_Y:
            return GetVelocityY(exclusion_list, sensor_status, sensor_history);
        // Position X
        case POSITION_X:
            return GetPositionX(exclusion_list, sensor_status, sensor_history);
        // Position Y
        case POSITION_Y:
            return GetPositionY(exclusion_list, sensor_status, sensor_history);
        // Angle
        case ANGLE:
            return GetAngle(exclusion_list, sensor_status, sensor_history);
        // Sonar
        case SONAR:
            return GetSonar(exclusion_list, sensor_status, sensor_history);
        // RangeDist
        case RANGEDIST:
            return GetRangeDist(exclusion_list, sensor_status, sensor_history);
        default:
            SensorValue default_sensor_value = {0.0, 2};
            return default_sensor_value;
    }
}

SensorValue GetVelocityX(std::vector<int> exclusion_list, SensorStatus sensor_status, const SensorHistory *sensor_history) {
    SensorValue sensor_value;
    if (sensor_status.velocity_x_ok && !SensorInExclusionList(VELOCITY_X, exclusion_list)) {
        sensor_value.value = Velocity_X();
        sensor_value.mode = 0;
        return sensor_value;
    }

    // Position X still functioning AND not excluded AND position_x is derivable
    exclusion_list.push_back(VELOCITY_X);
    SensorValue position_x_sensor_value = GetSensorValue(POSITION_X, exclusion_list, sensor_status, sensor_history);
    if (position_x_sensor_value.mode != 2) {
        sensor_value.value = (sensor_history->position_x_hist[sensor_history->current_index] - sensor_history->position_x_hist[(sensor_history->current_index + 1) % READINGS]) / (READINGS/40);
        sensor_value.mode = 1;
        return sensor_value;
    }

    return sensor_value;
}

SensorValue GetVelocityY(std::vector<int> exclusion_list, SensorStatus sensor_status, const SensorHistory *sensor_history) {
    SensorValue sensor_value;
    if (sensor_status.velocity_y_ok && !SensorInExclusionList(VELOCITY_Y, exclusion_list)) {
        sensor_value.value = Velocity_Y();
        sensor_value.mode = 0;
        return sensor_value;
    }

    // Position Y still function AND not excluded AND position_y is derivable
    exclusion_list.push_back(VELOCITY_Y);
    SensorValue position_y_sensor_value = GetSensorValue(POSITION_Y, exclusion_list, sensor_status, sensor_history);
    if (position_y_sensor_value.mode != 2) {
        sensor_value.value = (sensor_history->position_y_hist[sensor_history->current_index] - sensor_history->position_y_hist[(sensor_history->current_index + 1) % READINGS]) / (READINGS/50);
        sensor_value.value *= -1;
        sensor_value.mode = 1;
        return sensor_value;
    }
    
    return sensor_value;
}

SensorValue GetPositionX(std::vector<int> exclusion_list, SensorStatus sensor_status, const SensorHistory *sensor_history) {
    SensorValue sensor_value;
    if (sensor_status.position_x_ok && !SensorInExclusionList(POSITION_X, exclusion_list)) {
        sensor_value.value = Position_X();
        sensor_value.mode = 0;
        return sensor_value;
    }

    // Velocity X still functioning AND not excluded AND velocity_x is derivable
    exclusion_list.push_back(POSITION_X);
    SensorValue velocity_x_sensor_value = GetSensorValue(VELOCITY_X, exclusion_list, sensor_status, sensor_history);
    if (velocity_x_sensor_value.mode != 2) {
        // Calculate current position using past position and current velocity
        double past_position_x = sensor_history->position_x_hist[(sensor_history->current_index + 1 + READINGS) % READINGS];
        double current_velocity_x = velocity_x_sensor_value.value;
        double time_step = 4;
        
        sensor_value.value = past_position_x + (current_velocity_x * time_step);
        sensor_value.mode = 1;
        return sensor_value;
    }

    return sensor_value;
}

SensorValue GetPositionY(std::vector<int> exclusion_list, SensorStatus sensor_status, const SensorHistory *sensor_history) {
    SensorValue sensor_value;
    if (sensor_status.position_y_ok && !SensorInExclusionList(POSITION_Y, exclusion_list)) {
        sensor_value.value = Position_Y();
        sensor_value.mode = 0;
        return sensor_value;
    }

    // Velocity Y still functioning AND not excluded and velocity_y is derivable
    exclusion_list.push_back(POSITION_Y);
    SensorValue velocity_y_sensor_value = GetSensorValue(VELOCITY_Y, exclusion_list, sensor_status, sensor_history);
    if (velocity_y_sensor_value.mode != 2) {
        // Calculate current position using past position and current velocity
        double past_position_y = sensor_history->position_y_hist[(sensor_history->current_index + 1 + READINGS) % READINGS];
        double current_velocity_y = velocity_y_sensor_value.value;
        double time_step = 4; // Same time step used in velocity calculation
        
        sensor_value.value = past_position_y + (-1 * current_velocity_y * time_step);
        sensor_value.mode = 1; // Derived value
        return sensor_value;
    }

    return sensor_value;
}

SensorValue GetAngle(std::vector<int> exclusion_list, SensorStatus sensor_status, const SensorHistory *sensor_history) {
    SensorValue sensor_value;

    double sum = 0;
    for (int i = 0; i < 1000; i++) {
        sum += (Angle() + 360.0);
    }
    sensor_value.value = sum / 1000.0;
    sensor_value.mode = 0;

    return sensor_value;
}

SensorValue GetSonar(std::vector<int> exclusion_list, SensorStatus sensor_status, const SensorHistory *sensor_history) {
    SensorValue sensor_value;
    if (sensor_status.sonar_ok && !SensorInExclusionList(SONAR, exclusion_list)) {
        sensor_value.value = 0;
        sensor_value.mode = 0;
        return sensor_value;
    }

    // We will probably not derive for sonar
    SensorValue default_invalid_sensor_value = {0.0, 2};
    return default_invalid_sensor_value;
}

SensorValue GetRangeDist(std::vector<int> exclusion_list, SensorStatus sensor_status, const SensorHistory *sensor_history) {
    // RangeDist is never fails so return the value directly
    SensorValue sensor_value;
    sensor_value.value = RangeDist();
    sensor_value.mode = 0;
    return sensor_value;
}
