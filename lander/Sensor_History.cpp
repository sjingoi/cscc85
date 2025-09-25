#include "Sensor_History.h"
#include "Lander_Control.h"
#include "Sensor_Fallback.h"
#include "stdio.h"
#include <vector>

// Global instance definition (actual memory allocation)
SensorHistory sensor_history = {0, 0, {0}, {0}, {0}, {0}, {0}, {0}, {{0}}}; // global instance

void UpdateSensorHistory(SensorStatus sensor_status, SensorHistory *sensor_history) {
    static int call_count = 0;
    call_count++;

    // DEBUG: Print before and after index change
    int old_index = sensor_history->current_index;
    sensor_history->current_index = (sensor_history->current_index + 1) % READINGS;
    
    // DEBUG: Every 10 calls, show what's happening
    if (call_count % 10 == 0) {
        printf("DEBUG: call %d, old_idx=%d, new_idx=%d, count=%d, READINGS=%d\n", 
               call_count, old_index, sensor_history->current_index, sensor_history->count, READINGS);
        printf("       velocity_x_hist: ");
        for (int i = 0; i < READINGS; i++) {
            printf("%f ", sensor_history->velocity_x_hist[i]);
        }
        printf("\n");
    }
    
    if (sensor_history->count < READINGS) {
        sensor_history->count++;
    }

    // Create empty exclusion list for robust sensor reading
    std::vector<int> exclusion_list;
    
    int idx_hist = sensor_history->current_index;
    sensor_history->velocity_x_hist[idx_hist] = GetSensorValue(VELOCITY_X, exclusion_list, sensor_status, sensor_history).value;
    sensor_history->velocity_y_hist[idx_hist] = GetSensorValue(VELOCITY_Y, exclusion_list, sensor_status, sensor_history).value;
    sensor_history->position_x_hist[idx_hist] = GetSensorValue(POSITION_X, exclusion_list, sensor_status, sensor_history).value;
    sensor_history->position_y_hist[idx_hist] = GetSensorValue(POSITION_Y, exclusion_list, sensor_status, sensor_history).value;
    sensor_history->angle_hist[idx_hist] = GetSensorValue(ANGLE, exclusion_list, sensor_status, sensor_history).value;
    sensor_history->range_dist_hist[idx_hist] = GetSensorValue(RANGEDIST, exclusion_list, sensor_status, sensor_history).value;
    
    // For sonar, we still use direct access since GetSensorValue(SONAR, ...) doesn't handle individual indices
    // Future enhancement: implement GetSensorValue for individual sonar indices
    // For now, this provides the raw sonar data that can be validated by sensor status functions
    for (int i = 0; i < 36; i++) {
        sensor_history->sonar_hist[idx_hist][i] = SONAR_DIST[i];
    }
}

// helper funcs
double GetHistoricalVelocityX(int steps_back) {
    if (steps_back >= sensor_history.count) {
        return sensor_history.velocity_x_hist[sensor_history.current_index];
    }
    int idx = (sensor_history.current_index - steps_back + READINGS) % READINGS;
    return sensor_history.velocity_x_hist[idx];
}

double GetHistoricalVelocityY(int steps_back) {
    if (steps_back >= sensor_history.count) {
        return sensor_history.velocity_y_hist[sensor_history.current_index];
    }
    int idx = (sensor_history.current_index - steps_back + READINGS) % READINGS;
    return sensor_history.velocity_y_hist[idx];
}

double GetHistoricalPositionX(int steps_back) {
    if (steps_back >= sensor_history.count) {
        return sensor_history.position_x_hist[sensor_history.current_index];
    }
    int idx = (sensor_history.current_index - steps_back + READINGS) % READINGS;
    return sensor_history.position_x_hist[idx];
}

double GetHistoricalPositionY(int steps_back) {
    if (steps_back >= sensor_history.count) {
        return sensor_history.position_y_hist[sensor_history.current_index];
    }
    int idx = (sensor_history.current_index - steps_back + READINGS) % READINGS;
    return sensor_history.position_y_hist[idx];
}

double GetHistoricalAngle(int steps_back) {
    if (steps_back >= sensor_history.count) {
        return sensor_history.angle_hist[sensor_history.current_index];
    }
    int idx = (sensor_history.current_index - steps_back + READINGS) % READINGS;
    return sensor_history.angle_hist[idx];
}

double GetHistoricalRangeDist(int steps_back) {
    if (steps_back >= sensor_history.count) {
        return sensor_history.range_dist_hist[sensor_history.current_index];
    }
    int idx = (sensor_history.current_index - steps_back + READINGS) % READINGS;
    return sensor_history.range_dist_hist[idx];
}

double GetHistoricalSonar(int steps_back, int sonar_index) {
    if (steps_back >= sensor_history.count) {
        return sensor_history.sonar_hist[sensor_history.current_index][sonar_index];
    }
    int idx = (sensor_history.current_index - steps_back + READINGS) % READINGS;
    return sensor_history.sonar_hist[idx][sonar_index];
}
