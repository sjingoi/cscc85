#include "Sensor_History.h"
#include "Lander_Control.h"
#include "stdio.h"

#define READINGS 50
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

SensorHistory sensor_history = {0, 0, {0}, {0}, {0}, {0}, {0}, {0}, {{0}}}; // global instance

void UpdateSensorHistory() {
    static int call_count = 0;
    call_count++;

    // DEBUG: Print before and after index change
    int old_index = sensor_history.current_index;
    sensor_history.current_index = (sensor_history.current_index + 1) % READINGS;
    
    // DEBUG: Every 10 calls, show what's happening
    if (call_count % 10 == 0) {
        printf("DEBUG: call %d, old_idx=%d, new_idx=%d, count=%d, READINGS=%d\n", 
               call_count, old_index, sensor_history.current_index, sensor_history.count, READINGS);
    }
    
    if (sensor_history.count < READINGS) {
        sensor_history.count++;
    }

    int idx_hist = sensor_history.current_index;
    sensor_history.velocity_x_hist[idx_hist] = Velocity_X();
    sensor_history.velocity_y_hist[idx_hist] = Velocity_Y();
    sensor_history.position_x_hist[idx_hist] = Position_X();
    sensor_history.position_y_hist[idx_hist] = Position_Y();
    sensor_history.angle_hist[idx_hist] = Angle();
    sensor_history.range_dist_hist[idx_hist] = RangeDist();
    for (int i = 0; i < 36; i++) {
        sensor_history.sonar_hist[idx_hist][i] = SONAR_DIST[i];
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
