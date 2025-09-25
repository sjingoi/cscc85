#include "Lander.h"
#include "Sensor_Fallback.h"
#include "Sensor_History.h"
#include "Sensor_Status.h"
#include "math.h"
#include <vector>

// previous sensor readings
double prev_velocity_x = 0.0;
double prev_velocity_y = 0.0;
double prev_position_x = 0.0;
double prev_position_y = 0.0;
double prev_angle = 0.0;

// function to detect sensor failures by checking for anomalies
void UpdateSensorStatus(SensorStatus *sensor_status, SensorHistory sensor_history, std::vector<double> sonar) {
    UpdateSensorHistory(*sensor_status);
    
    static int call_count = 0;
    call_count++;
    
     std::vector<int> exclusion_list;
    // Get current readings
    double curr_velocity_x = GetSensorValue(VELOCITY_X, exclusion_list, *sensor_status, sensor_history).value;
    double curr_velocity_y = GetSensorValue(VELOCITY_Y, exclusion_list, *sensor_status, sensor_history).value;
    double curr_position_x = GetSensorValue(POSITION_X, exclusion_list, *sensor_status, sensor_history).value;
    double curr_position_y = GetSensorValue(POSITION_Y, exclusion_list, *sensor_status, sensor_history).value;
    double curr_angle = fmod(GetSensorValue(ANGLE, exclusion_list, *sensor_status, sensor_history).value, 360.0);
    
    // after a few calls, start checking for anomalies
    if (call_count > 10) {
        // check velocity X sensor
        if (fabs(curr_velocity_x - prev_velocity_x) > 15.0 || 
            (curr_velocity_x == prev_velocity_x && call_count > 10)) {
            sensor_status->velocity_x_ok = 0;
        }
        
        // check velocity Y sensor
        if (fabs(curr_velocity_y - prev_velocity_y) > 15.0 || 
            (curr_velocity_y == prev_velocity_y && call_count > 10)) {
            sensor_status->velocity_y_ok = 0;
        }
        
        // check position X sensor
        if (fabs(curr_position_x - prev_position_x) > 100.0) {
            sensor_status->position_x_ok = 0;
        }
        
        // check position Y sensor
        if (fabs(curr_position_y - prev_position_y) > 100.0) {
            sensor_status->position_y_ok = 0;
        }
        
        // check angle sensor
        double angle_diff = fabs(curr_angle - prev_angle);
        // if difference > 180, it's actually the shorter path around the circle
        if (angle_diff > 180.0) {
            angle_diff = 360.0 - angle_diff;
        }
        if (angle_diff > 90.0) {
            sensor_status->angle_ok = 0;
        }
        
        // check sonar sensor
        static int sonar_has_had_readings = 0;
        static int prev_valid_count = 0;
        double range_dist = GetSensorValue(RANGEDIST, exclusion_list, *sensor_status, sensor_history).value;
        
        int current_valid_count = 0;
        for (int i = 0; i < 36; i++) {
            if (sonar[i] > 0) current_valid_count++;
        }
        
        // track if we've ever had valid sonar readings
        if (current_valid_count > 0) {
            sonar_has_had_readings = 1;
        }
        
        // validate with RangeDist (laser NEVER fails)
        if (range_dist > 0 && range_dist < 300) {  // ground detected within reasonable sonar range
            // RangeDist points in main thruster direction
            // check sonar readings (roughly indices 14-22 for downward ~180°)
            int downward_readings = 0;
            for (int i = 14; i < 23; i++) {
                if (sonar[i] > 0 && sonar[i] < range_dist + 50) {  // Allow some tolerance
                    downward_readings++;
                }
            }
            
            // if laser sees ground but no sonar readings, sonar is broken
            if (downward_readings == 0) {
                sensor_status->sonar_ok = 0;  // sonar failed
            }
        }
        
        // secondary check: pattern analysis (had readings before)
        if (sonar_has_had_readings) {
            if (current_valid_count == 0 && prev_valid_count > 3) {
                sensor_status->sonar_ok = 0;  // complete sensor failure
            }
            else if (prev_valid_count > 10 && current_valid_count < 3) {
                sensor_status->sonar_ok = 0;  // severe sensor degradation
            }
        }
        
        prev_valid_count = current_valid_count;
    }
    
    // Update previous readings
    prev_velocity_x = curr_velocity_x;
    prev_velocity_y = curr_velocity_y;
    prev_position_x = curr_position_x;
    prev_position_y = curr_position_y;
    prev_angle = curr_angle;
}

// // Function to print current sensor status
// void PrintSensorStatus() {
//     static int last_velocity_x_ok = -1;
//     static int last_velocity_y_ok = -1;
//     static int last_position_x_ok = -1;
//     static int last_position_y_ok = -1;
//     static int last_angle_ok = -1;
//     static int last_sonar_ok = -1;
//     static int print_counter = 0;
    
//     print_counter++;
    
//     // Print status every 50 calls or when status changes
//     if (print_counter % 50 == 0 ||
//         sensor_status.velocity_x_ok != last_velocity_x_ok ||
//         sensor_status.velocity_y_ok != last_velocity_y_ok ||
//         sensor_status.position_x_ok != last_position_x_ok ||
//         sensor_status.position_y_ok != last_position_y_ok ||
//         sensor_status.angle_ok != last_angle_ok ||
//         sensor_status.sonar_ok != last_sonar_ok) {
        
//         printf("=== SENSOR STATUS ===\n");
//         printf("Velocity X: %s\n", sensor_status.velocity_x_ok ? "OK" : "FAILED");
//         printf("Velocity Y: %s\n", sensor_status.velocity_y_ok ? "OK" : "FAILED");
//         printf("Position X: %s\n", sensor_status.position_x_ok ? "OK" : "FAILED");
//         printf("Position Y: %s\n", sensor_status.position_y_ok ? "OK" : "FAILED");
//         printf("Angle:      %s\n", sensor_status.angle_ok ? "OK" : "FAILED");
//         printf("Sonar:      %s\n", sensor_status.sonar_ok ? "OK" : "FAILED");
//         printf("====================\n");
        
//         last_velocity_x_ok = sensor_status.velocity_x_ok;
//         last_velocity_y_ok = sensor_status.velocity_y_ok;
//         last_position_x_ok = sensor_status.position_x_ok;
//         last_position_y_ok = sensor_status.position_y_ok;
//         last_angle_ok = sensor_status.angle_ok;
//         last_sonar_ok = sensor_status.sonar_ok;
//     }
// }