#include "Denoising.h"
#include "Lander_Control.h"
#include <math.h>
#include <string.h>

#define MAX_SENSORS 10
#define MAX_BUFFER_SIZE 20

// global buffers for each sensor (circular buffers)
static double sensor_buffers[MAX_SENSORS][MAX_BUFFER_SIZE];
static int buffer_indices[MAX_SENSORS];
static int buffer_sizes[MAX_SENSORS];
static int initialized[MAX_SENSORS] = {0};

// initialize a filter buffer for a specific sensor
void InitializeFilter(int sensor_id, int buffer_size) {
    if (sensor_id >= MAX_SENSORS || buffer_size > MAX_BUFFER_SIZE) return;
    
    buffer_sizes[sensor_id] = buffer_size;
    buffer_indices[sensor_id] = 0;
    
    // Initialize with zeros
    for (int i = 0; i < buffer_size; i++) {
        sensor_buffers[sensor_id][i] = 0.0;
    }
    
    initialized[sensor_id] = 1;
}

// convolution filter implementation (O(j) = Σ I(j+k) * h(-k))
double ConvolutionFilter(double new_value, double* kernel, int kernel_size, int sensor_id) {
    if (!initialized[sensor_id]) {
        InitializeFilter(sensor_id, kernel_size);
    }
    
    // add new value to circular buffer
    sensor_buffers[sensor_id][buffer_indices[sensor_id]] = new_value;
    buffer_indices[sensor_id] = (buffer_indices[sensor_id] + 1) % buffer_sizes[sensor_id];
    
    // perform convolution: O(j) = Σ I(j+k) * h(-k)
    double output = 0.0;
    
    for (int k = 0; k < kernel_size; k++) {
        // Get value from k steps back in circular buffer
        int buffer_idx = (buffer_indices[sensor_id] - 1 - k + buffer_sizes[sensor_id]) % buffer_sizes[sensor_id];
        output += sensor_buffers[sensor_id][buffer_idx] * kernel[k];
    }
    
    return output;
}

// gaussian averaging filter (bell curve weighting)
double GaussianFilter(double new_value, int sensor_id) {
    // 5-tap Gaussian kernel (normalized)
    static double gaussian_kernel[] = {0.06136, 0.24477, 0.38774, 0.24477, 0.06136};
    return ConvolutionFilter(new_value, gaussian_kernel, 5, sensor_id);
}

// triangular filter (linear weight decrease)
double TriangularFilter(double new_value, int sensor_id) {
    // 5-tap triangular kernel (normalized)
    static double triangular_kernel[] = {0.06667, 0.13333, 0.20000, 0.26667, 0.33333};
    return ConvolutionFilter(new_value, triangular_kernel, 5, sensor_id);
}

// uniform filter (simple moving average)
double UniformFilter(double new_value, int sensor_id) {
    // 5-tap uniform kernel
    static double uniform_kernel[] = {0.2, 0.2, 0.2, 0.2, 0.2};
    return ConvolutionFilter(new_value, uniform_kernel, 5, sensor_id);
}

// weighted average utility function
double WeightedAverage(double* values, double* weights, int count) {
    if (count <= 0) return 0.0;
    
    double weighted_sum = 0.0;
    double weight_sum = 0.0;
    
    for (int i = 0; i < count; i++) {
        weighted_sum += values[i] * weights[i];
        weight_sum += weights[i];
    }
    
    return weight_sum > 0 ? weighted_sum / weight_sum : 0.0;
}

// sensor-specific denoising functions using convolution filters
double DenoiseVelocityX(SensorHistory h) {
    double current_value = Velocity_X();
    // use gaussian filter for velocity (smooth but responsive)
    return GaussianFilter(current_value, 0); // sensor_id = 0 for VelX
}

double DenoiseVelocityY(SensorHistory h) {
    double current_value = Velocity_Y();
    return GaussianFilter(current_value, 1); // sensor_id = 1 for VelY
}

double DenoisePositionX(SensorHistory h) {
    double current_value = Position_X();
    return TriangularFilter(current_value, 2); // sensor_id = 2 for PosX
}

double DenoisePositionY(SensorHistory h) {
    double current_value = Position_Y();
    return TriangularFilter(current_value, 3); // sensor_id = 3 for PosY
}

double DenoiseSonar(int sonar_index, SensorHistory h) {
    if (sonar_index < 0 || sonar_index >= 36) {
        return -1.0; // invalid sonar index
    }
    
    double current_value = SONAR_DIST[sonar_index];
    
    // Handle invalid readings
    if (current_value <= 0) {
        return current_value; // Return as-is for invalid readings
    }
    
    // Use gaussian filter for sonar (smooth noise reduction)
    // Use sensor_id = 4 + sonar_index, but limit to available sensors
    int sensor_id = 4 + (sonar_index % 6); // Rotate through 6 sensor slots for 36 sonars
    return UniformFilter(current_value, sensor_id);
}