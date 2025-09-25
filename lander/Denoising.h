#include "Lander.h"

// convolution-based denoising functions
double ConvolutionFilter(double new_value, double* kernel, int kernel_size, int sensor_id);
double GaussianFilter(double new_value, int sensor_id);
double TriangularFilter(double new_value, int sensor_id); 
double UniformFilter(double new_value, int sensor_id);

// denoising functions
double DenoiseVelocityX(SensorHistory h);
double DenoiseVelocityY(SensorHistory h);
double DenoisePositionX(SensorHistory h);
double DenoisePositionY(SensorHistory h);
double DenoiseSonar(int sonar_index, SensorHistory h);

// Utility functions
double WeightedAverage(double* values, double* weights, int count);