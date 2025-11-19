#pragma once

#include "roboAI.h"

void normalize_vector(double *x, double *y);
int calculateShootingVector(struct RoboAI *ai, double *goal_x, double *goal_y, double *vec_x, double *vec_y);
int calculatePointsWithinEpsilon(double *x1, double *y1, double *x2, double *y2, double epsilon);
int calculateGoalPosition(struct RoboAI *ai, double *goal_x, double *goal_y);
int calculateTargetPointVector(struct RoboAI *ai, double *targetPointX, double *targetPointY, double *vectorX, double *vectorY);
double f_angle(double x1, double y1, double x2, double y2);