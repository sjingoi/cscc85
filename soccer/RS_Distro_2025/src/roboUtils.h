#pragma once

#include "roboAI.h"

#define CM_PER_PIXEL_X 0.1328
#define CM_PER_PIXEL_Y 0.1597

void convert_to_metric(struct RoboAI *ai);
void determine_facing(struct RoboAI *ai);
double dot(double x1, double y1, double x2, double y2);
double norm(double x, double y);
void normalize_vector(double *x, double *y);
double angle_diff(double angle, double target_angle);
int calculateShootingVector(struct RoboAI *ai, double *goal_x, double *goal_y, double *vec_x, double *vec_y, int team);
int calculatePointsWithinEpsilon(double *x1, double *y1, double *x2, double *y2, double epsilon);
int calculateGoalPosition(struct RoboAI *ai, double *goal_x, double *goal_y, int team);
int calculateTargetPointVector(struct RoboAI *ai, double *targetPointX, double *targetPointY, double *vectorX, double *vectorY, int team);
double f_angle(double x1, double y1, double x2, double y2);
void update_vars(struct RoboAI *ai);
double point_distance(double x1, double y1, double x2, double y2);
void updateTargetDefensivePosition(struct RoboAI *ai);