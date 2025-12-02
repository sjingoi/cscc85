#include "roboAI.h"

/** 
 * Make a turn with turn radius turn_rad in direction left (dir == 0) or right (dir == 1)
 */
void turn_radius(int pw, double turn_rad, int dir, struct RoboAI *ai);

void go_to_point(struct RoboAI *ai, int power, double turn_smoothness, double target_x, double target_y);

void move_forward(int pw, struct RoboAI *ai);

void stop_moving(struct RoboAI *ai);

void turn_left(int pw);

void turn_right(int pw);

/**
 * Turn towards a direction vector. Returns 1 if aligned, 0 if not.
 */
int turn_towards_dir(struct RoboAI *ai, double t_dir_x, double t_dir_y);

void safe_go_to_point(struct RoboAI *ai, int power, double turn_smoothness, double target_x, double target_y);

int detect_ball(struct RoboAI *ai, double forward_dist, double lateral_dist, int team);