#include "roboAI.h"

/** 
 * Make a turn with turn radius turn_rad in direction left (dir == 0) or right (dir == 1)
 */
void turn_radius(int pw, double turn_rad, int dir);

void move_forward(int pw, struct RoboAI *ai);

void turn_left(int pw);

void turn_right(int pw);

/**
 * Turn towards a direction vector. Returns 1 if aligned, 0 if not.
 */
int turn_towards_dir(struct RoboAI *ai, double t_dir_x, double t_dir_y);