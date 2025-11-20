#pragma once

#include "./EV3_RobotControl/bytecodes.h"

enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

enum RelativeDirection {
    FORWARD = 0,
    RIGHT = 1,
    BACKWARD = 2,
    LEFT = 3
};

struct MapIntersection {
    NXTCOLOR nw; // Northwest color
    NXTCOLOR ne; // Northeast color
    NXTCOLOR se; // Southeast color
    NXTCOLOR sw; // Southwest color
};

struct IntersectionBelief {
    double north; // Probability north
    double east;  // Probability east
    double south; // Probability south
    double west;  // Probability west
};

struct LocalizationMap {
    struct MapIntersection instersections[400];
    struct IntersectionBelief beliefs[400];
    int sx; // Size of map x
    int sy; // Size of map y
};

/**
 * Finds the possible intersections given an intersection layout without a known direction 
 */
int find_intersections_nodir(const struct LocalizationMap *map, const NXTCOLOR corners[4], int possibilities[400], enum Direction directions[400]);

/**
 * Finds the possible intersections given a known intersection layout
 */
int find_intersections_dir(const struct LocalizationMap *map, struct MapIntersection intersection, int possibilities[400]);

/**
 * Update the localization model with an initial scanned intersection.
 */
void localize_init(struct LocalizationMap *map, const NXTCOLOR scanned_corners[4]);

/**
 * Update the localization model with a discovered intersection and the direction travelled from the last call.
 */
void localize(struct LocalizationMap *map, NXTCOLOR scanned_corners[4], enum RelativeDirection last_movement);

/**
 * Returns true if the coord is in the map, false otherwise.
 */
bool valid_coord(const struct LocalizationMap *map, int x, int y);

/**
 * Converts map index to an xy coordinate based on the size of the map
 */
void idx_to_coord(const struct LocalizationMap *map, int idx, int *x, int *y);

/**
 * Converts a coordinate into a map index
 */
int coord_to_idx(const struct LocalizationMap *map, int x, int y);

/**
 * Returns a pointer to the double which represents the belief at intersecion b and direction d
 */
double *belief(struct IntersectionBelief *b, enum Direction d);

/**
 * 
 */
void get_highest_belief_coord(const struct LocalizationMap *map, int *x_ret, int *y_ret, enum Direction *dir_ret);

/**
 * Add a relative direction to a caridnal direction
 */
enum Direction dir_add(enum Direction dir, enum RelativeDirection rel_dir);

/**
 * Subtract a relative direction to a caridnal direction
 */
enum Direction dir_sub(enum Direction dir, enum RelativeDirection rel_dir);

/**
 * Relative direction between dir1 -> dir 2
 */
enum RelativeDirection dir_diff(enum Direction dir1, enum Direction dir2);

/**
 * Shift coordinate by a direction
 */
void pos_add(int x, int y, enum Direction dir, int *x_ret, int *y_ret);

/**
 * Shift coordinate by the opposite direction
 */
void pos_sub(int x, int y, enum Direction dir, int *x_ret, int *y_ret);

void print_beliefs(const struct LocalizationMap *map);