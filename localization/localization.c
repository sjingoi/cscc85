#include "localization.h"
#include "string.h"
#include "stdio.h"

// The following variables are defined globally to prevent unessesary memory re-allocation on each function call. Ensure that only one is used at a time.
// possibilites return
int p_ret[400]; 
// directions return
enum Direction d_ret[400]; 
// new beliefs
struct IntersectionBelief new_beliefs[400];

int find_intersections_nodir(const struct LocalizationMap *map, const NXTCOLOR corners[4], int possibilities[400], enum Direction directions[400]) {
    int count = 0;
    int count_n, count_e, count_s, count_w;

    struct MapIntersection r0 = {corners[0], corners[1], corners[2], corners[3], };
    struct MapIntersection r1 = {corners[3], corners[0], corners[1], corners[2], };
    struct MapIntersection r2 = {corners[2], corners[3], corners[0], corners[1], };
    struct MapIntersection r3 = {corners[1], corners[2], corners[3], corners[0], };
    
    count_n = find_intersections_dir(map, r0, &(possibilities[0                             ]));
    count_e = find_intersections_dir(map, r1, &(possibilities[count_n                       ]));
    count_s = find_intersections_dir(map, r2, &(possibilities[count_n + count_e             ]));
    count_w = find_intersections_dir(map, r3, &(possibilities[count_n + count_e + count_s   ]));
    
    for (int i = 0; i < count_n; i++) directions[0 + i] = NORTH;
    for (int i = 0; i < count_e; i++) directions[count_n + i] = EAST;
    for (int i = 0; i < count_s; i++) directions[count_n + count_e + i] = SOUTH;
    for (int i = 0; i < count_w; i++) directions[count_n + count_e + count_s + i] = WEST;
    
    return count_n + count_e + count_s + count_w;
}

bool intersection_equal(const struct MapIntersection *a, const struct MapIntersection *b) {
    return (a->nw == b->nw) && (a->ne == b->ne) && (a->se == b->se) && (a->sw == b->sw);
}

int find_intersections_dir(const struct LocalizationMap *map, struct MapIntersection intersection, int possibilities[400]) {
    int count = 0;
    for (int i = 0; i < map->sx * map->sy; i++) {
        if (intersection_equal(&intersection, &(map->instersections[i]))) {
            possibilities[count] = i;
            count++;
        }
    }
    return count;
}

bool valid_coord(const struct LocalizationMap *map, int x, int y) {
    return x >= 0 && y >= 0 && x < map->sx && y < map->sy;
}

void idx_to_coord(const struct LocalizationMap *map, int idx, int *x, int *y) {
    *x = idx % map->sx;
    *y = idx / map->sx;
}

int coord_to_idx(const struct LocalizationMap *map, int x, int y) {
    return y * map->sx + x;
}

void normalize_beliefs(struct IntersectionBelief beliefs[400], int sx, int sy) {
    double sum = 0;

    for (int i = 0; i < sx * sy; i++) sum += beliefs[i].east;
    for (int i = 0; i < sx * sy; i++) sum += beliefs[i].north;
    for (int i = 0; i < sx * sy; i++) sum += beliefs[i].south;
    for (int i = 0; i < sx * sy; i++) sum += beliefs[i].west;

    for (int i = 0; i < sx * sy; i++) beliefs[i].north /= sum;
    for (int i = 0; i < sx * sy; i++) beliefs[i].east /= sum;
    for (int i = 0; i < sx * sy; i++) beliefs[i].south /= sum;
    for (int i = 0; i < sx * sy; i++) beliefs[i].west /= sum;
}

void normalize(struct LocalizationMap *map) {
    normalize_beliefs(map->beliefs, map->sx, map->sy);
}

double *belief(struct IntersectionBelief *b, enum Direction d) {
    if (d == NORTH) return &(b->north);
    if (d == EAST) return &(b->east);
    if (d == SOUTH) return &(b->south);
    if (d == WEST) return &(b->west);
    return NULL;
}

void get_highest_belief_coord(const struct LocalizationMap *map, int *x_ret, int *y_ret, enum Direction *dir_ret) {
    struct LocalizationMap *map_m = (struct LocalizationMap *) map;
    int highest_idx = 0;
    enum Direction highest_dir = NORTH;
    for (int i = 0; i < map_m->sx * map_m->sy; i++) {
        for (int j = 0; j < 4; j++) {
            if (*(belief(&(map_m->beliefs[i]), (enum Direction) j)) > *(belief(&(map_m->beliefs[highest_idx]), (enum Direction) highest_dir))) {
                highest_idx = i;
                highest_dir = (enum Direction) j;
            }
        }
    }
    idx_to_coord(map_m, highest_idx, x_ret, y_ret);
    *dir_ret = highest_dir;
}

void localize_init(struct LocalizationMap *map, const NXTCOLOR scanned_corners[4]) {
    int count = find_intersections_nodir(map, scanned_corners, p_ret, d_ret);

    double confidence = 1.5;

    for (int i = 0; i < count; i++) {
        int idx = p_ret[i];
        enum Direction dir = d_ret[i];
        *(belief(&(map->beliefs[idx]), dir)) += confidence;
    }

    normalize(map);
}

enum Direction dir_add(enum Direction dir, enum RelativeDirection rel_dir) {
    return (enum Direction) (((int) dir + (int) rel_dir) % 4);
}

enum Direction dir_sub(enum Direction dir, enum RelativeDirection rel_dir) {
    return (enum Direction) (((int) dir - (int) rel_dir + 4) % 4);
}

enum RelativeDirection dir_diff(enum Direction dir1, enum Direction dir2) {
    return (enum RelativeDirection) (((int) dir2 - (int) dir1 + 8) % 4);
}

void pos_add(int x, int y, enum Direction dir, int *x_ret, int *y_ret) {
    switch (dir)
    {
    case NORTH:
        *y_ret = y - 1;
        *x_ret = x;
        break;
    case EAST:
        *y_ret = y;
        *x_ret = x + 1;
        break;
    case SOUTH:
        *y_ret = y + 1;
        *x_ret = x;
        break;
    case WEST:
        *y_ret = y;
        *x_ret = x - 1;
        break;
    default:
        printf("ERROR UNKNOWN DIRECTION ADDED.\n");
        *y_ret = -1;
        *x_ret = -1;
        break;
    }
}

void pos_sub(int x, int y, enum Direction dir, int *x_ret, int *y_ret) {
    pos_add(x, y, dir_add(dir, BACKWARD), x_ret, y_ret);
}

void localize(struct LocalizationMap *map, NXTCOLOR scanned_corners[4], enum RelativeDirection last_movement) {
    int count = find_intersections_nodir(map, scanned_corners, p_ret, d_ret);

    double confidence = 4.0;
    
    // Initialize new beliefs
    for (int i = 0; i < map->sx * map->sy; i++) {
        int x, y, x_prev, y_prev;
        idx_to_coord(map, i, &x, &y);
        for (int j = 0; j < 4; j++) {
            enum Direction dir = (enum Direction) j;
            enum Direction dir_prev = dir_sub(dir, last_movement);
            pos_sub(x, y, dir, &x_prev, &y_prev);
            
            // Shift beliefs in the direction of movement
            if (valid_coord(map, x_prev, y_prev)) {
                int i_prev = coord_to_idx(map, x_prev, y_prev);
                *(belief(&(new_beliefs[i]), dir)) = *(belief(&(map->beliefs[i_prev]), dir_prev));
            } else {
                // If the previous intersection falls outside of the map, then we could be at this x,y.
                *(belief(&(new_beliefs[i]), dir)) = 0.005;
            }
        }
    }

    normalize_beliefs(new_beliefs, map->sx, map->sy);

    for (int i = 0; i < count; i++) {
        int idx = p_ret[i];
        int prev_idx;
        int x, y, prev_x, prev_y;
        idx_to_coord(map, idx, &x, &y);
        enum Direction dir = d_ret[i];
        enum Direction prev_dir;

        double *last_pos;
        prev_dir = dir_sub(dir, last_movement); // the previous direction is the possible direction minus the last movement direction (east and we moved right => we were facing north before)
        pos_sub(x, y, dir, &prev_x, &prev_y); // the previous position is always the position behind of the current direction we are facing
        
        if (valid_coord(map, prev_x, prev_y)) {
            prev_idx = coord_to_idx(map, prev_x, prev_y);
            double prev_belief = *(belief(&(map->beliefs[prev_idx]), prev_dir));
            *(belief(&(new_beliefs[idx]), dir)) += confidence * prev_belief;
        } else {
            // If that position is invalid, this position is less likely. We will increase this possibility by a bit anyways just incase there was an error navigating.
            *(belief(&(new_beliefs[idx]), dir)) += confidence / 5;
        }

        printf("Increased index %d\n", idx);
    }

    memcpy(&(map->beliefs[0]), &(new_beliefs[0]), sizeof(new_beliefs));

    normalize(map);
}

void print_beliefs(const struct LocalizationMap *map) {
    printf("=============== BELIEFS MAP: ==============\n");
    for (int i = 0; i < map->sy; i++) {
        printf("\n");
        for (int j = 0; j < map->sx; j++) printf("       %.4f       ", map->beliefs[i * map->sx + j].north);
        printf("\n");
        printf("\n");
        for (int j = 0; j < map->sx; j++) printf("   %.4f  %.4f   ", map->beliefs[i * map->sx + j].west, map->beliefs[i * map->sx + j].east);
        printf("\n");
        printf("\n");
        for (int j = 0; j < map->sx; j++) printf("       %.4f       ", map->beliefs[i * map->sx + j].south);
        printf("\n");
        printf("\n");
    }
}