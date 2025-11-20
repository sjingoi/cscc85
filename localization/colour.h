#include "./EV3_RobotControl/bytecodes.h"

#define COLOUR_HIST_SIZE 32

struct ColourHSV {
    double h;
    double s;
    double v;
};

struct ColourRGB {
    double r;
    double g;
    double b;
};

#define MAX_TRAINING_COUNT 1024

struct ColourTrainingSet {
    struct ColourHSV data[MAX_TRAINING_COUNT];
    NXTCOLOR targets[MAX_TRAINING_COUNT];
    int count;
};

struct ColourConfidence {
    NXTCOLOR colour;
    double confidence; // Confidence from 0.0 (unsure) to 1.0 (certain)
};

struct ColourHistory {
    NXTCOLOR history[COLOUR_HIST_SIZE];
    int size; // Size of history from 0 to COLOUR_HIST_SIZE
    int latest_idx; // Index of the latest colour.
};

struct ColourHSV rgb_to_hsv(const struct ColourRGB *c);

struct ColourRGB get_sensor_rgb(char port);

double colour_hsv_dist(const struct ColourHSV *hsv1, const struct ColourHSV *hsv2);

void add_datapoint(struct ColourTrainingSet *ts, const struct ColourHSV *hsv, NXTCOLOR target);

NXTCOLOR predict_colour(const struct ColourTrainingSet *ts, const struct ColourHSV *hsv);

NXTCOLOR read_nxt_color(const struct ColourTrainingSet *ts);

struct ColourConfidence read_colour();

void start_colour_thread(const struct ColourTrainingSet *ts);

/**
 * Blocks until the color c is detected. The color is considered detected
 * when it was read n times in a row.
 */
void wait_for_colour(NXTCOLOR c, int n);

void print_colour_nxt(NXTCOLOR c);

void print_colour_rgb(const struct ColourRGB *c);

void print_colour_hsv(const struct ColourHSV *hsv);

void load_training_set(struct ColourTrainingSet *ts);

void write_training_set(const struct ColourTrainingSet *ts);