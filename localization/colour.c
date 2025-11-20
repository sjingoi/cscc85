#include "colour.h"
#include "math.h"
#include "stdio.h"
#include "EV3_Localization.h"
#include "pthread.h"

struct ColourConfidence current_colour;
pthread_mutex_t current_colour_lock;
struct ColourHistory history;

struct ColourHSV rgb_to_hsv(const struct ColourRGB *c) {
    struct ColourHSV hsv;
    double max = fmax(c->r, fmax(c->g, c->b));
    double min = fmin(c->r, fmin(c->g, c->b));
    double delta = max - min;

    hsv.v = max;
    hsv.s = max == 0 ? 0 : delta / max;
    if (delta == 0) {
        hsv.h = 0;
    } else if (c->r == max) {
        hsv.h = fmod((c->g - c->b) / delta, 6.0);
    } else if (c->g == max) {
        hsv.h = (c->b - c->r) / delta + 2.0;
    } else {
        hsv.h = (c->r - c->g) / delta + 4.0;
    }

    hsv.h /= 6.0;
    if (hsv.h < 0.0) hsv.h += 1.0;

    return hsv;
}

struct ColourRGB get_sensor_rgb(char port) {
    struct ColourRGB c;
    int r, g, b, a;
    BT_read_colour_RGBraw_NXT(port, &r, &g, &b, &a);
    r += a;
    g += a;
    b += a;
    c.r = r / 1087.0;
    c.g = g / 1087.0;
    c.b = b / 1087.0;
    return c;
}

double colour_hsv_dist(const struct ColourHSV *hsv1, const struct ColourHSV *hsv2) {
    double dist;

    double h_dist = fabs(hsv1->h - hsv2->h);
    h_dist = h_dist > 0.5 ? (1.0 - h_dist) : h_dist;
    dist = h_dist * h_dist;
    dist += fabs(hsv1->s - hsv2->s) * fabs(hsv1->s - hsv2->s);
    dist += fabs(hsv1->v - hsv2->v) * fabs(hsv1->v - hsv2->v);
    dist = sqrt(dist);
    
    return dist;
}

void add_datapoint(struct ColourTrainingSet *ts, const struct ColourHSV *hsv, NXTCOLOR target) {
    if (ts->count >= MAX_TRAINING_COUNT) return;
    ts->data[ts->count] = *hsv;
    ts->targets[ts->count] = target;
    ts->count++;
}

NXTCOLOR predict_colour(const struct ColourTrainingSet *ts, const struct ColourHSV *hsv) {
    NXTCOLOR closest_target = WHITECOLOR;
    double closest_dist = INFINITY;
    for (int i = 0; i < ts->count; i++) {
        double dist = colour_hsv_dist(hsv, &(ts->data[i]));
        if (dist < closest_dist) {
            closest_target = ts->targets[i];
            closest_dist = dist;
        }
    }
    return closest_target;
}

void update_colour_confidence() {
    int counts[7] = {-1, 0, 0, 0, 0, 0, 0}; // map from color to occurances
    int largest_idx = -1;
    for (int i = 0; i < history.size; i++) counts[(int) history.history[i]]++;
    
    // Find most common occurance
    for (int idx = 1; idx < sizeof(counts) / sizeof(int); idx++) {
        if (counts[idx] > counts[largest_idx]) {
            largest_idx = idx;
        }
    }

    pthread_mutex_lock(&current_colour_lock);
    current_colour.colour = (NXTCOLOR) largest_idx;
    current_colour.confidence = (double) counts[largest_idx] / COLOUR_HIST_SIZE; // If the history hasnt been filled yet, the confidence will be low.
    pthread_mutex_unlock(&current_colour_lock);
}

struct ColourConfidence read_colour() {
    struct ColourConfidence value;
    pthread_mutex_lock(&current_colour_lock);
    value = current_colour;
    pthread_mutex_unlock(&current_colour_lock);
    return value;
}

NXTCOLOR read_nxt_color(const struct ColourTrainingSet *ts) {
    struct ColourRGB c = get_sensor_rgb(PORT_1);
    struct ColourHSV hsv = rgb_to_hsv(&c);
    return predict_colour(ts, &hsv);
}

void *colour_loop(void *arg) {
    const struct ColourTrainingSet *ts = (const struct ColourTrainingSet *) arg;
    for (history.size = 0; history.size < COLOUR_HIST_SIZE; history.size++) {
        history.latest_idx = history.size;
        history.history[history.latest_idx] = read_nxt_color(ts);
        update_colour_confidence();
    }

    while (1) {
        history.latest_idx = (history.latest_idx + 1) % COLOUR_HIST_SIZE;
        history.history[history.latest_idx] = read_nxt_color(ts);
        update_colour_confidence();
    }
}

void start_colour_thread(const struct ColourTrainingSet *ts) {
    pthread_t thread_id;
    pthread_mutex_init(&current_colour_lock, NULL);
    current_colour.colour = WHITECOLOR;
    current_colour.confidence = 0;
    history.size = 0;
    history.latest_idx = 0;
    pthread_create(&thread_id, NULL, colour_loop, (void *) ts);
}

void wait_for_colour(const struct ColourTrainingSet *ts, NXTCOLOR c, int n) {
    int count = 0;
    while (1) {
        NXTCOLOR c_read = read_nxt_color(ts);
        count = (c_read == c) ? count + 1 : 0;
        if (count >= n) break;
    }
}

void print_colour_nxt(NXTCOLOR c) {
    switch (c)
    {
    case BLACKCOLOR:
        printf("Black\n");
        break;
    case BLUECOLOR:
        printf("Blue\n");
        break;
    case GREENCOLOR:
        printf("Green\n");
        break;
    case YELLOWCOLOR:
        printf("Yellow\n");
        break;
    case REDCOLOR:
        printf("Red\n");
        break;
    case WHITECOLOR:
        printf("White\n");
        break;
    
    default:
        break;
    }
}

void print_colour_rgb(const struct ColourRGB *c) {
    printf("ColourRGB(%f %f %f)\n", c->r, c->g, c->b);
}

void print_colour_hsv(const struct ColourHSV *hsv) {
    printf("ColourHSV(%f %f %f)\n", hsv->h, hsv->s, hsv->v);
}

void load_training_set(struct ColourTrainingSet *ts) {
    FILE* file = fopen("color_training_set.csv", "r");
    if (!file) {
        perror("Error opening file");
        return;
    }

    char line[256];
    int line_num = 0;

    while(fgets(line, sizeof(line), file)) {
        if (line_num == 0) {
            line_num++;
            continue;
        }

        struct ColourHSV hsv;
        NXTCOLOR target;
        if (sscanf(line, "%lf,%lf,%lf,%d", &hsv.h, &hsv.s, &hsv.v, &target) == 4) {
            if(ts->count >= MAX_TRAINING_COUNT) break;

            if (ts->count >= MAX_TRAINING_COUNT) {
            fprintf(stderr, "Exceeded maximum training set size (%d)\n", MAX_TRAINING_COUNT);
            break;
            }

            add_datapoint(ts, &hsv, target);
        } else {
            fprintf(stderr, "Failed to parse line %d: %s", line_num + 1, line);
        }

        line_num++;
    }

    fclose(file);
}

int is_file_empty(const char* filename) {
  struct stat st;
  if (stat(filename, &st) != 0) {
      return 1;
  }
  return st.st_size == 0;
}

void write_training_set(const struct ColourTrainingSet *ts) {
    char filename[1024] = "color_training_set.csv";
    FILE* file = fopen(filename, "a");
    if (!file) {
        perror("Error opening file");
    }

    if (is_file_empty(filename)) {
        fprintf(file, "H,S,V,Target\n");
    }

    for (int i = 0; i < ts->count; ++i) {
        struct ColourHSV hsv = ts->data[i];
        int target_int = (int)ts->targets[i];
        fprintf(file, "%.3f,%.3f,%.3f,%d\n", hsv.h, hsv.s, hsv.v, target_int);
    }

    fclose(file);
}