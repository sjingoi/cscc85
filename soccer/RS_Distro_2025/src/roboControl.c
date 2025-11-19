#include "roboControl.h"
#include "roboUtils.h"

void turn_radius(int pw, double turn_rad, int dir) {
  double wheel_sep = 12.0;
  double wheel_turn_rad = turn_rad - (wheel_sep / 2);
  double p_outer = 1;
  double p_inner = (turn_rad > 1000) ? 1 : wheel_turn_rad / (wheel_turn_rad + wheel_sep);

  printf("Turning with power po %f pi %f: ", p_outer, p_inner);

  if (dir == 1) {
    BT_turn(MOTOR_A, pw * p_outer, MOTOR_D, pw * p_inner * 0.95);
  } else {
    BT_turn(MOTOR_A, pw * p_inner, MOTOR_D, pw * p_outer * 0.95);
  }
}

void move_forward(int pw) {
    BT_drive(MOTOR_A, MOTOR_D, pw);
}

void turn_left(int pw) {
    BT_turn(MOTOR_A, -pw, MOTOR_D, pw);
}

void turn_right(int pw) {
    BT_turn(MOTOR_A, pw, MOTOR_D, -pw);
}

int turn_towards_dir(struct RoboAI *ai, double t_dir_x, double t_dir_y) {

  struct blob *my_bot = ai->st.self;
  double sx = my_bot->dx;
  double sy = my_bot->dy;
  double theta_th = 0.85;      // cos(angle threshold)
  int turn_pw     = 30;

  normalize_vector(&t_dir_x, &t_dir_y);
  normalize_vector(&sx, &sy);

  double c_theta = t_dir_x * sx + t_dir_y * sy;

  if (c_theta < 0) {
    fprintf(stderr, "[201] Facing away, turn 180.\n");
    turn_right(50);
    return 0; // Not aligned
  }

  if (c_theta < theta_th) {
    double cross = sx * t_dir_x - sy * t_dir_y;
    if (cross < 0) {
        turn_left(turn_pw);
    } else {
        turn_right(turn_pw);
    }
    return 0; // Not aligned
  } else {
    return 1; // Aligned
  }
}