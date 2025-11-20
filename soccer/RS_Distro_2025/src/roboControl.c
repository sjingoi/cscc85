#include "roboControl.h"
#include "roboUtils.h"

void turn_radius(int pw, double turn_rad, int dir, struct RoboAI *ai) {
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

void move_forward(int pw, struct RoboAI *ai) {
  if (norm(ai->st.svxm, ai->st.svym) > 1.5 && pw > 10) {
    ai->st.driving_dir = 1;
  } else {
    ai->st.driving_dir = 0;
  }
  BT_drive(MOTOR_A, MOTOR_D, pw);
}

void stop_moving(struct RoboAI *ai) {
  ai->st.driving_dir = 0;
  BT_all_stop(0);
}

void turn_left(int pw) {
    BT_turn(MOTOR_A, -pw, MOTOR_D, pw);
}

void turn_right(int pw) {
    BT_turn(MOTOR_A, pw, MOTOR_D, -pw);
}

int turn_towards_dir(struct RoboAI *ai, double t_dir_x, double t_dir_y) {

  double sx = ai->st.sdx;
  double sy = ai->st.sdy;
  double theta_th = 0.1;      // cos(angle threshold)
  int turn_pw     = 25;

  normalize_vector(&t_dir_x, &t_dir_y);
  normalize_vector(&sx, &sy);

  double c_theta = abs(t_dir_x * sx + t_dir_y * sy);

  // if (c_theta < 0) {
  //   fprintf(stderr, "[201] Facing away, turn 180.\n");
  //   turn_right(50);
  //   return 0; // Not aligned
  // }

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