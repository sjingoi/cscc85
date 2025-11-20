#include "roboControl.h"
#include "roboUtils.h"

void turn_radius(int pw, double turn_rad, int dir, struct RoboAI *ai) {
  double wheel_sep = 12.0;
  double wheel_turn_rad = turn_rad - (wheel_sep / 2);
  double p_outer = 1;
  double p_inner = (turn_rad > 1000) ? 1 : wheel_turn_rad / (wheel_turn_rad + wheel_sep);
  
  if (turn_rad > 10 && norm(ai->st.svxm, ai->st.svym) > 1.0 && pw > 10) {
    ai->st.driving_dir = 1;
  } else {
    ai->st.driving_dir = 0;
  }

  if (dir == 1) {
    BT_turn(MOTOR_A, pw * p_outer, MOTOR_D, pw * p_inner * 0.95);
  } else {
    BT_turn(MOTOR_A, pw * p_inner, MOTOR_D, pw * p_outer * 0.95);
  }
}

/**
 * Goes to a point
 */
void go_to_point(struct RoboAI *ai, int power, double target_x, double target_y) {
  double dir_x = target_x - ai->st.spxm; // X direction to 
  double dir_y = target_y - ai->st.spym;
  normalize_vector(&dir_x, &dir_y);
  double angle_delta = angle_diff(ai->st.fa, atan2(dir_y, dir_x)); 
  double a = 10; // P-controller variable
  turn_radius(power, a/(angle_delta * angle_delta), (angle_delta < 0) ? 0 : 1, ai);
}

void move_forward(int pw, struct RoboAI *ai) {
  if (norm(ai->st.svxm, ai->st.svym) > 1.5 && pw > 10) {
    ai->st.driving_dir = 1;
  } else {
    ai->st.driving_dir = 0;
  }
  BT_turn(MOTOR_A, pw, MOTOR_D, pw*0.95);
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

// double alignment(struct RoboAI *ai, double t_dir_x, double t_dir_y) {
//   double t_angle = atan2(t_dir_y, t_dir_x);
//   double c_theta = dot(ai->st.fxm, ai->st.fym, t_dir_x, t_dir_y);
//   if (atan(c_theta) * 180 / 3.1415 < )
// }

int turn_towards_dir(struct RoboAI *ai, double t_dir_x, double t_dir_y) {
  printf("Im here.\n");
    double sx = ai->st.sdx;
    double sy = ai->st.sdy;

    double theta_th = 0.99; // cos(angle) threshold (11 degrees)
    int turn_pw = 15;

    normalize_vector(&t_dir_x, &t_dir_y);
    normalize_vector(&sx, &sy);

    double c_theta = t_dir_x * sx + t_dir_y * sy;

    // If facing more than 90 away, just turn until we're no longer opposite
    if (c_theta < 0) {
        double cross = sx * t_dir_y - sy * t_dir_x;
        if (cross < 0) {
            turn_left(turn_pw);
        } else {
            turn_right(turn_pw);
        }
        return 0; // Not aligned yet
    }

    // Normal alignment check
    if (c_theta < theta_th) {
        double cross = sx * t_dir_y - sy * t_dir_x;
        if (cross < 0) {
            turn_left(turn_pw);
        } else {
            turn_right(turn_pw);
        }
        return 0; // Not aligned yet
    }

    return 1; // Aligned
}
