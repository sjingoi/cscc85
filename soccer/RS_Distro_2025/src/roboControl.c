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
void go_to_point(struct RoboAI *ai, int power, double turn_smoothness, double target_x, double target_y) {
  double dir_x = target_x - ai->st.spxm; // X direction to 
  double dir_y = target_y - ai->st.spym;
  normalize_vector(&dir_x, &dir_y);
  double angle_delta = angle_diff(ai->st.fa, atan2(dir_y, dir_x)); 
  turn_radius(power, turn_smoothness/(angle_delta * angle_delta), (angle_delta < 0) ? 0 : 1, ai);
}

void safe_go_to_point(struct RoboAI *ai, int power, double turn_smoothness, double target_x, double target_y) {
  // check if the enemy is going to be along the path along the x axis or the y axis
  double start_x = ai->st.spxm;
  double start_y = ai->st.spym;

  // if we detect a potential collision then move along the x or y axis instead so the next iteration detects no collision
  double dx = target_x - start_x;
  double dy = target_y - start_y;

  double seg_length = point_distance(start_x, start_y, target_x, target_y);

  // calculate the unit direction
  double dir_x = dx / seg_length;
  double dir_y = dy / seg_length;

  // "draw" a circle around the enemy with some radius in metric
  double enemy_x = ai->st.opxm;
  double enemy_y = ai->st.opym;
  double enemy_radius = 20.0; // 10 cm radius

  // vector from enemy center to segment start
  double f_x = start_x - enemy_x;
  double f_y = start_y - enemy_y;

  // solve quadratic equation
  double b = 2 * (dir_x * f_x + dir_y * f_y);
  double c = (f_x * f_x + f_y * f_y) - enemy_radius * enemy_radius;
  double discriminant = b * b - 4 * c;

  bool intersects = false;
  if (discriminant >= 0) {
    double sqrt_disc = sqrt(discriminant);
    double t1 = (-b - sqrt_disc) / 2;
    double t2 = (-b + sqrt_disc) / 2;
    if ((t1 >= 0 && t1 <= seg_length) || (t2 >= 0 && t2 <= seg_length)) {
      intersects = true;
    }
  }

  // if we are not interesecting then go to point normally
  if (!intersects) {
    go_to_point(ai, power, turn_smoothness, target_x, target_y);
    return;
  }

  printf("Avoiding collision...\n");
  
  // we will default to moving along the axis that is further from us
  if (dx > dy) {
    // move along y axis if the enemy is too close to x
    if (fabs(enemy_x - start_x) < enemy_radius) {
      go_to_point(ai, power, turn_smoothness, target_x, start_y);
    }
    go_to_point(ai, power, turn_smoothness, start_x, target_y);
  } else {
    // move along x axis if the enemy is too close to our y
    if (fabs(enemy_y - start_y) < enemy_radius) {
      go_to_point(ai, power, turn_smoothness, start_x, target_y);
    }
    go_to_point(ai, power, turn_smoothness, target_x, start_y);
  }
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

int detect_ball(struct RoboAI *ai, double forward_dist, double lateral_dist, int team) {

  // check if all are fixed
  if (!ai || !ai->st.ballID || ai->st.ball == NULL) return 0;

  // positions and forward vector
  double bx = ai->st.bpxm;
  double by = ai->st.bpym;
  
  double sx, sy, fx, fy;

  if (team == 0) {
    sx = ai->st.spxm;
    sy = ai->st.spym;
    fx = ai->st.fxm;
    fy = ai->st.fym;
  } else {
    sx = ai->st.opxm;
    sy = ai->st.opym;
    fx = ai->st.odxm;
    fy = ai->st.odym;
  }

  // bot to ball vector
  double vx = bx - sx;
  double vy = by - sy;

  // perpendicular vector
  double nx = -fy;
  double ny = fx;

  double f2 = fx*fx + fy*fy;
  if (f2 <= 0) return 0;

  // project onto robot basis
  double forward = (vx*fx + vy*fy) / f2;
  double lateral = (vx*nx + vy*ny) / f2;

  // ball behind check
  if (forward <= 0) return 0;

  if (forward > forward_dist) return 0; // too far ahead
  if (lateral > lateral_dist || lateral < -lateral_dist) return 0; // too far laterally

  return 1; // all requirements fulfilled
}
