#include "roboUtils.h"

void convert_to_metric(struct RoboAI *ai) {
  ai->st.bvxm = ai->st.bvx * CM_PER_PIXEL_X;		       // Ball velocity vector
	ai->st.bmxm = ai->st.bmx * CM_PER_PIXEL_X;		       // Ball motion vector
	ai->st.bdxm = ai->st.bdx * CM_PER_PIXEL_X;               // Ball heading direction (from blob shape)
	ai->st.svxm = ai->st.svx * CM_PER_PIXEL_X;		       // Current self [vx vy]
	ai->st.smxm = ai->st.smx * CM_PER_PIXEL_X;		       // Self motion vector
	ai->st.sdxm = ai->st.sdx * CM_PER_PIXEL_X;               // Self heading direction (from blob shape)
	ai->st.ovxm = ai->st.ovx * CM_PER_PIXEL_X;		       // Current opponent [vx vy]
	ai->st.omxm = ai->st.omx * CM_PER_PIXEL_X;		       // Opponent motion vector
	ai->st.odxm = ai->st.odx * CM_PER_PIXEL_X;               // Opponent heading direction (from blob shape)

  ai->st.bvym = ai->st.bvy * CM_PER_PIXEL_Y;			       // Ball velocity vector
	ai->st.bmym = ai->st.bmy * CM_PER_PIXEL_Y;			       // Ball motion vector
	ai->st.bdym = ai->st.bdy * CM_PER_PIXEL_Y;                // Ball heading direction (from blob shape)
	ai->st.svym = ai->st.svy * CM_PER_PIXEL_Y;			       // Current self [vx vy]
	ai->st.smym = ai->st.smy * CM_PER_PIXEL_Y;			       // Self motion vector
	ai->st.sdym = ai->st.sdy * CM_PER_PIXEL_Y;                // Self heading direction (from blob shape)
	ai->st.ovym = ai->st.ovy * CM_PER_PIXEL_Y;			       // Current opponent [vx vy]
	ai->st.omym = ai->st.omy * CM_PER_PIXEL_Y;			       // Opponent motion vector
	ai->st.odym = ai->st.ody * CM_PER_PIXEL_Y;                // Opponent heading direction (from blob shape)

  if (ai->st.ball != NULL) {
    ai->st.bpxm = ai->st.ball->cx * CM_PER_PIXEL_X;
    ai->st.bpym = ai->st.ball->cy * CM_PER_PIXEL_Y;
  }
  if (ai->st.self != NULL) {
    ai->st.spxm = ai->st.self->cx * CM_PER_PIXEL_X;
    ai->st.spym = ai->st.self->cy * CM_PER_PIXEL_Y;
  }
  if (ai->st.opp != NULL) {
    ai->st.opxm = ai->st.opp->cx * CM_PER_PIXEL_X;
    ai->st.opym = ai->st.opp->cy * CM_PER_PIXEL_Y;
  }
}

double normalize_angle(double a) {
    a = fmod(a, 2*M_PI);
    if (a < 0) a += 2*M_PI;
    return a;
}

double angle_diff(double angle, double target_angle) {
    angle = normalize_angle(angle);
    target_angle = normalize_angle(target_angle);

    double diff = fmod(target_angle - angle  + M_PI, 2*M_PI);
    if (diff < 0) diff += 2*M_PI;
    return diff - M_PI;
}

void determine_facing(struct RoboAI *ai) {
  if (ai->st.driving_dir == 1) {
    ai->st.fxm = ai->st.svxm;
    ai->st.fym = ai->st.svym;
  } else {
    // When driving slow, use the last facing direction to determine which way our bot is facing.
    if (dot(ai->st.fxm, ai->st.fym, ai->st.sdxm, ai->st.sdym) < 0) {
      ai->st.fxm = ai->st.sdxm * -1;
      ai->st.fym = ai->st.sdym * -1;
    } else {
      ai->st.fxm = ai->st.sdxm;
      ai->st.fym = ai->st.sdym;
    }
  }
  normalize_vector(&ai->st.fxm, &ai->st.fym);
  ai->st.fa = atan2(ai->st.fym, ai->st.fxm);
}

double dot(double x1, double y1, double x2, double y2) {
  return x1 * x2 + y1 * y2;
}

double norm(double x, double y) {
  return sqrt(dot(x, y, x, y));
}

void normalize_vector(double *x, double *y) {
    double mag = sqrt((*x)*(*x) + (*y)*(*y));
    if (mag > 1e-6) { *x /= mag; *y /= mag; }
}

// Helper function that returns the goal position based on the side the bot is on
int calculateGoalPosition(struct RoboAI *ai, double *goal_x, double *goal_y, int team) {
  // Side 0: Left side bot, goal is on the right
  // Side 1: Right side bot, goal is on the left
  if (ai->st.side == 0) {
        // Left side bot, goal is on the right
        *goal_x = sx - 1;
        if (team == 1) {
          *goal_x = 0;
        }
    } else {
        // Right side bot, goal is on the left
        *goal_x = 0;
        if (team == 1) {
          *goal_x = sx - 1;
        }
    }
  // Assumption that the goal is in the middle of the field vertically
  *goal_y = sy / 2.0;
  return 1;
}

// This helper calculates the vector between the ball and the goal with the goal being the forward direction
// Team = 0 (our), 1 (opp)
// Returns: 1 on success, 0 on failure (if ball position not available)
// Output parameters: goal_x, goal_y - goal center coordinates
//                    vec_x, vec_y - normalized vector from ball to goal
int calculateShootingVector(struct RoboAI *ai, double *goal_x, double *goal_y, double *vec_x, double *vec_y, int team) {
    // Check if ball position is available
    if (ai->st.ball == NULL || ai->st.ballID == 0) {
        // Ball position not available, return failure as we can't calculate a shooting vector
        return 0;
    }
    
    // Get ball's current position
    double ball_x = ai->st.ball->cx;
    double ball_y = ai->st.ball->cy;
    
    // Determine goal position based on which side we're on
    // side=0: bot's own side is left, goal to score on is on the right (x = sx)
    // side=1: bot's own side is right, goal to score on is on the left (x = 0)
    // Goal center is at the middle of the field vertically (y = sy/2)

    if (ai->st.side == 0) {
        // Left side bot, goal is on the right
        *goal_x = sx - 1;
        if (team == 1) {
          *goal_x = 0;
        }
    } else {
        // Right side bot, goal is on the left
        *goal_x = 0;
        if (team == 1) {
          *goal_x = sx - 1;
        }
    }

    // Assumption that the goal is in the middle of the field vertically
    *goal_y = sy / 2.0;
    
    // Calculate vector from ball to goal
    double dx = *goal_x - ball_x;
    double dy = *goal_y - ball_y;
    
    // Normalize the vector
    double magnitude = sqrt(dx * dx + dy * dy);
    if (magnitude < 1e-6) {
        // Ball is already at goal (shouldn't happen, but handle it)
        *vec_x = 0;
        *vec_y = 0;
        return 0;
    }
    
    *vec_x = dx / magnitude;
    *vec_y = dy / magnitude;
    
    return 1;
}

// Calculate the vector form from the bot to a target point
int calculateTargetPointVector(struct RoboAI *ai, double *targetPointX, double *targetPointY, double *vectorX, double *vectorY, int team) {
  double dx;
  double dy;
  if (team == 0) {
    dx = *targetPointX - ai->st.self->cx;
    dy = *targetPointY - ai->st.self->cy;
  } else {
    dx = *targetPointX - ai->st.opp->cx;
    dy = *targetPointY - ai->st.opp->cy;
  }
  // Normalize the vector
  double magnitude = sqrt(dx * dx + dy * dy);
  *vectorX = dx / magnitude;
  *vectorY = dy / magnitude;
  return 1;
}

// Calculate if two points are within a given distance epsilon
// Returns: 1 on success, 0 on failure (if points not available)
int calculatePointsWithinEpsilon(double *x1, double *y1, double *x2, double *y2, double epsilon) {
    // Calculate the distance between the two points with 2 norm
    double distance = sqrt(pow(*x1 - *x2, 2) + pow(*y1 - *y2, 2));
    printf("Distance: %f\n", distance);
    // Check if the distance is within the epsilon
    if (distance <= epsilon) {
        return 1;
    }
    return 0;
}

// Calculate if two headings are within a given angle epsilon (can be radian or degree just pick one and modify this function)
int calculateHeadingDifference(double *heading1, double *heading2, double epsilon) {
    // Calculate the difference between the two headings
    double difference = fabs(*heading1 - *heading2);
    // Check if the difference is within the epsilon
    if (difference <= epsilon) {
        return 1;
    }
    return 0;
}

// Some helper to calculate the ratio of wheel speed needed to turn an arc of a circle of diameter x
int calculateWheelSpeedRatio(double *arcDiameter, double *ratio) {
  // lets trial and error how much torque the wheels have before we fill this out 
  return 1;
}

void update_vars(struct RoboAI *ai)
{ 
  int oteam;
  if (ai->st.side == 0) {
    oteam = 1;
  }  else {
    oteam = 0;
  }
    // update facing direction
    determine_facing(ai);

    // update goal position
    calculateGoalPosition(ai, &(ai->st.goalx), &(ai->st.goaly), ai->st.side);
    calculateGoalPosition(ai, &(ai->st.ogoalx), &(ai->st.ogoaly), oteam);

    // update shooting vector
    int ok1 = calculateShootingVector(ai, &(ai->st.goalx), &(ai->st.goaly), &ai->st.shootingVectorX, &ai->st.shootingVectorY, oteam);
    if (!ok1) {
        // ball not visible — set vector to zero
        ai->st.shootingVectorX = 0;
        ai->st.shootingVectorY = 0;
    }

    int ok2 = calculateShootingVector(ai, &(ai->st.ogoalx), &(ai->st.ogoaly), &ai->st.oShootingVectorX, &ai->st.oShootingVectorY, oteam);
    if (!ok2) {
        // ball not visible — set vector to zero
        ai->st.oShootingVectorX = 0;
        ai->st.oShootingVectorY = 0;
    }
}
