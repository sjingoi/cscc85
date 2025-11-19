#include "roboUtils.h"

void normalize_vector(double *x, double *y) {
    double mag = sqrt((*x)*(*x) + (*y)*(*y));
    if (mag > 1e-6) { *x /= mag; *y /= mag; }
}

// Helper function that returns the goal position based on the side the bot is on
int calculateGoalPosition(struct RoboAI *ai, double *goal_x, double *goal_y) {
  // Side 0: Left side bot, goal is on the right
  // Side 1: Right side bot, goal is on the left
  if (ai->st.side == 0) {
    *goal_x = sx - 1;
  } else {
    // Right side bot, goal is on the left
    *goal_x = 0;
  }
  // Assumption that the goal is in the middle of the field vertically
  *goal_y = sy / 2.0;
  return 1;
}

// This helper calculates the vector between the ball and the goal with the goal being the forward direction
// Returns: 1 on success, 0 on failure (if ball position not available)
// Output parameters: goal_x, goal_y - goal center coordinates
//                    vec_x, vec_y - normalized vector from ball to goal
int calculateShootingVector(struct RoboAI *ai, double *goal_x, double *goal_y, double *vec_x, double *vec_y) {
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
    } else {
        // Right side bot, goal is on the left
        *goal_x = 0;
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
int calculateTargetPointVector(struct RoboAI *ai, double *targetPointX, double *targetPointY, double *vectorX, double *vectorY) {
  double dx = *targetPointX - ai->st.self->cx;
  double dy = *targetPointY - ai->st.self->cy;
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