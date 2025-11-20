/*

  CSC C85 - Embedded Systems - Project # 1 - EV3 Robot Localization
  
 This file provides the implementation of all the functionality required for the EV3
 robot localization project. Please read through this file carefully, and note the
 sections where you must implement functionality for your bot. 
 
 You are allowed to change *any part of this file*, not only the sections marked
 ** TO DO **. You are also allowed to add functions as needed (which must also
 be added to the header file). However, *you must clearly document* where you 
 made changes so your work can be properly evaluated by the TA.

 NOTES on your implementation:

 * It should be free of unreasonable compiler warnings - if you choose to ignore
   a compiler warning, you must have a good reason for doing so and be ready to
   defend your rationale with your TA.
 * It must be free of memory management errors and memory leaks - you are expected
   to develop high wuality, clean code. Test your code extensively with valgrind,
   and make sure its memory management is clean.
 
 In a nutshell, the starter code provides:
 
 * Reading a map from an input image (in .ppm format). The map is bordered with red, 
   must have black streets with yellow intersections, and buildings must be either
   blue, green, or be left white (no building).
   
 * Setting up an array with map information which contains, for each intersection,
   the colours of the buildings around it in ** CLOCKWISE ** order from the top-left.
   
 * Initialization of the EV3 robot (opening a socket and setting up the communication
   between your laptop and your bot)
   
 What you must implement:
 
 * All aspects of robot control:
   - Finding and then following a street
   - Recognizing intersections
   - Scanning building colours around intersections
   - Detecting the map boundary and turning around or going back - the robot must not
     wander outside the map (though of course it's possible parts of the robot will
     leave the map while turning at the boundary)

 * The histogram-based localization algorithm that the robot will use to determine its
   location in the map - this is as discussed in lecture.

 * Basic robot exploration strategy so the robot can scan different intersections in
   a sequence that allows it to achieve reliable localization
   
 * Basic path planning - once the robot has found its location, it must drive toward a 
   user-specified position somewhere in the map.

 --- OPTIONALLY but strongly recommended ---
 
  The starter code provides a skeleton for implementing a sensor calibration routine,
 it is called when the code receives -1  -1 as target coordinates. The goal of this
 function should be to gather informatin about what the sensor reads for different
 colours under the particular map/room illumination/battery level conditions you are
 working on - it's entirely up to you how you want to do this, but note that careful
 calibration would make your work much easier, by allowing your robot to more
 robustly (and with fewer mistakes) interpret the sensor data into colours. 
 
   --> The code will exit after calibration without running localization (no target!)
       SO - your calibration code must *save* the calibration information into a
            file, and you have to add code to main() to read and use this
            calibration data yourselves.
   
 What you need to understand thoroughly in order to complete this project:
 
 * The histogram localization method as discussed in lecture. The general steps of
   probabilistic robot localization.

 * Sensors and signal management - your colour readings will be noisy and unreliable,
   you have to handle this smartly
   
 * Robot control with feedback - your robot does not perform exact motions, you can
   assume there will be error and drift, your code has to handle this.
   
 * The robot control API you will use to get your robot to move, and to acquire 
   sensor data. Please see the API directory and read through the header files and
   attached documentation
   
 Starter code:
 F. Estrada, 2018 - for CSC C85 
 
*/

#include "EV3_Localization.h"
#include "./EV3_RobotControl/bytecodes.h"
#include <stdlib.h>

#include "colour.h"
#include "localization.h"

int map[400][4];            // This holds the representation of the map, up to 20x20
                            // intersections, raster ordered, 4 building colours per
                            // intersection.
int sx, sy;                 // Size of the map (number of intersections along x and y)
double beliefs[400][4];     // Beliefs for each location and motion direction
struct LocalizationMap lm;

int main(int argc, char *argv[])
{
 char mapname[1024];
 int dest_x, dest_y, rx, ry;
 unsigned char *map_image;
 
 memset(&map[0][0],0,400*4*sizeof(int));
 sx=0;
 sy=0;
 
 if (argc<4)
 {
  fprintf(stderr,"Usage: EV3_Localization map_name dest_x dest_y\n");
  fprintf(stderr,"    map_name - should correspond to a properly formatted .ppm map image\n");
  fprintf(stderr,"    dest_x, dest_y - target location for the bot within the map, -1 -1 calls calibration routine\n");
  exit(1);
 }
 strcpy(&mapname[0],argv[1]);
 dest_x=atoi(argv[2]);
 dest_y=atoi(argv[3]);

 if (dest_x==-1&&dest_y==-1)
 {
  calibrate_sensor();
  exit(1);
 }

 /******************************************************************************************************************
  * OPTIONAL TO DO: If you added code for sensor calibration, add just below this comment block any code needed to
  *   read your calibration data for use in your localization code. Skip this if you are not using calibration
  * ****************************************************************************************************************/

 struct ColourTrainingSet ts;
 ts.count = 0;

 load_training_set(&ts);

 // Your code for reading any calibration information should not go below this line //
 
 map_image=readPPMimage(&mapname[0],&rx,&ry);
 if (map_image==NULL)
 {
  fprintf(stderr,"Unable to open specified map image\n");
  exit(1);
 }
 
 if (parse_map(map_image, rx, ry)==0)
 { 
  fprintf(stderr,"Unable to parse input image map. Make sure the image is properly formatted\n");
  free(map_image);
  exit(1);
 }
 memcpy(&(lm.instersections), &map, sizeof(map));
 memcpy(&(lm.beliefs), &beliefs, sizeof(beliefs));
 lm.sx = sx;
 lm.sy = sy;

 if (dest_x<0||dest_x>=sx||dest_y<0||dest_y>=sy)
 {
  fprintf(stderr,"Destination location is outside of the map\n");
  free(map_image);
  exit(1);
 }

 // Initialize beliefs - uniform probability for each location and direction
 for (int j=0; j<sy; j++)
  for (int i=0; i<sx; i++)
  {
   lm.beliefs[i+(j*sx)].north=1.0/(double)(sx*sy*4);
   lm.beliefs[i+(j*sx)].east=1.0/(double)(sx*sy*4);
   lm.beliefs[i+(j*sx)].south=1.0/(double)(sx*sy*4);
   lm.beliefs[i+(j*sx)].west=1.0/(double)(sx*sy*4);
  }

  // NXTCOLOR scan1[4] = {GREENCOLOR, WHITECOLOR, BLUECOLOR, WHITECOLOR};
  // enum RelativeDirection dir2 = FORWARD;
  // NXTCOLOR scan2[4] = {GREENCOLOR, BLUECOLOR, BLUECOLOR, WHITECOLOR};
  // enum RelativeDirection dir3 = FORWARD;
  // NXTCOLOR scan3[4] = {GREENCOLOR, WHITECOLOR, GREENCOLOR, BLUECOLOR};

  // localize_init(&lm, scan1);
  // print_beliefs(&lm);
  // localize(&lm, scan2, dir2);
  // print_beliefs(&lm);
  // localize(&lm, scan3, dir3);
  // print_beliefs(&lm);

  // exit(0);

 // Open a socket to the EV3 for remote controlling the bot.
 if (BT_open(HEXKEY)!=0)
 {
  fprintf(stderr,"Unable to open comm socket to the EV3, make sure the EV3 kit is powered on, and that the\n");
  fprintf(stderr," hex key for the EV3 matches the one in EV3_Localization.h\n");
  free(map_image);
  exit(1);
 }

 fprintf(stderr,"All set, ready to go!\n");
 
/*******************************************************************************************************************************
 *
 *  TO DO - Implement the main localization loop, this loop will have the robot explore the map, scanning intersections and
 *          updating beliefs in the beliefs array until a single location/direction is determined to be the correct one.
 * 
 *          The beliefs array contains one row per intersection (recall that the number of intersections in the map_image
 *          is given by sx, sy, and that the map[][] array contains the colour indices of buildings around each intersection.
 *          Indexing into the map[][] and beliefs[][] arrays is by raster order, so for an intersection at i,j (with 0<=i<=sx-1
 *          and 0<=j<=sy-1), index=i+(j*sx)
 *  
 *          In the beliefs[][] array, you need to keep track of 4 values per intersection, these correspond to the belief the
 *          robot is at that specific intersection, moving in one of the 4 possible directions as follows:
 * 
 *          beliefs[i][0] <---- belief the robot is at intersection with index i, facing UP
 *          beliefs[i][1] <---- belief the robot is at intersection with index i, facing RIGHT
 *          beliefs[i][2] <---- belief the robot is at intersection with index i, facing DOWN
 *          beliefs[i][3] <---- belief the robot is at intersection with index i, facing LEFT
 * 
 *          Initially, all of these beliefs have uniform, equal probability. Your robot must scan intersections and update
 *          belief values based on agreement between what the robot sensed, and the colours in the map. 
 * 
 *          You have two main tasks these are organized into two major functions:
 * 
 *          robot_localization()    <---- Runs the localization loop until the robot's location is found
 *          go_to_target()          <---- After localization is achieved, takes the bot to the specified map location
 * 
 *          The target location, read from the command line, is left in dest_x, dest_y
 * 
 *          Here in main(), you have to call these two functions as appropriate. But keep in mind that it is always possible
 *          that even if your bot managed to find its location, it can become lost again while driving to the target
 *          location, or it may be the initial localization was wrong and the robot ends up in an unexpected place - 
 *          a very solid implementation should give your robot the ability to determine it's lost and needs to 
 *          run localization again.
 *
 *******************************************************************************************************************************/  
 
 // HERE - write code to call robot_localization() and go_to_target() as needed, any additional logic required to get the
 //        robot to complete its task should be here.
 
  // int black_angle_left = find_black(&ts, 45, 1);
  // int black_angle_right = find_black(&ts, 45, 0);

  // int dir;
  // int angle;
  // if (black_angle_left < black_angle_right) {
  //   dir = 1;
  //   angle = black_angle_left;
  // } else {
  //   dir = 0;
  //   angle = black_angle_right;
  // }

  // if (angle == INT_MAX) {
  //   printf("Could not find black within this angle.");
  // } else {
  //   printf("Found black at dir: %d, angle: %d\n", dir, angle);
  //   printf("Angles: %d %d\n", black_angle_left, black_angle_right);
  // }

  // start_colour_thread(&ts);

  // while(1) {
  //   struct ColourConfidence c = read_colour();
  //   printf("Colour detected: ");
  //   print_colour_nxt(c.colour);
  //   printf("Confidence: %f\n", c.confidence);
  //   sleep(0.1);
  // }

// BT_motor_port_start(MOTOR_D, 10); // big disc
// drive_until_street(&ts);
// if (drive_along_street(&ts, 0) == 0) {
//   drive_until_street(&ts);
// }
// drive_along_street(&ts, 0);

robot_localization(0, 0, 0, &ts);
go_to_target(dest_x, dest_y, &ts);
// while (1) {
// read_nxt_color(&ts);
// print_colour_nxt(read_nxt_color(&ts));
// }

// int a, b, c, d;
// scan_intersection(&a, &b, &c, &d, &ts);

 // Cleanup and exit - DO NOT WRITE ANY CODE BELOW THIS LINE
 BT_close();
 free(map_image);
 exit(0);
}

// int perimeter_scan(const struct ColourTrainingSet *ts) {
//   /*
//     * This function carries out a perimeter scan of the map, making the robot drive around the entire map perimeter
//     * while following the street, and scanning intersections as it goes. The goal of this function is to gather
//     * information about all intersections in the map to allow reliable localization.
//     */
  
//   // Return a zero to indicate failure

//   // Main perimeter scan loop

  
// }

int drive_until_street(const struct ColourTrainingSet *ts)
{
  // Start driving forward
  const int forward_power = 40;
  BT_drive(MOTOR_A, MOTOR_B, -forward_power);

  // While driving, read the colour sensor
  while (1) {
    NXTCOLOR c = read_nxt_color(ts);
    print_colour_nxt(c);
    // Found black so start running the find_street function to 
    if (c == BLACKCOLOR) {
      printf("Found street\n");
      if (find_street(ts) == 1) {
        break;
      }
      // Failed so continue driving
    }
    if (c == REDCOLOR) {
      printf("Hit red boundary\n");
      // Hit red boundary, perform a right turn then continue driving 
      // This should always result in a street or getting out of corner
      turn_at_intersection(0, 90);
    }
  }

  // Now perform alignment with a street though driving its enterity
  // drive along a street till red boundary success twice so it will drive along an entire street
  if (drive_along_street(ts, 1) == 1) {
    printf("Successfully scanned red once\n");
    if (drive_along_street(ts, 1) == 1) {
      printf("Successfully scanned red twice\n");
      return 0;
    } else {
      printf("Failed to do double red boundary drive, retrying\n");
      drive_until_street(ts);
    }
  } else {
    printf("Failed to drive along street, restarting search\n");
    // Restart motors and continue searching
    BT_drive(MOTOR_A, MOTOR_B, -forward_power);
    // Continue the search loop
    goto restart_search;
  }

restart_search:
  // Continue driving and searching for a street
  while (1) {
    NXTCOLOR c = read_nxt_color(ts);
    print_colour_nxt(c);
    if (c == BLACKCOLOR) {
      printf("Found street\n");
      if (find_street(ts) == 1) {
        break;
      }
      // Failed so continue driving
    }
    if (c == REDCOLOR) {
      printf("Hit red boundary\n");
      turn_at_intersection(0, 90);
    }
  }

  return 0;
}

int find_street(const struct ColourTrainingSet *ts)   
{
 /*
  * This function gets your robot onto a street, wherever it is placed on the map. You can do this in many ways, but think
  * about what is the most effective and reliable way to detect a street and stop your robot once it's on it.
  * 
  * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
  * bot after calling this function
  */   

  // rotate until you find black
  
  const int search_step_deg = 10;
  const int max_search_degree = 360;
  const int forward_power = 40;

  for (int angle = 0; angle < max_search_degree; angle += search_step_deg) {
    turn_at_intersection(0, search_step_deg); // rotate right in steps
    NXTCOLOR c = read_nxt_color(ts);
    if (c == BLACKCOLOR) {
      // found black, stop rotating
      BT_motor_port_stop(MOTOR_A | MOTOR_B, 1);
      // move forward a bit to ensure fully on street
      BT_drive(MOTOR_A, MOTOR_B, -forward_power);
      usleep(1000000); // move forward for 0.5 seconds
      BT_motor_port_stop(MOTOR_A | MOTOR_B, 1);
      return 1; // success
    }
  }
  

  BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);
  return(0);
}

int find_black(const struct ColourTrainingSet *ts, int max_angle, int turn_direction) {
  int epsilon = 0; 
  int turnPower = 52;
  int current_angle, angle_delta, turn_speed;
  int target_angle = (turn_direction == 0) ? max_angle : -1 * max_angle;
  int found_black = 0;

  BT_read_gyro(PORT_4, 1, &current_angle, &turn_speed);
  while (1) {
    BT_read_gyro(PORT_4, 0, &current_angle, &turn_speed);
    if (abs(current_angle) > 360) {
      printf("Skipping garbage angle: %d\n", current_angle);
      continue;
    } // Sometimes the angle is a garbage value.
    angle_delta = (current_angle - target_angle + 540) % 360 - 180;
    printf("Target: %d, Current: %d, Delta: %d\n", target_angle, current_angle, angle_delta);
    
    if (read_nxt_color(ts) == BLACKCOLOR) {
      BT_motor_port_stop(MOTOR_A, 1);
      BT_motor_port_stop(MOTOR_B, 1);
      found_black = 1;
      break;
    }

    if (abs(angle_delta) > epsilon) {
      int motor_power = turnPower * angle_delta / fabs(angle_delta);
      BT_turn(MOTOR_A, motor_power, MOTOR_B, -1 * motor_power);
    } else {
      BT_motor_port_stop(MOTOR_A, 1);
      BT_motor_port_stop(MOTOR_B, 1);
      break;
    }
  }

  // Reset position
  turn_at_intersection(1 - turn_direction, abs(current_angle));

  if (found_black) return abs(current_angle);
  return INT_MAX;
}

// if stop_at_intersection is 1, the function will stop at intersection boundary else it continues until red boundary
// we use red boundary for alignment as you get a longer street
int drive_along_street(const struct ColourTrainingSet *ts, int skip_yellow)
{
 /*
  * This function drives your bot along a street, making sure it stays on the street without straying to other pars of
  * the map. It stops at an intersection.
  * 
  * You can implement this in many ways, including a controller (PID for example), a neural network trained to track and
  * follow streets, or a carefully coded process of scanning and moving. It's up to you, feel free to consult your TA
  * or the course instructor for help carrying out your plan.
  * 
  * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
  * bot after calling this function.
  */   
  
  const int forward_power = 20;
  const int step_delay_us = 5000;
  const int search_step_deg = 3;   // rotation step size when searching (changed to 15° increments)
  const int max_search_deg = 60;   // maximum angle to search each side

  // Start driving forward
  BT_drive(MOTOR_A, MOTOR_B, -forward_power);

  // while (1) {

  //   NXTCOLOR col = read_nxt_color(ts);
  //   print_colour_nxt(col);
  // }
  while (1) {
    NXTCOLOR col = read_nxt_color(ts);

    if (col == REDCOLOR) {
      // on red
      printf("Hit red boundary, going back\n");
      turn_at_intersection(1, 180); // turn around
      BT_drive(MOTOR_A, MOTOR_B, 70);
      usleep(1500000);
      BT_motor_port_stop(MOTOR_A | MOTOR_B, 1);
      BT_drive(MOTOR_A, MOTOR_B, -forward_power);

      // if (stop_at_red == 1) {
      printf("Stopped at red\n");
      BT_motor_port_stop(MOTOR_A | MOTOR_B, 1);
      return 1;
      // }

      // usleep(1000000);
    }
    if (col == GREENCOLOR) {
      // on green
      printf("Hit green forward retry\n");
      BT_drive(MOTOR_A, MOTOR_B, -forward_power);
      //usleep(1000000);
      usleep(500000);
      printf("Done drive\n");
      col = read_nxt_color(ts);
    }

    print_colour_nxt(col);
    if (col == BLACKCOLOR) {
      // on black, keep going
      BT_drive(MOTOR_A, MOTOR_B, -forward_power);
      usleep(20000);
      continue;
    }

    if (col == YELLOWCOLOR) {
      // Do not stop if we are looking for a red boundary as we want to align with the street
      if (skip_yellow != 1) {
        BT_motor_port_stop(MOTOR_A | MOTOR_B, 1);
        return 2;
      }
      continue;
    }
    // Not black & not intersection => lost the line
    BT_motor_port_stop(MOTOR_A | MOTOR_B, 1);
    int scan_angle = 45;
    int black_angle_left = find_black(ts, scan_angle, 1);
    int black_angle_right = find_black(ts, scan_angle, 0);

    int dir;
    int angle;
    if (black_angle_left < black_angle_right) {
      dir = 1;
      angle = black_angle_left;
    } else {
      dir = 0;
      angle = black_angle_right;
    }

    if (angle > scan_angle + 3) {
      fprintf(stderr, "drive_along_street: lost line, unable to re-acquire within 45 deg\n");
      return 0;
    } else {
      printf("Found black at dir: %d, angle: %d\n", dir, angle);
      printf("Angles: %d %d\n", black_angle_left, black_angle_right);
    }

    turn_at_intersection(dir, angle + 2);
    BT_drive(MOTOR_A, MOTOR_B, -forward_power);
    usleep(200000);
    BT_motor_port_stop(MOTOR_A | MOTOR_B, 1);

    // After moving toward the line loop and continue following
  }

  // unreachable
  return 0;
}

int scan_intersection(NXTCOLOR *tl, NXTCOLOR *tr, NXTCOLOR *br, NXTCOLOR *bl, const struct ColourTrainingSet *ts)
{
 /*
  * This function carries out the intersection scan - the bot should (obviously) be placed at an intersection for this,
  * and the specific set of actions will depend on how you designed your bot and its sensor. Whatever the process, you
  * should make sure the intersection scan is reliable - i.e. the positioning of the sensor is reliably over the buildings
  * it needs to read, repeatably, and as the robot moves over the map.
  * 
  * Use the APIs sensor reading calls to poll the sensors. You need to remember that sensor readings are noisy and 
  * unreliable so * YOU HAVE TO IMPLEMENT SOME KIND OF SENSOR / SIGNAL MANAGEMENT * to obtain reliable measurements.
  * 
  * Recall your lectures on sensor and noise management, and implement a strategy that makes sense. Document your process
  * in the code below so your TA can quickly understand how it works.  printf("Gyro: %d %d\n", R, G);
  * 
  * Once your bot has read the colours at the intersection, it must return them using the provided pointers to 4 integer
  * variables:
  * 
  * tl - top left building colour
  * tr - top right building colour
  * br - bottom right building colour
  * bl - bottom left building colour
  * 
  * The function's return value can be used to indicate success or failure, or to notify your code of the bot's state
  * after this call.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

 // Return invalid colour values, and a zero to indicate failure (you will replace this with your code)

 // For this function we are assuming a call when intersection is just visible

 // Drive forward so that the bot is centered

 // We will rotate clockwise for 3 rotations so it reads in the order of:
 /*
 tr, br, bl, tl

 */

  BT_drive(MOTOR_A, MOTOR_B, -70);
  usleep(1050000);// go for 1 second
  BT_motor_port_stop(MOTOR_A | MOTOR_B, 1);

  // start reading colours
  turn_at_intersection(0, 45);

  NXTCOLOR a;
  NXTCOLOR interColors[4];

  int angles[4] = {0, 90, 180, 270}; // Example: 4 positions (adjust as needed)

  // Do 3 full rotations: CW, CCW, CW
  for (int i = 0; i < 4; i++) {
    NXTCOLOR a = read_nxt_color(ts);
    interColors[i] = a;
    printf("Pos %d: %d\n", i+1, a);
    turn_at_intersection(0, 90); // 90° per step, adjust if you want more positions
  }
  // Return alignment
  turn_at_intersection(1, 45);
  // Example: turn right on next street
  // turn_at_intersection(1, 90);

  // Assign to output variables in tl, tr, br, bl order
  *tr = interColors[0];
  *br = interColors[1];
  *bl = interColors[2];
  *tl = interColors[3];

  for (int i = 0; i < 4; i++) {
      print_colour_nxt(interColors[i]);
  }

  printf("Scanned colors: TR=%d, BR=%d, BL=%d, TL=%d\n", *tr, *br, *bl, *tl);

  return 0;
}

// turn angle is the amount of degrees to right or left doesnt take in negative as direction accounts for that
int turn_at_intersection(int turn_direction, int turn_angle)
{
 /*
  * This function is used to have the robot turn either left or right at an intersection (obviously your bot can not just
  * drive forward!). 
  * 
  * If turn_direction=0, turn right, else if turn_direction=1, turn left.
  * 
  * You're free to implement this in any way you like, but it should reliably leave your bot facing the correct direction
  * and on a street it can follow. 
  * 
  * You can use the return value to indicate success or failure, or to inform your code of the state of the bot
  */
  int epsilon = 0; 
  int turnPower = 52;
  int current_angle, angle_delta, turn_speed;
  int target_angle = (turn_direction == 0) ? turn_angle : -1 * turn_angle;
  // int target_angle = (target_angle + 360) % 360;

  BT_read_gyro(PORT_4, 1, &current_angle, &turn_speed);
  while (1) {
    BT_read_gyro(PORT_4, 0, &current_angle, &turn_speed);
    if (current_angle < INT_MIN / 2) {
      printf("Compensating for overflow");
      current_angle = current_angle - INT_MIN;
    }
    if (abs(current_angle) > 360) {
      printf("Skipping garbage angle: %d\n", current_angle);
      continue;
    } // Sometimes the angle is a garbage value.
    angle_delta = (current_angle - target_angle + 540) % 360 - 180;
    printf("Target: %d, Current: %d, Delta: %d\n", target_angle, current_angle, angle_delta);

    if (abs(angle_delta) > epsilon) {
      int motor_power = turnPower * angle_delta / fabs(angle_delta);
      BT_turn(MOTOR_A, motor_power, MOTOR_B, -1 * motor_power);
    } else {
      BT_motor_port_stop(MOTOR_A, 1);
      BT_motor_port_stop(MOTOR_B, 1);
      break;
    }
  }

  // int offset;
  // int currentAngle;
  // int turnSpeed;
  // bool overshot = false;

  // // Read the gyro from scratch and assume that we are zeroed (we can potentially call a street homing function first)
  // int epsilon = 2;
  
  // while (1) {
  //   BT_read_gyro(PORT_4, 0, &currentAngle, &turnSpeed);
  //   printf("Gyro: %d %d\n", currentAngle, turnSpeed);
  // }
  
  // // Keep turning until we hit within 0 degree of 90
  // while (abs(turn_angle - abs(currentAngle)) > 2) {
  //   // If we over shoot then rotate back the other direction
  //   if (abs(turn_angle) > abs(currentAngle)) {
  //     overshot = true;
  //     turnPower = turnPower * -1;
  //   }
  //   // We over did it on the turn when reversing from overshoot
  //   if (abs(turn_angle) < abs(currentAngle) && overshot) {
  //     overshot = false;
  //     turnPower = turnPower * -1;
  //   }
  //   // Right turn
  //   if (turn_direction == 0) {
  //     BT_turn(MOTOR_A, -turnPower, MOTOR_B, turnPower);
  //   } else {
  //     BT_turn(MOTOR_A, turnPower, MOTOR_B, -turnPower);
  //   }

  //   // Update the gyro value
  //   BT_read_gyro(PORT_4, 0, &currentAngle, &turnSpeed);
  // }

  // // Stop the motors
  // BT_motor_port_stop(MOTOR_A, 1);
  // BT_motor_port_stop(MOTOR_B, 1);
  
  return(0);
}

int robot_localization(int *robot_x, int *robot_y, int *direction, const struct ColourTrainingSet *ts)
{
 /*  This function implements the main robot localization process. You have to write all code that will control the robot
  *  and get it to carry out the actions required to achieve localization.
  *
  *  Localization process:
  *
  *  - Find the street, and drive along the street toward an intersection
  *  - Scan the colours of buildings around the intersection
  *  - Update the beliefs in the beliefs[][] array according to the sensor measurements and the map data
  *  - Repeat the process until a single intersection/facing direction is distintly more likely than all the rest
  * 
  *  * We have provided headers for the following functions:
  * 
  *  find_street()
  *  drive_along_street()
  *  scan_intersection()
  *  turn_at_intersection()
  * 
  *  You *do not* have to use them, and can write your own to organize your robot's work as you like, they are
  *  provided as a suggestion.
  * 
  *  Note that *your bot must explore* the map to achieve reliable localization, this means your intersection
  *  scanning strategy should not rely exclusively on moving forward, but should include turning and exploring
  *  other streets than the one your bot was initially placed on.
  * 
  *  For each of the control functions, however, you will need to use the EV3 API, so be sure to become familiar with
  *  it.
  * 
  *  In terms of sensor management - the API allows you to read colours either as indexed values or RGB, it's up to
  *  you which one to use, and how to interpret the noisy, unreliable data you're likely to get from the sensor
  *  in order to update beliefs.
  * 
  *  HOWEVER: *** YOU must document clearly both in comments within this function, and in your report, how the
  *               sensor is used to read colour data, and how the beliefs are updated based on the sensor readings.
  * 
  *  DO NOT FORGET - Beliefs should always remain normalized to be a probability distribution, that means the
  *                  sum of beliefs over all intersections and facing directions must be 1 at all times.
  * 
  *  The function receives as input pointers to three integer values, these will be used to store the estimated
  *   robot's location and facing direction. The direction is specified as:
  *   0 - UP
  *   1 - RIGHT
  *   2 - BOTTOM
  *   3 - LEFT
  * 
  *  The function's return value is 1 if localization was successful, and 0 otherwise.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

 // Return an invalid location/direction and notify that localization was unsuccessful (you will delete this and replace it
 // with your code).
 
  int iteration = 0;
  int it_amt = 100;
  int int_colors[it_amt][4];
  int turn_from_border = 0;

  enum RelativeDirection last_dir;

  for (int i = 0;; i++) {
    // Ensure we dont start off on intersection
    // drive until intersection or border
    int stop_reason = drive_along_street(ts, 0);
    if (stop_reason == 1) {
      turn_from_border = 1;
    } else if (stop_reason == 2 && !turn_from_border) {
      NXTCOLOR int_colors[4];
      scan_intersection(&int_colors[0], &int_colors[1], &int_colors[2], &int_colors[3], ts);
  
      if (i == 0) {
        localize_init(&lm, int_colors);
      } else {
        localize(&lm, int_colors, last_dir);
      }
      print_beliefs(&lm);

      int highest_x, highest_y;
      enum Direction highest_dir;
      get_highest_belief_coord(&lm, &highest_x, &highest_y, &highest_dir);
      int highest_idx = coord_to_idx(&lm, highest_x, highest_y);
      printf("??%d\n", highest_idx);
      double highest_belief = *(belief(&(lm.beliefs[highest_idx]), highest_dir));
      printf("!!\n");
      printf("Highest belief: %f, Location: %d %d %d\n", highest_belief, highest_x, highest_y, highest_dir);
      if (highest_belief >= 0.5) {
        printf("Returning");
        return (0);
      }
      
      // Move off of yellow and continue (forward)
      BT_drive(MOTOR_A, MOTOR_B, -70);
      usleep(1100000);// go for 1 second
      BT_motor_port_stop(MOTOR_A | MOTOR_B, 1);
      last_dir = FORWARD;
    } else if (stop_reason == 2 && turn_from_border) {
      // turn right and continue driving
      printf("Turning right due to encounter with border\n");

      BT_drive(MOTOR_A, MOTOR_B, -70);
      usleep(1050000); // Center axis of rotation on intersection
      BT_motor_port_stop(MOTOR_A | MOTOR_B, 1);
      turn_at_intersection(0, 90);
      turn_from_border = 0;
      last_dir = LEFT; // We are travelling the opposite direction because of border encounter, so left means right relative to how we were positioned before.
    } else {
      printf("Could not localize\n");
      return(-1);
    }


    

    printf("Continuing along street\n");
    iteration++;
    if (iteration >= it_amt) {
      break;
    }
    usleep(500000);
  }
  
  printf("Perimeter scan complete\n");
  return(0);
 *(robot_x)=-1;
 *(robot_y)=-1;
 *(direction)=-1;
 return(0);
}

int go_to_target(int target_x, int target_y, const struct ColourTrainingSet *ts)
{
 /*
  * This function is called once localization has been successful, it performs the actions required to take the robot
  * from its current location to the specified target location. 
  *
  * You have to write the code required to carry out this task - once again, you can use the function headers provided, or
  * write your own code to control the bot, but document your process carefully in the comments below so your TA can easily
  * understand how everything works.
  *
  * Your code should be able to determine if the robot has gotten lost (or if localization was incorrect), and your bot
  * should be able to recover.
  * 
  * Inputs - The robot's current location x,y (the intersection coordinates, not image pixel coordinates)
  *          The target's intersection location
  * 
  * Return values: 1 if successful (the bot reached its target destination), 0 otherwise
  */   
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/
  
  printf("Called with: %d %d\n", target_x, target_y);
   while (1) {
    
    printf("GOING TO TARGET! WE NEED TO GO: \n");
    enum RelativeDirection next_dir;
    // localize
    int current_x, current_y;
    enum Direction current_direction;

    printf("1\n");
    get_highest_belief_coord(&lm, &current_x, &current_y, &current_direction);

    printf("2\n");
    
    if (current_x == target_x && current_y == target_y) {
      break; // WE ARE HERE!!!
    }
    // set current x and y to the localized x and y
    if (current_x < target_x) {
      printf("EAST\n");
      next_dir = dir_diff(current_direction, EAST);
    } else if (current_x > target_x) {
      printf("WEST\n");
      next_dir = dir_diff(current_direction, WEST);
    } else if (current_y < target_y) {
      printf("SOUTH\n");
      next_dir = dir_diff(current_direction, SOUTH);
    } else if (current_y > target_y) {
      printf("NORTH\n");
      next_dir = dir_diff(current_direction, NORTH);
    } else {
      break;
    }

    
    int angle_delta = (int) next_dir * 90;
    turn_at_intersection(0, angle_delta);

    BT_drive(MOTOR_A, MOTOR_B, -70);
    usleep(1050000);
    BT_motor_port_stop(MOTOR_A | MOTOR_B, 1);

    drive_along_street(ts, 0);

    NXTCOLOR int_colors[4];
    scan_intersection(&int_colors[0], &int_colors[1], &int_colors[2], &int_colors[3], ts);

    localize(&lm, int_colors, next_dir);
    print_beliefs(&lm);
  }

  // do some action here play music???
  printf("Success SEGFAULT SEGFAULTSEGFAULTSEGFAULTSEGFAULT\n");
  return(0);  
}

// Gets the number of clockwise rotations needed to get from the current direction to the target direction
int get_direction_rotations_needed(enum RelativeDirection rel_dir)
{
  return (int) rel_dir;
  // int rotations = 0;
  // while (current_direction != target_direction) {
  //   current_direction = (current_direction + 1) % 4;
  //   rotations++;
  // }
  // return rotations;
}

void calibrate_sensor(void)
{
 /*
  * This function is called when the program is started with -1  -1 for the target location. 
  *
  * You DO NOT NEED TO IMPLEMENT ANYTHING HERE - but it is strongly recommended as good calibration will make sensor
  * readings more reliable and will make your code more resistent to changes in illumination, map quality, or battery
  * level.
  * 
  * The principle is - Your code should allow you to sample the different colours in the map, and store representative
  * values that will help you figure out what colours the sensor is reading given the current conditions.
  * 
  * Inputs - None
  * Return values - None - your code has to save the calibration information to a file, for later use (see in main())
  * 
  * How to do this part is up to you, but feel free to talk with your TA and instructor about it!
  */   

  /************************************************************************************************************************
   *   OIPTIONAL TO DO  -   Complete this function
   ***********************************************************************************************************************/
  fprintf(stderr,"Calibration function called!\n");  

  // Open a socket to the EV3 for remote controlling the bot.
  if (BT_open(HEXKEY)!=0)
  {
    fprintf(stderr,"Unable to open comm socket to the EV3, make sure the EV3 kit is powered on, and that the\n");
    fprintf(stderr," hex key for the EV3 matches the one in EV3_Localization.h\n");
    exit(1);
  }
  struct ColourTrainingSet ts;
  ts.count = 0;

  while (1) {
    NXTCOLOR target_colour;
    printf("Input the current colour under the sensor: ");
    scanf("%d", &target_colour);
    if (target_colour < 1 || target_colour > 6) break;
    struct ColourRGB rgb = get_sensor_rgb(PORT_1);
    struct ColourHSV hsv = rgb_to_hsv(&rgb);
    add_datapoint(&ts, &hsv, target_colour);
  }

  write_training_set(&ts);
}

int parse_map(unsigned char *map_img, int rx, int ry)
{
 /*
   This function takes an input image map array, and two integers that specify the image size.
   It attempts to parse this image into a representation of the map in the image. The size
   and resolution of the map image should not affect the parsing (i.e. you can make your own
   maps without worrying about the exact position of intersections, roads, buildings, etc.).

   However, this function requires:
   
   * White background for the image  [255 255 255]
   * Red borders around the map  [255 0 0]
   * Black roads  [0 0 0]
   * Yellow intersections  [255 255 0]
   * Buildings that are pure green [0 255 0], pure blue [0 0 255], or white [255 255 255]
   (any other colour values are ignored - so you can add markings if you like, those 
    will not affect parsing)

   The image must be a properly formated .ppm image, see readPPMimage below for details of
   the format. The GIMP image editor saves properly formatted .ppm images, as does the
   imagemagick image processing suite.
   
   The map representation is read into the map array, with each row in the array corrsponding
   to one intersection, in raster order, that is, for a map with k intersections along its width:
   
    (row index for the intersection)
    
    0     1     2    3 ......   k-1
    
    k    k+1   k+2  ........    
    
    Each row will then contain the colour values for buildings around the intersection 
    clockwise from top-left, that is
    
    
    top-left               top-right
            
            intersection
    
    bottom-left           bottom-right
    
    So, for the first intersection (at row 0 in the map array)
    map[0][0] <---- colour for the top-left building
    map[0][1] <---- colour for the top-right building
    map[0][2] <---- colour for the bottom-right building
    map[0][3] <---- colour for the bottom-left building
    
    Color values for map locations are defined as follows (this agrees with what the
    EV3 sensor returns in indexed-colour-reading mode):
    
    1 -  Black
    2 -  Blue
    3 -  Green
    4 -  Yellow
    5 -  Red
    6 -  White
    
    If you find a 0, that means you're trying to access an intersection that is not on the
    map! Also note that in practice, because of how the map is defined, you should find
    only Green, Blue, or White around a given intersection.
    
    The map size (the number of intersections along the horizontal and vertical directions) is
    updated and left in the global variables sx and sy.

    Feel free to create your own maps for testing (you'll have to print them to a reasonable
    size to use with your bot).
    
 */    
 
 int last3[3];
 int x,y;
 unsigned char R,G,B;
 int ix,iy;
 int bx,by,dx,dy,wx,wy;         // Intersection geometry parameters
 int tgl;
 int idx;
 
 ix=iy=0;       // Index to identify the current intersection
 
 // Determine the spacing and size of intersections in the map
 tgl=0;
 for (int i=0; i<rx; i++)
 {
  for (int j=0; j<ry; j++)
  {
   R=*(map_img+((i+(j*rx))*3));
   G=*(map_img+((i+(j*rx))*3)+1);
   B=*(map_img+((i+(j*rx))*3)+2);
   if (R==255&&G==255&&B==0)
   {
    // First intersection, top-left pixel. Scan right to find width and spacing
    bx=i;           // Anchor for intersection locations
    by=j;
    for (int k=i; k<rx; k++)        // Find width and horizontal distance to next intersection
    {
     R=*(map_img+((k+(by*rx))*3));
     G=*(map_img+((k+(by*rx))*3)+1);
     B=*(map_img+((k+(by*rx))*3)+2);
     if (tgl==0&&(R!=255||G!=255||B!=0))
     {
      tgl=1;
      wx=k-i;
     }
     if (tgl==1&&R==255&&G==255&&B==0)
     {
      tgl=2;
      dx=k-i;
     }
    }
    for (int k=j; k<ry; k++)        // Find height and vertical distance to next intersection
    {
     R=*(map_img+((bx+(k*rx))*3));
     G=*(map_img+((bx+(k*rx))*3)+1);
     B=*(map_img+((bx+(k*rx))*3)+2);
     if (tgl==2&&(R!=255||G!=255||B!=0))
     {
      tgl=3;
      wy=k-j;
     }
     if (tgl==3&&R==255&&G==255&&B==0)
     {
      tgl=4;
      dy=k-j;
     }
    }
    
    if (tgl!=4)
    {
     fprintf(stderr,"Unable to determine intersection geometry!\n");
     return(0);
    }
    else break;
   }
  }
  if (tgl==4) break;
 }
  fprintf(stderr,"Intersection parameters: base_x=%d, base_y=%d, width=%d, height=%d, horiz_distance=%d, vertical_distance=%d\n",bx,by,wx,wy,dx,dy);

  sx=0;
  for (int i=bx+(wx/2);i<rx;i+=dx)
  {
   R=*(map_img+((i+(by*rx))*3));
   G=*(map_img+((i+(by*rx))*3)+1);
   B=*(map_img+((i+(by*rx))*3)+2);
   if (R==255&&G==255&&B==0) sx++;
  }

  sy=0;
  for (int j=by+(wy/2);j<ry;j+=dy)
  {
   R=*(map_img+((bx+(j*rx))*3));
   G=*(map_img+((bx+(j*rx))*3)+1);
   B=*(map_img+((bx+(j*rx))*3)+2);
   if (R==255&&G==255&&B==0) sy++;
  }
  
  fprintf(stderr,"Map size: Number of horizontal intersections=%d, number of vertical intersections=%d\n",sx,sy);

  // Scan for building colours around each intersection
  idx=0;
  for (int j=0; j<sy; j++)
   for (int i=0; i<sx; i++)
   {
    x=bx+(i*dx)+(wx/2);
    y=by+(j*dy)+(wy/2);
    
    fprintf(stderr,"Intersection location: %d, %d\n",x,y);
    // Top-left
    x-=wx;
    y-=wy;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][0]=3;
    else if (R==0&&G==0&&B==255) map[idx][0]=2;
    else if (R==255&&G==255&&B==255) map[idx][0]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Top-Left RGB=%d,%d,%d\n",i,j,R,G,B);

    // Top-right
    x+=2*wx;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][1]=3;
    else if (R==0&&G==0&&B==255) map[idx][1]=2;
    else if (R==255&&G==255&&B==255) map[idx][1]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Top-Right RGB=%d,%d,%d\n",i,j,R,G,B);

    // Bottom-right
    y+=2*wy;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][2]=3;
    else if (R==0&&G==0&&B==255) map[idx][2]=2;
    else if (R==255&&G==255&&B==255) map[idx][2]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Bottom-Right RGB=%d,%d,%d\n",i,j,R,G,B);
    
    // Bottom-left
    x-=2*wx;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][3]=3;
    else if (R==0&&G==0&&B==255) map[idx][3]=2;
    else if (R==255&&G==255&&B==255) map[idx][3]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Bottom-Left RGB=%d,%d,%d\n",i,j,R,G,B);
    
    fprintf(stderr,"Colours for this intersection: %d, %d, %d, %d\n",map[idx][0],map[idx][1],map[idx][2],map[idx][3]);
    
    idx++;
   }

 return(1);  
}

unsigned char *readPPMimage(const char *filename, int *rx, int *ry)
{
 // Reads an image from a .ppm file. A .ppm file is a very simple image representation
 // format with a text header followed by the binary RGB data at 24bits per pixel.
 // The header has the following form:
 //
 // P6
 // # One or more comment lines preceded by '#'
 // 340 200
 // 255
 //
 // The first line 'P6' is the .ppm format identifier, this is followed by one or more
 // lines with comments, typically used to inidicate which program generated the
 // .ppm file.
 // After the comments, a line with two integer values specifies the image resolution
 // as number of pixels in x and number of pixels in y.
 // The final line of the header stores the maximum value for pixels in the image,
 // usually 255.
 // After this last header line, binary data stores the RGB values for each pixel
 // in row-major order. Each pixel requires 3 bytes ordered R, G, and B.
 //
 // NOTE: Windows file handling is rather crotchetty. You may have to change the
 //       way this file is accessed if the images are being corrupted on read
 //       on Windows.
 //

 FILE *f;
 unsigned char *im;
 char line[1024];
 int i;
 unsigned char *tmp;
 double *fRGB;

 im=NULL;
 f=fopen(filename,"rb+");
 if (f==NULL)
 {
  fprintf(stderr,"Unable to open file %s for reading, please check name and path\n",filename);
  return(NULL);
 }
 fgets(&line[0],1000,f);
 if (strcmp(&line[0],"P6\n")!=0)
 {
  fprintf(stderr,"Wrong file format, not a .ppm file or header end-of-line characters missing\n");
  fclose(f);
  return(NULL);
 }
 fprintf(stderr,"%s\n",line);
 // Skip over comments
 fgets(&line[0],511,f);
 while (line[0]=='#')
 {
  fprintf(stderr,"%s",line);
  fgets(&line[0],511,f);
 }
 sscanf(&line[0],"%d %d\n",rx,ry);                  // Read image size
 fprintf(stderr,"nx=%d, ny=%d\n\n",*rx,*ry);

 fgets(&line[0],9,f);  	                // Read the remaining header line
 fprintf(stderr,"%s\n",line);
 im=(unsigned char *)calloc((*rx)*(*ry)*3,sizeof(unsigned char));
 if (im==NULL)
 {
  fprintf(stderr,"Out of memory allocating space for image\n");
  fclose(f);
  return(NULL);
 }
 fread(im,(*rx)*(*ry)*3*sizeof(unsigned char),1,f);
 fclose(f);

 return(im);    
}