/*
	Lander Control simulation.

	Updated by F. Estrada for CSC C85, Oct. 2013
	Updated by Per Parker, Sep. 2015

	Learning goals:

	- To explore the implementation of control software
	  that is robust to malfunctions/failures.

	The exercise:

	- The program loads a terrain map from a .ppm file.
	  the map shows a red platform which is the location
	  a landing module should arrive at.
	- The control software has to navigate the lander
	  to this location and deposit the lander on the
	  ground considering:

	  * Maximum vertical speed should be less than 10 m/s at touchdown
	  * Maximum landing angle should be less than 15 degrees w.r.t vertical

	- Of course, touching any part of the terrain except
	  for the landing platform will result in destruction
	  of the lander

	This has been made into many videogames. The oldest one
	I know of being a C64 game called 1985 The Day After.
        There are older ones! (for bonus credit, find the oldest
        one and send me a description/picture plus info about the 
        platform it ran on!)

	Your task:

	- These are the 'sensors' you have available to control
          the lander.

	  Velocity_X();  - Gives you the lander's horizontal velocity
	  Velocity_Y();	 - Gives you the lander's vertical velocity
	  Position_X();  - Gives you the lander's horizontal position (0 to 1024)
	  Position Y();  - Gives you the lander's vertical position (0 to 1024)

          Angle();	 - Gives the lander's angle w.r.t. vertical in DEGREES (upside-down = 180 degrees)

	  SONAR_DIST[];  - Array with distances obtained by sonar. Index corresponds
                           to angle w.r.t. vertical direction measured clockwise, so that
                           SONAR_DIST[0] is distance at 0 degrees (pointing upward)
                           SONAR_DIST[1] is distance at 10 degrees from vertical
                           SONAR_DIST[2] is distance at 20 degrees from vertical
                           .
                           .
                           .
                           SONAR_DIST[35] is distance at 350 degrees from vertical

                           if distance is '-1' there is no valid reading. Note that updating
                           the sonar readings takes time! Readings remain constant between
                           sonar updates.

          RangeDist();   - Uses a laser range-finder to accurately measure the distance to ground
                           in the direction of the lander's main thruster.
                           The laser range finder never fails (probably was designed and
                           built by PacoNetics Inc.)

          Note: All sensors are NOISY. This makes your life more interesting.

	- Variables accessible to your 'in flight' computer

	  MT_OK		- Boolean, if 1 indicates the main thruster is working properly
	  RT_OK		- Boolean, if 1 indicates the right thruster is working properly
	  LT_OK		- Boolean, if 1 indicates thr left thruster is working properly
          PLAT_X	- X position of the landing platform
          PLAY_Y        - Y position of the landing platform

	- Control of the lander is via the following functions
          (which are noisy!)

	  Main_Thruster(double power);   - Sets main thurster power in [0 1], 0 is off
	  Left_Thruster(double power);	 - Sets left thruster power in [0 1]
	  Right_Thruster(double power);  - Sets right thruster power in [0 1]
	  Rotate(double angle);	 	 - Rotates module 'angle' degrees clockwise
					   (ccw if angle is negative) from current
                                           orientation (i.e. rotation is not w.r.t.
                                           a fixed reference direction).

 					   Note that rotation takes time!


	- Important constants

	  G_ACCEL = 8.87	- Gravitational acceleration on Venus
	  MT_ACCEL = 35.0	- Max acceleration provided by the main thruster
	  RT_ACCEL = 25.0	- Max acceleration provided by right thruster
	  LT_ACCEL = 25.0	- Max acceleration provided by left thruster
          MAX_ROT_RATE = .075    - Maximum rate of rotation (in radians) per unit time

	- Functions you need to analyze and possibly change

	  * The Lander_Control(); function, which determines where the lander should
	    go next and calls control functions
          * The Safety_Override(); function, which determines whether the lander is
            in danger of crashing, and calls control functions to prevent this.

	- You *can* add your own helper functions (e.g. write a robust thruster
	  handler, or your own robust sensor functions - of course, these must
	  use the noisy and possibly faulty ones!).

	- The rest is a black box... life sometimes is like that.

        - Program usage: The program is designed to simulate different failure
                         scenarios. Mode '1' allows for failures in the
                         controls. Mode '2' allows for failures of both
                         controls and sensors. There is also a 'custom' mode
                         that allows you to test your code against specific
                         component failures.

			 Initial lander position, orientation, and velocity are
                         randomized.

	  * The code I am providing will land the module assuming nothing goes wrong
          with the sensors and/or controls, both for the 'easy.ppm' and 'hard.ppm'
          maps.

	  * Failure modes: 0 - Nothing ever fails, life is simple
			   1 - Controls can fail, sensors are always reliable
			   2 - Both controls and sensors can fail (and do!)
			   3 - Selectable failure mode, remaining arguments determine
                               failing component(s):
                               1 - Main thruster
                               2 - Left Thruster
                               3 - Right Thruster
                               4 - Horizontal velocity sensor
                               5 - Vertical velocity sensor
                               6 - Horizontal position sensor
                               7 - Vertical position sensor
                               8 - Angle sensor
                               9 - Sonar

        e.g.

             Lander_Control easy.ppm 3 1 5 8

             Launches the program on the 'easy.ppm' map, and disables the main thruster,
             vertical velocity sensor, and angle sensor.

		* Note - while running. Pressing 'q' on the keyboard terminates the 
			program.

        * Be sure to complete the attached REPORT.TXT and submit the report as well as
          your code by email. Subject should be 'C85 Safe Landings, name_of_your_team'

	Have fun! try not to crash too many landers, they are expensive!

  	Credits: Lander image and rocky texture provided by NASA
		 Per Parker spent some time making sure you will have fun! thanks Per!
*/

/*
  Standard C libraries
*/
#include <math.h>

#include "Lander_Control.h"
#include "stdio.h"

struct LanderState {
 // Physical properties of the craft
 double altitude; // Vertical distance from the ground
 double pos_x;
 double vel_x, vel_y;
 double angle; // Current spacecraft angle in degrees

 // Calculated properies of the craft
 double max_acc; // Maximum acceleration based on available thrusters
 double landing_acc; // Vertical acceleration required to stop the craft at the ground based on the current falling rate
};

enum LandingPhase {
  FALLING,
  LANDING_BURN,
  LANDED
};

// Constants
double k = 5.5; // Constant adjustment factor to make physics work properly (determined by trial and error)
double landing_cutoff_v = -5; // Minimum vertical velocity (signed) needed to end the landing burn;
double landing_burn_k = 1.0; // Ideal acceleration to do the landing burn at as a multiple of max_acc
double landing_height_offset = 25.0; // Height at which the lander will come to a stop above the landing pad

// Gobally declared state variables
struct LanderState lander_state;
enum LandingPhase landing_phase = FALLING;

struct LanderState calculateLanderState() {
 struct LanderState ls;

 ls.altitude = 1000 - Position_Y() - (1000 - PLAT_Y);
 ls.pos_x = Position_X();
 ls.vel_y = Velocity_Y();
 ls.angle = fmod(Angle(), 360.0);

 ls.landing_acc = k * ls.vel_y * ls.vel_y / (2 * (ls.altitude - landing_height_offset)) + 8.87;
 ls.max_acc = (MT_OK) ? 35.0 : 25.0;
 return ls;
}

void orientLander(double angle_rad, const struct LanderState *ls) {
  double epsilon = 2.0;
  double target_angle = fmod((angle_rad / (2.0 * PI)) * 360.0, 360.0); // Convert to degrees
  double angle_delta = fmod((ls->angle - target_angle + 540.0), 360.0) - 180.0;

  if (fabs(angle_delta) > epsilon) {
    Rotate(-1 * angle_delta);
  }
}

void thrustVector(double angle_rad, double acceleration, const struct LanderState *ls) {
  angle_rad += (MT_OK) ? 0 : (RT_OK) ? PI / 2.0 : PI / -2.0;
  double epsilon = 2.0;
  double target_angle = fmod((angle_rad / (2.0 * PI)) * 360.0, 360.0); // Convert to degrees
  double angle_delta = fmod((ls->angle - target_angle + 540.0), 360.0) - 180.0;

  if (fabs(angle_delta) > epsilon) {
    Rotate(-1 * angle_delta);
  }
  
  printf("Burning at acceleration %f\n", acceleration);
  if (MT_OK) {
    Main_Thruster(acceleration / 35.0);
  } else if (RT_OK) {
    Right_Thruster(acceleration / 25.0);
  } else {
    Left_Thruster(acceleration / 25.0); 
  }
}

void displayState(const struct LanderState *lander_state, enum LandingPhase phase) {
 printf("===== Lander State ===============================\n");
 printf("PHASE:                        %d\n", phase);
 printf("Velocity Y:                   %f\n", lander_state->vel_y);
 printf("Altitude Gnd:                 %f\n", lander_state->altitude);
 printf("Landing Acc:                  %f\n", lander_state->landing_acc);
}

enum LandingPhase determineLandingPhase(enum LandingPhase current_phase, const struct LanderState *lander_state) {
 double landing_burn_target_acc = lander_state->max_acc * landing_burn_k;

 if (current_phase == LANDED || current_phase == LANDING_BURN && lander_state->vel_y >= landing_cutoff_v) {
  return LANDED;
 } if (current_phase == LANDING_BURN && (lander_state->landing_acc < landing_burn_target_acc / 2.0 || lander_state->vel_y >= landing_cutoff_v)) {
  return FALLING;
 } else if (lander_state->landing_acc >= landing_burn_target_acc) {
  return LANDING_BURN;
 } else {
  return current_phase;
 }
}

void Lander_Control(void)
{
 /*
   This is the main control function for the lander. It attempts
   to bring the ship to the location of the landing platform
   keeping landing parameters within the acceptable limits.

   How it works:

   - First, if the lander is rotated away from zero-degree angle,
     rotate lander back onto zero degrees.
   - Determine the horizontal distance between the lander and
     the platform, fire horizontal thrusters appropriately
     to change the horizontal velocity so as to decrease this
     distance
   - Determine the vertical distance to landing platform, and
     allow the lander to descend while keeping the vertical
     speed within acceptable bounds. Make sure that the lander
     will not hit the ground before it is over the platform!

   As noted above, this function assumes everything is working
   fine.
*/

/*************************************************
 TO DO: Modify this function so that the ship safely
        reaches the platform even if components and
        sensors fail!

        Note that sensors are noisy, even when
        working properly.

        Finally, YOU SHOULD provide your own
        functions to provide sensor readings,
        these functions should work even when the
        sensors are faulty.

        For example: Write a function Velocity_X_robust()
        which returns the module's horizontal velocity.
        It should determine whether the velocity
        sensor readings are accurate, and if not,
        use some alternate method to determine the
        horizontal velocity of the lander.

        NOTE: Your robust sensor functions can only
        use the available sensor functions and control
        functions!
	DO NOT WRITE SENSOR FUNCTIONS THAT DIRECTLY
        ACCESS THE SIMULATION STATE. That's cheating,
        I'll give you zero.
**************************************************/
 
 lander_state = calculateLanderState();
 landing_phase = determineLandingPhase(landing_phase, &lander_state);
 displayState(&lander_state, landing_phase);

 if (landing_phase == LANDED) {
  // Turn off engine and do nothing else
  Main_Thruster(0);
  Left_Thruster(0);
  Right_Thruster(0);
  return;
 }

 if (landing_phase == FALLING) {
  orientLander(0, &lander_state);
 } else if (landing_phase == LANDING_BURN) {
  thrustVector(0, lander_state.landing_acc, &lander_state);
 }
}

void Safety_Override(void)
{
  return;
}
