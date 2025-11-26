/**************************************************************************
  CSC C85 - UTSC RoboSoccer AI core

  This file is where the actual planning is done and commands are sent
  to the robot.

  Please read all comments in this file, and add code where needed to
  implement your game playing logic. 

  Things to consider:

  - Plan - don't just react
  - Use the heading vectors!
  - Mind the noise (it's everywhere)
  - Try to predict what your oponent will do
  - Use feedback from the camera

  What your code should not do: 

  - Attack the opponent, or otherwise behave aggressively toward the
    oponent
  - Hog the ball (you can kick it, push it, or leave it alone)
  - Sit at the goal-line or inside the goal
  - Run completely out of bounds

  AI scaffold: Parker-Lee-Estrada, Summer 2013

  EV3 Version 2.0 - Updated Jul. 2022 - F. Estrada
***************************************************************************/

#include "roboAI.h"			// <--- Look at this header file!
#include "roboControl.h"
#include "roboUtils.h"
int laggy=0;
double fix_dx, fix_dy;
double o_dx, o_dy;

// Parameters
double theta_th = 0.3;      // cos(angle threshold)
double dis_th   = 50;       // distance threshold
int drive_pw    = 30;
int turn_pw     = 30;

// Distance from ball to position bot for kick (in pixels) we can update this later
#define KICK_POSITION_DISTANCE 200.0

#include "pacofuncs.c"

/**************************************************************************
 * AI state machine - this is where you will implement your soccer
 * playing logic
 * ************************************************************************/
void AI_main(struct RoboAI *ai, struct blob *blobs, void *state)
{
 /*************************************************************************
  This is your robot's state machine.
  
  It is called by the imageCapture code *once* per frame. And it *must not*
  enter a loop or wait for visual events, since no visual refresh will happen
  until this call returns!
  
  Therefore. Everything you do in here must be based on the states in your
  AI and the actions the robot will perform must be started or stopped 
  depending on *state transitions*. 

  E.g. If your robot is currently standing still, with state = 03, and
   your AI determines it should start moving forward and transition to
   state 4. Then what you must do is 
   - send a command to start forward motion at the desired speed
   - update the robot's state
   - return
  
  I can not emphasize this enough. Unless this call returns, no image
  processing will occur, no new information will be processed, and your
  bot will be stuck on its last action/state.

  You will be working with a state-based AI. You are free to determine
  how many states there will be, what each state will represent, and
  what actions the robot will perform based on the state as well as the
  state transitions.

  You must *FULLY* document your state representation in the report

  The first two states for each more are already defined:
  State 0,100,200 - Before robot ID has taken place (this state is the initial
            	    state, or is the result of pressing 'r' to reset the AI)
  State 1,101,201 - State after robot ID has taken place. At this point the AI
            	    knows where the robot is, as well as where the opponent and
            	    ball are (if visible on the playfield)

  Relevant UI keyboard commands:
  'r' - reset the AI. Will set AI state to zero and re-initialize the AI
	data structure.
  't' - Toggle the AI routine (i.e. start/stop calls to AI_main() ).
  'o' - Robot immediate all-stop! - do not allow your EV3 to get damaged!

   IMPORTANT NOTE: There are TWO sources of information about the 
                   location/parameters of each agent
                   1) The 'blob' data structures from the imageCapture module
                   2) The values in the 'ai' data structure.
                      The 'blob' data is incomplete and changes frame to frame
                      The 'ai' data should be more robust and stable
                      BUT in order for the 'ai' data to be updated, you
                      must call the function 'track_agents()' in your code
                      after eah frame!
                      
    DATA STRUCTURE ORGANIZATION:

    'RoboAI' data structure 'ai'
         \    \    \   \--- calibrate()  (pointer to AI_clibrate() )
          \    \    \--- runAI()  (pointer to the function AI_main() )
           \    \------ Display List head pointer 
            \_________ 'ai_data' data structure 'st'
                         \  \   \------- AI state variable and other flags
                          \  \---------- pointers to 3 'blob' data structures
                           \             (one per agent)
                            \------------ parameters for the 3 agents
                              
  ** Do not change the behaviour of the robot ID routine **
 **************************************************************************/

  static double ux,uy,len,mmx,mmy,tx,ty,x1,y1,x2,y2;
  double angDif;
  char line[1024];
  static int count=0;
  static double old_dx=0, old_dy=0;
      
  /************************************************************
   * Standard initialization routine for starter code,
   * from state **0 performs agent detection and initializes
   * directions, motion vectors, and locations
   * Triggered by toggling the AI on.
   * - Modified now (not in starter code!) to have local
   *   but STATIC data structures to keep track of robot
   *   parameters across frames (blob parameters change
   *   frame to frame, memoryless).
   ************************************************************/
 if (ai->st.state==0||ai->st.state==100||ai->st.state==200)  	// Initial set up - find own, ball, and opponent blobs
 {
  // Carry out self id process.
  fprintf(stderr,"Initial state, self-id in progress...\n");
  
  id_bot(ai,blobs);
  if ((ai->st.state%100)!=0)	  // The id_bot() routine will change the AI state to initial state + 1
  {				                 // if robot identification is successful.
      
   if (ai->st.self->cx>=512) ai->st.side=1; else ai->st.side=0;         // This sets the side the bot thinks as its own side 0->left, 1->right
   stop_moving(ai);
   
   fprintf(stderr,"Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], blob direction=[%f, %f], AI state=%d\n",ai->st.self->cx,ai->st.self->cy,ai->st.smx,ai->st.smy,ai->st.sdx,ai->st.sdy,ai->st.state);
   
   if (ai->st.self!=NULL)
   {
       // This checks that the motion vector and the blob direction vector
       // are pointing in the same direction. If they are not (the dot product
       // is less than 0) it inverts the blob direction vector so it points
       // in the same direction as the motion vector.
       if (((ai->st.smx*ai->st.sdx)+(ai->st.smy*ai->st.sdy))<0)
       {
           ai->st.self->dx*=-1.0;
           ai->st.self->dy*=-1.0;
           ai->st.sdx*=-1;
           ai->st.sdy*=-1;
       }
       old_dx=ai->st.sdx;
       old_dy=ai->st.sdy;
   }
  
   if (ai->st.opp!=NULL)
   {
       // Checks motion vector and blob direction for opponent. See above.
       if (((ai->st.omx*ai->st.odx)+(ai->st.omy*ai->st.ody))<0)
       {
           ai->st.opp->dx*=-1;
           ai->st.opp->dy*=-1;
           ai->st.odx*=-1;
           ai->st.ody*=-1;
       }       
   }

         
  }
  
  // Initialize BotInfo structures
   
 }
 else
 {
  



  /****************************************************************************
   TO DO:
   You will need to replace this 'catch-all' code with actual program logic to
   implement your bot's state-based AI.

   After id_bot() has successfully completed its work, the state should be
   1 - if the bot is in SOCCER mode
   101 - if the bot is in PENALTY mode
   201 - if the bot is in CHASE mode

   Your AI code needs to handle these states and their associated state
   transitions which will determine the robot's behaviour for each mode.

   Please note that in this function you should add appropriate functions below
   to handle each state's processing, and the code here should mostly deal with
   state transitions and with calling the appropriate function based on what
   the bot is supposed to be doing.
  *****************************************************************************/
//  fprintf(stderr,"Just trackin'!\n");	// bot, opponent, and ball.
//  track_agents(ai,blobs);		// Currently, does nothing but endlessly track
// Update blob tracking for this frame
    track_agents(ai, blobs);
    convert_to_metric(ai);
    // clean_heading(ai, &old_dx, &old_dy);
    determine_facing(ai);
    update_vars(ai);
    
    // printf("Driving dir: %d\n", ai->st.driving_dir);
    // printf("Facing direction: %f %f\n", ai->st.fxm, ai->st.fym);
    // printf("Angle: %f\n", ai->st.fa);

    // States on this range are for soccer against the opponent
    if (ai->st.state < 100) {
      double our_net_x = ((ai->st.side == 0) ? 0 : 170.0);
      double our_net_y = 115.0 / 2.0;
      double defense_point_x = (ai->st.bpxm + our_net_x) / 2;
      double defense_point_y = (ai->st.bpym + our_net_y) / 2;
      double distance = norm(ai->st.spxm - defense_point_x, ai->st.spym - defense_point_y);

      double opp_net_x = (ai->st.side == 0) ? 170.0 : 0;
      double opp_net_y = 115.0 / 2.0;
      double ball_opp_net_vec_x = opp_net_x - ai->st.bpxm;
      double ball_opp_net_vec_y = opp_net_y - ai->st.bpym;
      normalize_vector(&ball_opp_net_vec_x, &ball_opp_net_vec_y);
      double kick_dist = 25.0;
      double kick_point_x = ai->st.bpxm - ball_opp_net_vec_x * kick_dist;
      double kick_point_y = ai->st.bpym - ball_opp_net_vec_y * kick_dist;
      double (kick_point_y > 105) ? 105 : kick_point_y;
      double (kick_point_y < 10) ? 10 : kick_point_y;
      double (kick_point_x > 105) ? 170 : kick_point_y;
      double (kick_point_x < 0) ? 10 : kick_point_y;
      double kick_point_dist = norm(ai->st.spxm - kick_point_x, ai->st.spym - kick_point_y);
      double ball_speed = norm(ai->st.bvxm, ai->st.bvym);
      double da = angle_diff(ai->st.fa, atan2(ai->st.bpym - ai->st.spym, ai->st.bpxm - ai->st.spxm));

      printf("Def_pos X: %f, Y: %f", defense_point_x, defense_point_y);

      switch(ai->st.state) {
        case 1: // Figure out forward direction
          if (ai->st.driving_dir == 1) ai->st.state = 11;
          break;
        case 11: // Go to defensive position
          if (distance < 8.0) ai->st.state = 21;
          break;
        case 21: // Go to kick point
          if (fabs(ai->st.bpxm - our_net_x) < fabs(ai->st.spxm - our_net_x) + 5) ai->st.state = 11; // If the ball is closer to our net than we are then go defend
          if (kick_point_dist < 8.0) ai->st.state = 22;
          break;
        case 22: // Align
          if (fabs(ai->st.bpxm - our_net_x) < fabs(ai->st.spxm - our_net_x) + 5) ai->st.state = 11; // If the ball is closer to our net than we are then go defend
          if (da < 0.05) ai->st.state = 23;
        case 23: // Kick the ball
          if (norm(ai->st.spxm - ai->st.bpxm, ai->st.spym - ai->st.bpym) > 30.0) ai->st.state = 11;
          if (fabs(ai->st.bpxm - our_net_x) < fabs(ai->st.spxm - our_net_x) + 5) ai->st.state = 11; // If the ball is closer to our net than we are then go defend
          if (ball_speed > 9.0) ai->st.state = 11;
          break;
      }
      printf("State: %d\n", ai->st.state);
      // 0 - 20 are for defence
      // 21 - 99 are for offence
      switch(ai->st.state) {
        // Go to defence kick position
        case 1:
          move_forward(35, ai);
          break;
        case 11: // Go to defensive position
          safe_go_to_point(ai, 50, 10, defense_point_x, defense_point_y);
          break;
        case 21: // Go to kick point
          safe_go_to_point(ai, 50, 10, kick_point_x, kick_point_y);
          break;
        case 22: // Align
          safe_go_to_point(ai, 20, 0, ai->st.bpxm, ai->st.bpym);
          break;
        case 23: // Kick the ball
          safe_go_to_point(ai, 100, 20, ai->st.bpxm, ai->st.bpym);
          break;
      }
    } else if (ai->st.state < 200) {
      double offense_target_x = (ai->st.side == 0) ? 170.0 : 0;
      double offense_target_y = 115.0 / 2.0;

      double targetPointVectorX = offense_target_x - ai->st.bpxm;
      double targetPointVectorY = offense_target_y - ai->st.bpym;
      normalize_vector(&targetPointVectorX, &targetPointVectorY);
      double kick_dist = 25.0;
      double kick_point_x = ai->st.bpxm - targetPointVectorX * kick_dist;
      double kick_point_y = ai->st.bpym - targetPointVectorY * kick_dist;
      double kick_point_dist = norm(ai->st.spxm - kick_point_x, ai->st.spym - kick_point_y);
      double ball_speed = norm(ai->st.bvxm, ai->st.bvym);

      double da = angle_diff(ai->st.fa, atan2(offense_target_y - ai->st.spym, offense_target_x - ai->st.spxm));
      printf("Ball sped: %f\n", ball_speed);
      

      printf("Ball point %f %f\n", offense_target_y, ai->st.bpym);
      printf("Dist %f\n", kick_point_dist);
      printf("DA: %f\n", da);


      switch(ai->st.state) {
        case 101:
          // Go to shooting position coarse
          if (kick_point_dist < 16.0) ai->st.state = 102;
          break;
        case 102:
          // Go to shooting position fine tune
          if (kick_point_dist < 5.0) ai->st.state = 103;
          break;
        case 103:
          // Go to shooting position fine tune
          if (da < 0.02) ai->st.state = 104;
          break;
        case 104:
          if (ball_speed > 7) ai->st.state = 105;
          break;
      }

      printf("Current state: %d\n", ai->st.state);

      switch(ai->st.state) {
        case 101:
          go_to_point(ai, 65, 10, kick_point_x, kick_point_y);
          break;
        case 102:
          go_to_point(ai, 30, 3, kick_point_x, kick_point_y);
          break;
        case 103:
          go_to_point(ai, 10, 0, offense_target_x, offense_target_y);
          break;
        case 104:
          go_to_point(ai, 100, 10, offense_target_x, offense_target_y);
          break;
        case 105: 
          stop_moving(ai);
          break;
        default:
          stop_moving(ai);
          break;
      }
    } else if (ai->st.state < 300) {
      printf("Chase ball\n");

      struct blob *my_bot = ai->st.self;
      struct blob *ball = ai->st.ball;

      if (!my_bot || !ball) {
        printf("All stop.\n");
        if (!my_bot) {
          printf("No bot.\n");
        }
        else {
          printf("No ball.\n");

        }
          stop_moving(ai);
          return;
      }

      // --- Ball/self vectors
      double ballx = ai->st.bpxm;   // ball position in meters
      double bally = ai->st.bpym;
      double myx   = ai->st.spxm;   // robot position in meters
      double myy   = ai->st.spym;

      double dist_to_ball = point_distance(myx, myy, ballx, bally);

      double fast_speed = 50;
      double slow_speed = 35;
      double slow_dist  = 20;   // meters (20 cm)

      switch(ai->st.state) {
        case 201:
            printf("Distance: %f \n", dist_to_ball);

            // Slow down when close
            int power, turn_smooth;

            if (dist_to_ball < slow_dist) {
                power = slow_speed;
                turn_smooth = 10;
            }
            else {
                power = fast_speed;
                turn_smooth = 10;
            }

            go_to_point(ai, power, 10, ballx, bally);

            printf("ball x %f ball y %f  dist: %f  power: %d\n",
                  ballx, bally, dist_to_ball, power);

            break;

        default:
            fprintf(stderr, "[CHASE] Unknown state %d, default to 201.\n", ai->st.state);
            ai->st.state = 201;
            break;
      }

    }

  printf("CURRENT STATE: %d\n", ai->st.state);

 }
}
/**********************************************************************************
 TO DO:

 Add the rest of your game playing logic below. Create appropriate functions to
 handle different states (be sure to name the states/functions in a meaningful
 way), and do any processing required in the space below.

 AI_main() should *NOT* do any heavy lifting. It should only call appropriate
 functions based on the current AI state.

 You will lose marks if AI_main() is cluttered with code that doesn't belong
 there.
**********************************************************************************/

void clean_heading(struct RoboAI *ai, double *old_dx, double *old_dy)
{
    // New raw blob heading
    double ndx = ai->st.self->dx;
    double ndy = ai->st.self->dy;

    double mag = sqrt(ndx*ndx + ndy*ndy);
    if (mag < 1e-6) return;   // ignore zero/noisy heading

    // Normalize
    ndx /= mag;
    ndy /= mag;

    // Compare new heading to old stabilized heading using dot product
    double dotp = ndx * (*old_dx) + ndy * (*old_dy);

    // If dot < 0 the heading flipped 180º → reverse it
    if (dotp < 0) {
        ndx = -ndx;
        ndy = -ndy;
    }

    // Save clean/stable heading
    ai->st.sdx = ndx;
    ai->st.sdy = ndy;

    *old_dx = ndx;
    *old_dy = ndy;
}

double f_angle(double x1, double y1, double x2, double y2)
{
  double angle = atan2(x1*y2 - x2*y1, x1*x2 + y1*y2);
  printf("Angle: %f\n", angle);
  return angle;
}

// PENALTY KICK STUFF

// Gist of state transactions (rename state numbers into the actual groups later)
// State 1: Initial state, bot not moving
//          - Calculate shooting vector of the ball to the goal
//          - Calculate the pathing needed to reach the ball and also take into account the shooting vector so can be factored in pathing (maybe something like an arc)
//          - Move into next state which is pathing to the point (the ball itself)
// State 2: Pathing to the ball
//          - Check if the bot is within the distance to the ball (if so then can enter into the alignment state)
//          - Recalculate the pathing to the ball needed
//          - Turn on the motors with some motor power needed on both wheels (same if forward or some ratio if driving in an arc / turning)
//             - The gyro can be used here as I think the camera refresh rate is not fast enough if turning at full speed
//          - Store the target angle wanted in a state variable along with the heading before the turn was initiated (if not straight pathing)
// State 3: Alignment check
//          - Check what the bots current vector is compared to the shooting vector and our difference around the ball is acceptable
//          - If the difference is larger than some epsilon then we need to drive around the ball and orient the bot to the shooting vecotr
//          - NOTE: alignment will look like some point on a circle of x diameter from the ball so we have time to accelerate and bump the ball
//          - If the difference is small enough then we can enter into the kick state (state 4)
// State 3.1: Start Distance Alignment
//          - Get our distance from the ball and either start moving forward or backward to get to the target diameter
//          - Transition to State 3.2: Distance Alignment
// State 3.2: Diameter Alignment
//          - Check if the bot is within the target diameter (if so then transition to state 3.3)
//          - If not then continue to move forward or backward to get to the target diameter
// State 3.3: Circumference Positioning
//          - Check if the bot is aligned to start driving along the circumference of the circle we have around the ball (the bot will be tangent to the circle if aligned correctly)
//          - If not then start a turn then transition to state 3.4: Vector Positioning
// State 3.4: Circumference Alignment Positioning
//          - Check if the bot is aligned along the circumference of the circle we have around the ball
//          - If so then transitions to state 3.5: Circumference Positioning and start driving in an arc around the ball
// State 3.5: Circumference Positioning
//          - Check if the bot is close enough to a point along the shooting vector
//          - If so then transition to state 3.6: Final Shooting Check
// State 3.6: Final Shooting Check
//          - Check if the bot has:
//            - Good diameter around the ball
//            - Good alignment with the shooting vector
//          - If so then transitions to state 4 and full send motors with max power
//          - else restart the alignment process by transitioning to state 3.1
// State 4: Kick state
//          - Check if the ball has moved since its last known location by some epsilon (2 norm?)
//          - If not then continue to drive straight
//          - If so then transition to state 5: Goal state
// State 5: Goal state
//          - Celebrate or something cause if you missing the goal then rip lmao
