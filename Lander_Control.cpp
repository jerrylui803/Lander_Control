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

	  * Maximum vertical speed should be less than 5 m/s at touchdown
	  * Maximum landing angle should be less than 10 degrees w.r.t vertical

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
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include "Lander_Control.h"

/* user-defined macros to keep track of direction of movement. */
#define LEFT -1
#define RIGHT 1

/* function signatures placed here as needed. */
double robust_angle(void);

/* keeps track of number of total frames accumulated during program run. */
static int frames = 0;

/* keeps track of the horizontal direction that the lander is moving towards. */
static int direction;

/* Sensor values. */
static double VX;
static double VY;
static double PX;
static double PY;
static double init_X; // not used

/* Sensor failure flags. */
static int VX_OK = 1; // Velocity X
static int VY_OK = 1; // Velocity Y
static int PX_OK = 1; // Position X
static int PY_OK = 1; // Position Y
static int SN_OK = 1; // Sonar

/* Replaces Velocity_X sensor when it fails. */
double robust_VX(void) {
    /* TO DO */
    VX = Velocity_X();
    return VX;

    /* DOES NOT WORK */
    /*
    double noisy_VX   = Velocity_X();
    double error      = fabs(noisy_VX-VX);
    double adjustment = (((double) rand() / RAND_MAX) * 0.1);
    //if (error<0.1)
    //    VX = VX + (direction*error);
    //else if (direction == LEFT)   VX = VX - 0.01;
    //else if (direction == RIGHT)  VX = VX + 0.01;
    //if (error<0.2)  VX = noisy_VX;
    //VX = VX + (direction*((Position_X()-PX)));
    VX = VX + direction*(fabs((Position_X()-PX)));
    */

}

/* Replaces Velocity_Y sensor when it fails. */
double robust_VY(void) {
    /* TO DO */
    VY = Velocity_Y();
    return VY;
}

/* Replaces Position_X sensor when it fails. */
double robust_PX(void) {
    /* TO DO */
    PX = Position_X();
    return PX;
    
    /* DOES NOT WORK */
    /*
    int        done           = 0;
    static int step           = 0;

    static double init_Y      = 0;
    static double saved_angle = 0;
    static double X           = 0;
    static double next_angle  = 0;
    static double dist_RX     = 0;
    static double dist_LX     = 0;
  
    if (step == 0) {
        saved_angle = robust_angle();
        init_X      = Position_X();
        init_Y      = Position_Y();
        step += 1;
    }

    if (step == 1) {
        if (init_X < PLAT_X) { // we are starting from the left
            next_angle = 90;
            if (Position_X() < init_X)
               // Can't use rangefinder yet
               return(Position_X());
        }
        else { // we are starting from the right
            next_angle = 270;
            if (Position_X() > init_X)
               // Can't use rangefinder yet
               return(Position_X());
        }

        if (orientate(next_angle)) {
             step += 1;
        }
    }

    if (step == 2) {
        if (init_X < PLAT_X) { // measure distances from the left of the map  
            dist_LX = fmax(dist_LX, RangeDist() * sin(next_angle * M_PI/180));
            X = fmax(X, dist_LX);
            next_angle -= 10;   
            if (next_angle > 90) {
                step = 3; // all measurements taken
            }
            else {
                step = 1; // Orientate to next angle
            }
        }
        else { // measure distances from the right of the map
            dist_RX = fmax(dist_RX, RangeDist() * sin(next_angle * M_PI/180));
            X = fmax(X, dist_LX);
            next_angle += 10;  
            if (next_angle < 270) {
                step = 3; // all measurements taken
            }
            else {
                step = 1; // Orientate to next angle
            }               
        }      
    }

    if (step == 3) {
        PX=X;
        // reset static variables
        step        = 0;
        saved_angle = 0;
        X           = 0;
        next_angle  = 0;
        dist_RX     = 0;
        dist_LX     = 0;

        done        = 1;
    }

    orientate(0);
    return(PX);
    */
}

/* Replaces Position_Y sensor when it fails. */
double robust_PY(void) {
    static int descend  = 0;
    double     noisy_PY = Position_Y();
    double     r        = RangeDist();

    r = r * cos(robust_angle() * M_PI / 180);
    PY = PLAT_Y - r;
    if(fabs(noisy_PY-PY) <= 100 || descend) {
        descend = 1;
        if (r < 100) {
            PY = PY - 30;
        }
        return PY;
    }
    else {
        return(noisy_PY);
    }
}

/* Replaces Angle sensor when it fails. */
double robust_angle(void) {
    double angle = Angle();
    if (angle < 0) return fmod(fabs(angle),360);
    else           return angle;
}

/* Replaces Sonar sensor when it fails. */
void robust_SN(void) {
    double r = RangeDist();

    if (r>0 && r<180 && fabs(PLAT_X-PX)>180 && fabs(PLAT_Y-PY>180)) {

        // rotate to 0 degrees
        if (robust_angle()>1 && robust_angle()<359) {
            if (robust_angle()>=180) Rotate(360-robust_angle());
            else Rotate(-robust_angle());
        }

        // set vertical sonar measurements to RangeDist()
        for (int i=14; i<22; i++) SONAR_DIST[i] = r;

        // lander is to the right of platform
        // rotate CW to aim laser to the left and take sonar measurements
        if (PX > PLAT_X) {
            if (robust_angle()>1 && robust_angle()<359) {
                if (robust_angle()>=270) Rotate(450-robust_angle());
                else Rotate(90-robust_angle());
            }
            for (int i=22; i<32; i++) SONAR_DIST[i] = RangeDist();
        }

        // lander is to the left of platform
        // rotate CCW to aim laser to the left and take sonar measurements
        if (PX < PLAT_X) {
            if (robust_angle()>1 && robust_angle()<359) {
                if (robust_angle()>=90) Rotate(270-robust_angle());
                else Rotate(-robust_angle()-90);
            }
            for (int i=5; i<14; i++) SONAR_DIST[i] = RangeDist();
        }

        // limit max scanning rotation to 60 degrees
        if (robust_angle()>60 && robust_angle()<300) {
            if (robust_angle()>=180) Rotate(360-robust_angle());
            else Rotate(-robust_angle());
        }

    }

    return;
}

/* Rotates lander to use main thruster. */
void robust_main_thruster(double power) {
    // disable thrusters
    Main_Thruster(0);
    Left_Thruster(0);
    Right_Thruster(0);
    // no fail case (0 or 360 degrees)
    if (MT_OK) {
        if (robust_angle()>1 && robust_angle()<359) {
            if (robust_angle()>=180) Rotate(360-robust_angle());
            else Rotate(-robust_angle());
        }
    }
    // thruster fail cases (rotate only if not close enough to land on platform)
    // enable thrusters only if not close enough to land on platform
    if (fabs(PLAT_Y-PY)>40) {
        // main thruster fail, use left thruster (270 degrees)
        if (!MT_OK) {
            if (robust_angle()>1 && robust_angle()<359) {
                if (robust_angle()>=90) Rotate(270-robust_angle());
                else Rotate(-robust_angle()-90);
            }
        }
        // main and left thruster fail, use right thruster (90 degrees)
        if (!MT_OK && !LT_OK) {
            if (robust_angle()>1 && robust_angle()<359) {
                if (robust_angle()>=270) Rotate(450-robust_angle());
                else Rotate(90-robust_angle());
            }
        }
        Main_Thruster(power);                    //  MT_OK
        Left_Thruster(power * !MT_OK);           // !MT_OK
        Right_Thruster(power * !MT_OK * !LT_OK); // !MT_OK && !LT_OK
    }
    return;
}

/* Rotates lander to use left thruster. */
void robust_left_thruster(double power) {
    direction = RIGHT;
    // disable thrusters
    Main_Thruster(0);
    Left_Thruster(0);
    Right_Thruster(0);
    // no fail case (0 or 360 degrees)
    // rotate back to 0 degrees if ready to land
    if (LT_OK) {
        if (robust_angle()>1 && robust_angle()<359) {
            if (robust_angle()>=180) Rotate(360-robust_angle());
            else Rotate(-robust_angle());
        }
    }
    // thruster fail cases (rotate only if not close enough to land on platform)
    // enable thrusters only if not close enough to land on platform
    if (fabs(PLAT_Y-PY)>40) {
        // left thruster fail, use main thruster (90 degrees)
        if (!LT_OK) {
            if (robust_angle()>1 && robust_angle()<359) {
                if (robust_angle()>=270) Rotate(450-robust_angle());
                else Rotate(90-robust_angle());
            }
        }
        // left and main thruster fail, use right thruster (180 degrees)
        if (!LT_OK && !MT_OK) {
            if (robust_angle()>1 && robust_angle()<359) {
                Rotate(180-robust_angle());
            }
        }
        Left_Thruster(power);                    //  LT_OK
        Main_Thruster(power * !LT_OK * 0.6);     // !LT_OK
        Right_Thruster(power * !LT_OK * !MT_OK); // !LT_OK && !MT_OK
    }
    return;
}

/* Rotates lander to use right thruster. */
void robust_right_thruster(double power) {
    direction = LEFT;
    // disable thrusters
    Main_Thruster(0);
    Left_Thruster(0);
    Right_Thruster(0);
    // no fail case (0 or 360 degrees)
    // special case: don't use right thruster if only main thruster failed
    // rotate back to 0 degrees if ready to land
    if (!(RT_OK && LT_OK && !MT_OK) || fabs(PLAT_Y-PY)<40) {
        if (robust_angle()>1 && robust_angle()<359) {
            if (robust_angle()>=180) Rotate(360-robust_angle());
            else Rotate(-robust_angle());
        }
    }
    // thruster fail cases (rotate only if not close enough to land on platform)
    // enable thrusters only if not close enough to land on platform
    if (fabs(PLAT_Y-PY)>40) {
        // right thruster fail, use main thruster (270 degrees)
        if (!RT_OK) {
            if (robust_angle()>1 && robust_angle()<359) {
                if (robust_angle()>=90) Rotate(270-robust_angle());
                else Rotate(-robust_angle()-90);
            }
        }
        // right and main thruster fail, use left thruster (180 degrees)
        // special case: use only left thruster if only main thruster failed
        if ((!RT_OK && !MT_OK) || (RT_OK && LT_OK && !MT_OK)) {
            if (robust_angle()>1 && robust_angle()<359) {
                Rotate(180-robust_angle());
            }
        }
        Right_Thruster(power * !(RT_OK && LT_OK && !MT_OK)); //  RT_OK
        Main_Thruster(power * !RT_OK * 0.6);                 // !RT_OK
        Left_Thruster(power * !RT_OK * !MT_OK);              // !RT_OK && !MT_OK
    }
    return;
}

/* DOES NOT WORK */
/*
int orientate(double required_angle) {
    double rotate_angle = 0; 
 
    // Check if we are already at required angle
    if (robust_angle() > required_angle + 1 || robust_angle() < required_angle - 1) {
        return(1);
    }
    
    // Compute rotation angle
    if (required_angle > robust_angle()) {
        if (required_angle - robust_angle() > 180)
            rotate_angle = required_angle - robust_angle() - 360;
        else
            rotate_angle = required_angle - robust_angle();
    }
    else { // required_angle < Angle()
        if (robust_angle() - required_angle > 180)
            rotate_angle = required_angle - robust_angle() + 360;
        else 
        rotate_angle = required_angle - robust_angle();
    }
    
    // Rotate to required_angle
    robust_left_thruster(0);
    robust_right_thruster(0);
    robust_main_thruster(0);
    Rotate(rotate_angle);

    return(0);
}
*/

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

 srand(time(NULL)); // initialize random seed

 double VXlim;
 double VYlim;

 // Set velocity limits depending on distance to platform.
 // If the module is far from the platform allow it to
 // move faster, decrease speed limits as the module
 // approaches landing. You may need to be more conservative
 // with velocity limits when things fail.

 // modified to consider component failures
 if (fabs(PX-PLAT_X)>200) VXlim=7;//25 - (5*(!MT_OK+!LT_OK+!RT_OK));
 else if (fabs(PX-PLAT_X)>100) VXlim=6;//15 - (5*(!MT_OK+!LT_OK+!RT_OK));
 else VXlim=5;

 // modified to consider component failures
 if (PLAT_Y-PY>200) VYlim=-6;//-20 + (5*(!MT_OK+!LT_OK+!RT_OK));
 else if (PLAT_Y-PY>100) VYlim=-5;//-10 + (3*(!MT_OK+!LT_OK+!RT_OK));  // These are negative because they
 else VYlim=-4;				                               // limit descent velocity

 // Ensure we will be OVER the platform when we land
 if (fabs(PLAT_X-PX)/fabs(VX)>1.25*fabs(PLAT_Y-PY)/fabs(VY)) VYlim=0;

 // IMPORTANT NOTE: The code below assumes all components working
 // properly. IT MAY OR MAY NOT BE USEFUL TO YOU when components
 // fail. More likely, you will need a set of case-based code
 // chunks, each of which works under particular failure conditions.

 // Check for rotation away from zero degrees - Rotate first,
 // use thrusters only when not rotating to avoid adding
 // velocity components along the rotation directions
 // Note that only the latest Rotate() command has any
 // effect, i.e. the rotation angle does not accumulate
 // for successive calls.

 // runs only at start of program
 if (frames == 0) {
     // rotate to 0 degrees
     if (Angle()>1&&Angle()<359) {
         if (Angle()>=180) Rotate(360-Angle());
         else Rotate(-Angle());
         return;
     }
     // take initial measurements before anything fails
     VX = Velocity_X();
     VY = Velocity_Y();
     PX = Position_X();
     PY = Position_Y();

     init_X = VX; // not used
 }
 frames++;
 
 if (fabs(Velocity_X()-VX)>1.5) VX_OK = 0; // check if Velocity_X sensor failed
 if (fabs(Velocity_Y()-VY)>1.5) VY_OK = 0; // check if Velocity_Y sensor failed
 if (fabs(Position_X()-PX)>100) PX_OK = 0; // check if Position_X sensor failed
 if (fabs(Position_Y()-PY)>100) PY_OK = 0; // check if Position_Y sensor failed

 // check if Sonar sensor failed
 if (fabs(PLAT_X-PX)>150 && fabs(PLAT_Y-PY)>150 && RangeDist()<250 && SONAR_DIST[18]<0)
     SN_OK = 0;

 if (VX_OK)  VX = Velocity_X(); // use Velocity_X sensor if it still works
 else        VX = robust_VX();  // otherwise use robust sensor

 if (VY_OK)  VY = Velocity_Y(); // use Velocity_Y sensor if it still works
 else        VY = robust_VY();  // otherwise use robust sensor

 if (PX_OK)  PX = Position_X(); // use Position_X sensor if it still works
 else        PX = robust_PX();  // otherwise use robust sensor

 if (PY_OK)  PY = Position_Y(); // use Position_Y sensor if it still works
 else        PY = robust_PY();  // otherwise use robust sensor

 /* print sensor status */
 //printf("%d%d%d%d\n", VX_OK, VY_OK, PX_OK, PY_OK, SN_OK);

 /* print state variables */ 
 //printf("Velocity_X=%f,\t Velocity_Y=%f,\t Position_X=%f,\t Position_Y=%f,\t Angle=%f,\t Range=%f\n", VX, VY, PX, PY, robust_angle(), RangeDist());

 /* print sonar measurements */
 /*
 int i;
 for (i=0; i<36; i++) {
     printf("Sonar[%d]:%f\n", i, SONAR_DIST[i]);
 }
 */

 // Module is oriented properly, check for horizontal position
 // and set thrusters appropriately.
 if (PX>PLAT_X)
 {
  // Lander is to the LEFT of the landing platform, use Right thrusters to move
  // lander to the left.
  Left_Thruster(0);	// Make sure we're not fighting ourselves here!
  if (VX>(-VXlim)) robust_right_thruster((VXlim+fmin(0,VX))/VXlim);
  else
  {
   // Exceeded velocity limit, brake
   Right_Thruster(0);
   robust_left_thruster(fabs(VXlim-VX));
  }
 }
 else
 {
  // Lander is to the RIGHT of the landing platform, opposite from above
  Right_Thruster(0);
  if (VX<VXlim) robust_left_thruster((VXlim-fmax(0,VX))/VXlim);
  else
  {
   Left_Thruster(0);
   robust_right_thruster(fabs(VXlim-VX));
  }
 }

 // Vertical adjustments. Basically, keep the module below the limit for
 // vertical velocity and allow for continuous descent. We trust
 // Safety_Override() to save us from crashing with the ground.
 if (VY<VYlim) robust_main_thruster(1.0);
 else Main_Thruster(0);
}

void Safety_Override(void)
{
 /*
   This function is intended to keep the lander from
   crashing. It checks the sonar distance array,
   if the distance to nearby solid surfaces and
   uses thrusters to maintain a safe distance from
   the ground unless the ground happens to be the
   landing platform.

   Additionally, it enforces a maximum speed limit
   which when breached triggers an emergency brake
   operation.
 */

/**************************************************
 TO DO: Modify this function so that it can do its
        work even if components or sensors
        fail
**************************************************/

/**************************************************
  How this works:
  Check the sonar readings, for each sonar
  reading that is below a minimum safety threshold
  AND in the general direction of motion AND
  not corresponding to the landing platform,
  carry out speed corrections using the thrusters
**************************************************/

 double DistLimit;
 double Vmag;
 double dmin;

 // Establish distance threshold based on lander
 // speed (we need more time to rectify direction
 // at high speed)
 Vmag=VX*VX;
 Vmag+=VY*VY;

 DistLimit=fmax(75,Vmag);

 // If we're close to the landing platform, disable
 // safety override (close to the landing platform
 // the Control_Policy() should be trusted to
 // safely land the craft)
 if (fabs(PLAT_X-PX)<150&&fabs(PLAT_Y-PY)<150) return;

 // Determine the closest surfaces in the direction
 // of motion. This is done by checking the sonar
 // array in the quadrant corresponding to the
 // ship's motion direction to find the entry
 // with the smallest registered distance

 /* use robust sensor if Sonar sensor fails */
 if (!SN_OK) robust_SN();

 // Horizontal direction.
 dmin=1000000;
 if (VX>0)
 {
  for (int i=5;i<14;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 else
 {
  for (int i=22;i<32;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 // Determine whether we're too close for comfort. There is a reason
 // to have this distance limit modulated by horizontal speed...
 // what is it?
 if (dmin<DistLimit*fmax(.25,fmin(fabs(VX)/5.0,1)))
 { // Too close to a surface in the horizontal direction

// removed from starter code
/*
  if (Angle()>1&&Angle()<359)
  {
   if (Angle()>=180) Rotate(360-Angle());
   else Rotate(-Angle());
   return;
  }
*/

  if (VX>0){
   robust_right_thruster(1.0);
   Left_Thruster(0.0);
  }
  else
  {
   robust_left_thruster(1.0);
   Right_Thruster(0.0);
  }
 }

 // Vertical direction
 dmin=1000000;
 if (VY>5)      // Mind this! there is a reason for it...
 {
  for (int i=0; i<5; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
  for (int i=32; i<36; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 else
 {
  for (int i=14; i<22; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 if (dmin<DistLimit)   // Too close to a surface in the horizontal direction
 {

// removed from starter code
/*
  if (Angle()>1||Angle()>359)
  {
   if (Angle()>=180) Rotate(360-Angle());
   else Rotate(-Angle());
   return;
  }
*/

  if (VY>2.0){
   Main_Thruster(0.0);
  }
  else
  {
   robust_main_thruster(1.0);
  }
 }
}
