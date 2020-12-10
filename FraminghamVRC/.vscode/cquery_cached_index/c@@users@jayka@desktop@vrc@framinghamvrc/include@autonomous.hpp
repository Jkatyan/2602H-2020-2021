#ifndef AUTONOMOUS_HPP
#define AUTONOMOUS_HPP

#include "main.h"
#include "PID.hpp"

extern PID drivePID; //Regular Drive PID
extern PID gyroDrivePID; //Gyro OP

extern float lastSlewTime;
extern float maxAccel;
extern float lastSlewRate;

extern lv_obj_t * myLabelGyro;extern char bufG[100];
extern lv_obj_t * myLabelEnc;extern char bufE[100];
extern lv_obj_t * myLabelTarget;extern char bufT[100];

extern int preset1; //Red Preset
extern int preset2; //Blue Preset

void enablePreset(int preset);
/*
*Applies preset code to the rest of the autonomous.
*PRESETS:
  1: RED
  2: BLUE
*/

struct preset1{ //RED side preset
  int side = 1;
  bool slow = false;
  bool slew = true;
  float rightCorrection = RC;
  float leftCorrection = LC;
} extern p1;

struct preset2{ //BLUE side preset
  int side = -1;
  bool slow = false;
  bool slew = true;
  float rightCorrection = RC;
  float leftCorrection = LC;
} extern p2;

int driveTarget(int target, float angle, int time, float speed);
// Same code as RawDriveTarget - To be used only when presets are enabled.

int targetTurn(int target, int time, int turn, float speed);
// Same code as RawDriveTurn - To be used only when presets are enabled.

void rawDriveTarget (int side, float angle, int target, int time, int turn, float speed, bool slow, bool slew, float rightCorrection, float leftCorrection);
/*
*Side - Side of the field you're on. 1 = Red, -1 = Blue.
*Angle - Uses Gyro to correct angle while driving / RELATIVE TO STARTING ALIGNMENT.
*Target - Desired Target Value
*Time - Allowed time for the function to run before it ends (ms)
*Turn - 0 for no turn, 1 for right turn, 2 for left Turn
*Speed - Speed multiplier for the drive (Ex. 0.5 50% speed, 1 100% speed)
*Slow - True: Chassis will drive slower to the target / False: Chassis will drive at regular speed
*Slew - True: Applies 1D Motion Profiling / False: Uses only PID
* DISABLED Accuracy - How close the robot needs to come to the target. Ex. 20 Accuracy means +- 20 from target. //Not in Function
*Correction - Correction coefficient for uneven drive. Set to 1 for no correction.
*/
void rawTargetTurn (int target, int time, int turn, float speed, bool slow, bool slew, float rightCorrection, float leftCorrection);
/*
*Same code as void driveTarget, however this code is NOT gyro assisted.
*/
void gyroTurn(int target, int accuracy, int time, float speed);
/*
 Turns using the GYRO.
*Target - Angle of Turn
*Accuracy - How close to the target the robot is
*Time - Amount of time allowed for the turn. (In MS)
*Speed - Speed coefficient. Ex. 1 is max speed, 0.5 is half speed.
*/
void motorTarget(int port, float p, float i, float d, int special, int target, int time, float speed, float accel, bool slew);/*
Single Motor Motion Function:
  -Port: Motor port number
  -Float p, i, d: Motor PID values
  -Int Special: SPECIAL VALUE (ROBOT SPECIFIC)
    -0: Take Value from MOTOR
    -1: Take value from TRAY POTENTIOMETER
*/
void stop();
/*
*Stops all drive motors / resets values to 0
*/
void reset();
/*
*Sets all motors to 0 (position).
*/
float slewRateCalculate (float desiredRate);
/*
A basic 1D motion profiler that handles acceleration.
*/
    float motorSlew (float desiredRate, float maxAccelTray);
#endif
