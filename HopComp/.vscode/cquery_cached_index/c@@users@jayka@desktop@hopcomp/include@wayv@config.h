#ifndef _CONFIG_H_
#define _CONFIG_H_

//Motors
#define LEFT_FRONT 11 //Port #
  #define LF_REVERSED true //Reversed?

#define LEFT_BACK 12
  #define LB_REVERSED true

#define RIGHT_FRONT 14
  #define RF_REVERSED false

#define RIGHT_BACK 15
  #define RB_REVERSED false

#define LEFT_INTAKE 13
  #define LI_REVERSED false

#define RIGHT_INTAKE 18
  #define RI_REVERSED false

#define MID_ROLLER 20
  #define MID_REVERSED false

#define TOP_ROLLER 19
  #define TOP_REVERSED false

//Sensors
#define RIGHT_TRACKER_A 0 //Port #
#define RIGHT_TRACKER_B 0
#define LEFT_TRACKER_A 0
#define LEFT_TRACKER_B 0
#define BACK_TRACKER_A 3
#define BACK_TRACKER_B 4
#define LINE_SENSOR 2
#define MID_LINE_SENSOR 1
#define IMU_PORT 2

//PID
#define DRIVE_RIGHT_KP 0.37 //Right Side Straight Drive P Value
#define DRIVE_RIGHT_KD 0.2 //Right Side Straight Drive D Value
#define DRIVE_LEFT_KP 0.37 //Left Side Straight Drive P Value
#define DRIVE_LEFT_KD 0.2 //Left Side Straight Drive D Value

#define ANGLE_KP 0.2 //Side Tracker Angle Correction P Value

#define TURN_KP 1.52 //Turning P Value
#define TURN_KD 0 //Turning D Value

//Limits
#define RC 1 //Right Side Speed Coefficient
#define LC 1 //Left Side Speed Coefficient
#define MAX_ACCEL 0.15 //Slew
#define LINE_LIMIT 2950

//Turning
#define TURN_CONSTANT 0

#endif
