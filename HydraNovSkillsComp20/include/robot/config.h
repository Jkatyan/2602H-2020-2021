#ifndef _CONFIG_H_
#define _CONFIG_H_

//Motor Ports
#define LEFT_FRONT 11
#define LEFT_BACK 12
#define RIGHT_FRONT 14
#define RIGHT_BACK 15
#define LEFT_INTAKE 13
#define RIGHT_INTAKE 18
#define MID_ROLLER 20
#define TOP_ROLLER 19

//Sensors
#define RIGHT_TRACKER_A 7
#define RIGHT_TRACKER_B 8
#define LEFT_TRACKER_A 5
#define LEFT_TRACKER_B 6
#define BACK_TRACKER_A 3
#define BACK_TRACKER_B 4
#define LINE_SENSOR 2
#define MID_LINE_SENSOR 1
#define IMU_PORT 2
#define OPTICAL_PORT 21

//PID
#define DRIVE_RIGHT_KP 0.21 //Right Side Straight Drive P Value
#define DRIVE_RIGHT_KD 0.225 //Right Side Straight Drive D Value
#define DRIVE_LEFT_KP 0.205 //Left Side Straight Drive P Value
#define DRIVE_LEFT_KD 0 //Left Side Straight Drive D Value

#define TURN_KP 1.35 //Turning P Value
#define TURN_KD 0.1 //Turning D Value

//Misc. Constants
#define RC -0.989 //Right Side Speed Coefficient
#define LC 1 //Left Side Speed Coefficient
#define MAX_ACCEL 0.15 //Slew
#define LINE_LIMIT 2950
#define MAX_SCORE_SPEED 127

//Turning
#define TURN_CONSTANT 0.325

#endif
