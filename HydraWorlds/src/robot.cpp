#include "main.h"

using namespace pros;

//Motors
#define LF 14
#define LB 15
#define RF 20
#define RB 17

#define LEFT_INTAKE 12
#define RIGHT_INTAKE 19
#define MID_ROLLER 11
#define TOP_ROLLER 13

//Sensors
#define IMUA_PORT 21
#define IMUB_PORT 10

#define LEFT_ENC_A 1
#define LEFT_ENC_B 2
#define RIGHT_ENC_A 7
#define RIGHT_ENC_B 8

//Devices
Controller master (CONTROLLER_MASTER);

Motor leftFront (LF, true);
Motor leftBack (LB, true);
Motor rightFront (RF);
Motor rightBack (RB);

Motor leftIntake (LEFT_INTAKE);
Motor rightIntake (RIGHT_INTAKE, true);
Motor midRoller (MID_ROLLER, true);
Motor topRoller (TOP_ROLLER, true);

Imu imuA (IMUA_PORT);
Imu imuB (IMUB_PORT);

ADIEncoder sEncoder (LEFT_ENC_A,LEFT_ENC_B);
ADIEncoder rEncoder (RIGHT_ENC_A,RIGHT_ENC_B, true);

//Tasks
Task *tracking = nullptr;
