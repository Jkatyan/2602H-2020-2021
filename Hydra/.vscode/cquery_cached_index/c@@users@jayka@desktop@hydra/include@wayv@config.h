#ifndef _CONFIG_H_
#define _CONFIG_H_

//SENSORS
  //ports
  #define TRACKER_A 3
  #define TRACKER_B 4

  #define LINE_SENSOR 2 //Indexer
  #define MID_LINE_SENSOR 1

  #define ULTRA_IN 3 //Angled Ultra
  #define ULTRA_OUT 4

  #define IMU_PORT 2

//INTAKE SETTINGS
  //ports
  #define LEFT_INTAKE 13
  #define RIGHT_INTAKE 18
  #define MID_ROLLER 20
  #define TOP_ROLLER 19

  //line sensor
  #define LINE_LIMIT 2950

  //speeds
  #define MAX_SCORE_SPEED 127
  #define MID_SCORE_SPEED 0

//CHASSIS SETTINGS
  //odom
  #define SIDE_WHEEL_DIAM 5.417
  #define BACK_WHEEL_DIAM 2.75

  //ports
  #define LEFT_FRONT 11  //Negative Means Reversed
  #define LEFT_BACK 12
  #define RIGHT_FRONT 14
  #define RIGHT_BACK 15

  //CUSTOM SETTINGS
  //pid constants
    //distance PID
    #define CUSTOM_DKP 0.39
    #define CUSTOM_DKD 0.2

    //angle PID
    #define CUSTOM_AKP 0.981
    #define CUSTOM_AKD 0

  //drive correction
  #define RC 0.978
  #define LC 1

  //slew
  #define MAX_ACCEL 0.15

  //*OKAPI SETTINGS
  //odom constant
  #define WHEEL_DIAM 5.417_in //If using a gear ratio, diam is # of teeth on powered gear / teeth on output * wheel diam
  #define WHEEL_TRACK 10.75_in //Distance in inches between left and right side wheels

  //pid constants
    //distance PID
    #define DRIVE_KP .0019 //Straight driving PD
    #define DRIVE_KD .00015

    //turn PID
    #define TURN_KP .00365 //Turning PD
    #define TURN_KD .0

    //angle PID
    #define ANGLE_KP .0001 //Chassis Angle P

  //2d motion profiling constants
  #define VEL 0.5 // Maximum linear velocity of the Chassis in m/s
  #define LIN 2 // Maximum linear acceleration of the Chassis in m/s/s
  #define JER 2 // Maximum linear jerk of the Chassis in m/s/s/s

#endif
