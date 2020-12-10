#ifndef PORTCONFIG_HPP
#define PORTCONFIG_HPP

//PORTS
#define RF 18 //Right Front
#define RB 20 //Right Back
#define LF 19 //Left Front
#define LB 1 //Left Back

#define LIFTRIGHT 17 //Lift Right
#define LIFTLEFT 10 //Lift Left
#define CLAWRIGHT 16 //Claw Right
#define CLAWLEFT 9 //Claw Left

#define GYROPORT 3

//MOTOR CONFIG
/*DIRECTION*/ #define RFREV false
/*true = rev*/#define RBREV false
              #define LFREV true
              #define LBREV true

              #define LIFTRIGHTREV false
              #define LIFTLEFTREV true
              #define CLAWRIGHTREV false
              #define CLAWLEFTREV true

/*GEARSET*/   #define RFG E_MOTOR_GEARSET_18
/*18 = Green*/#define RBG E_MOTOR_GEARSET_18
/*06 = Blue*/ #define LFG E_MOTOR_GEARSET_18
/*36 = Red*/  #define LBG E_MOTOR_GEARSET_18

              #define LIFTRG E_MOTOR_GEARSET_36
              #define LIFTLG E_MOTOR_GEARSET_36
              #define CLAWRG E_MOTOR_GEARSET_36
              #define CLAWLG E_MOTOR_GEARSET_36

//CHASSIS CORRECTION
#define RC 0.97
#define LC 1

//GYRO CORRECTION
#define GC 1
#endif
