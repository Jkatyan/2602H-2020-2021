#ifndef MOTORS_HPP
#define MOTORS_HPP

//PORTS
#define R1 20 //Right Front
#define R2 11 //Right Back
#define L1 10 //Left Front
#define L2 1 //Left Back

#define LIFT 12 //Lift
#define INTA 19 //RIGHT Intake
#define INTB 15 //LEFT Intake
#define MOGO 7 //MOGO

#define GYROPORT 5 //GYRO
#define TRAYPOT 8

//MOTOR CONFIG
/*DIRECTION*/ #define RFREV false
/*true = rev*/#define RBREV false
              #define LFREV true
              #define LBREV true

              #define LIFTREV false
              #define INTAREV false
              #define INTBREV false
              #define MOGOREV false

/*GEARSET*/   #define RFG E_MOTOR_GEARSET_18
/*18 = Green*/#define RBG E_MOTOR_GEARSET_18
/*06 = Blue*/ #define LFG E_MOTOR_GEARSET_18
/*36 = Red*/  #define LBG E_MOTOR_GEARSET_18

              #define LIFTG E_MOTOR_GEARSET_18
              #define INTAG E_MOTOR_GEARSET_18
              #define INTBG E_MOTOR_GEARSET_18
              #define MOGOG E_MOTOR_GEARSET_36

//CHASSIS CORRECTION
#define RC 1
#define LC 1

//PID Tuning
#define LARGE 99999999

/* Chassis PID */
#define DRIVEKP 0.95
#define DRIVEKI 0.01
#define DRIVEKD 0.1

/* Gyro Drive PID */
#define GYRODKP 1.2
#define GYRODKI 0
#define GYRODKD 0.6

/* Gyro PID */
#define GYROKP 2.3
#define GYROKI 0
#define GYROKD 0.38

/* Arm PID */
#define ARMKP 0.7
#define ARMKI 0
#define ARMKD 0.1

/* Tray PID */
#define TRAYKP 0.5157
#define TRAYKI 0
#define TRAYKD 0.0001

/* Intake A PID */
#define INTAKEAKP 1
#define INTAKEAKI 0
#define INTAKEAKD 0.1

/* Intake B PID */
#define INTAKEBKP 1
#define INTAKEBKI 0
#define INTAKEBKD 0.1

/* Turn PID */
#define TURNKP 1
#define TURNKI 0
#define TURNKD 0.1

#endif
