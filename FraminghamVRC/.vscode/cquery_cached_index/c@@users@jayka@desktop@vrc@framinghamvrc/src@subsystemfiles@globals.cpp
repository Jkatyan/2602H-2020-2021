#include "main.h"

//MOTORS
pros::Motor lB(LB, pros::LBG, LBREV, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor lF(LF, pros::LFG, LFREV, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor rB(RB, pros::RBG, RBREV, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor rF(RF, pros::RFG, RFREV, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor liftRight(LIFTRIGHT, pros::LIFTRG, LIFTRIGHTREV, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor liftLeft(LIFTLEFT, pros::LIFTLG, LIFTLEFTREV, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor clawRight(CLAWRIGHT, pros::CLAWRG, CLAWRIGHTREV, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor clawLeft(CLAWLEFT, pros::CLAWLG, CLAWLEFTREV, pros::E_MOTOR_ENCODER_COUNTS);

//CONTROLLERS
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Controller partner(pros::E_CONTROLLER_PARTNER);
