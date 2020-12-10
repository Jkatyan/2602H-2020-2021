#include "main.h"

//Drive Motors
pros::Motor LF (LEFT_FRONT, MOTOR_GEARSET_18, LF_REVERSED, MOTOR_ENCODER_COUNTS);
pros::Motor LB (LEFT_BACK, MOTOR_GEARSET_18, LB_REVERSED, MOTOR_ENCODER_COUNTS);
pros::Motor RF (RIGHT_FRONT, MOTOR_GEARSET_18, RF_REVERSED, MOTOR_ENCODER_COUNTS);
pros::Motor RB (RIGHT_BACK, MOTOR_GEARSET_18, RB_REVERSED, MOTOR_ENCODER_COUNTS);

//Intake Motors
pros::Motor rightIntake(RIGHT_INTAKE, RI_REVERSED);
pros::Motor leftIntake(LEFT_INTAKE, LI_REVERSED);

//Conveyor Motors
pros::Motor midRoller(MID_ROLLER, MID_REVERSED);
pros::Motor topRoller(TOP_ROLLER, TOP_REVERSED);

//Sensors
pros::ADIAnalogIn line(LINE_SENSOR);
pros::ADIAnalogIn midLine(MID_LINE_SENSOR);
pros::ADIEncoder sideEnc (BACK_TRACKER_A, BACK_TRACKER_B);
pros::ADIEncoder rightEnc (RIGHT_TRACKER_A, RIGHT_TRACKER_B);
pros::ADIEncoder leftEnc (LEFT_TRACKER_A, LEFT_TRACKER_B);

//Misc
pros::Controller master(CONTROLLER_MASTER);
pros::Imu imu (IMU_PORT);
