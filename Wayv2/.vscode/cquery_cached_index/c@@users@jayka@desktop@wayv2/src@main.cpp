#include "main.h"
#include "odometry.hpp"

pros::ADIEncoder sideEnc ('c', 'd');
pros::ADIEncoder rightEnc ('h', 'g');
pros::ADIEncoder leftEnc ('e', 'f');

pros::Motor rf (14);
pros::Motor rb (15);
pros::Motor lf (11, true);
pros::Motor lb (12, true);

pros::Controller user (CONTROLLER_MASTER);

void initialize() {
	pros::lcd::initialize();
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
	while (1){
		updatePosition();

		pros::lcd::print(0, "X: %f", getX());
		pros::lcd::print(1, "Y: %f", getY());
		pros::lcd::print(2, "Angle: %f", getAngleDegrees());

		pros::lcd::print(4, "Side Encoder: %d", sideEnc.get_value());
		pros::lcd::print(5, "Left Encoder: %d", -rightEnc.get_value());
		pros::lcd::print(6, "Right Encoder: %d", leftEnc.get_value());

		rf.move(user.get_analog(ANALOG_RIGHT_Y));
		rb.move(user.get_analog(ANALOG_RIGHT_Y));
		lf.move(user.get_analog(ANALOG_LEFT_Y));
		lb.move(user.get_analog(ANALOG_LEFT_Y));

		pros::delay(10);
	}
}
