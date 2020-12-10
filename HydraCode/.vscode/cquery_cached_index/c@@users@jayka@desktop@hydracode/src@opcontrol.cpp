#include "main.h"
#include "PID.hpp"
#include "meInYourLife.h"
#include "motors.hpp"

PID liftPID;
PID diagPID;
PID vertPID;

int lastTime = 0;
int lastTimeIntake = 0;

int pidFix = 0;

enum positions {
	home = 100, //Home
	one = 200, //Ground Level
	two = 300, //Low Tower
	three = 400 //Mid Tower
} positions;
enum positions liftTarget;

enum aPos {setA = 100} aPos; enum aPos aTarg = setA;
enum bPos {setB = 100} bPos; enum bPos bTarg = setB;

void opcontrol() {

	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Controller partner(pros::E_CONTROLLER_PARTNER);
	pros::Motor lB(L2, pros::LBG, LBREV, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor lF(L1, pros::LFG, LFREV, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rB(R2, pros::RBG, RBREV, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rF(R1, pros::RFG, RFREV, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor door(DOOR, pros::DOORG, DOORREV, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor lift(LIFT, pros::LIFTG, LIFTREV, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor diag(INTA, pros::INTAG, INTAREV, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor vert(INTB, pros::INTBG, INTBREV, pros::E_MOTOR_ENCODER_DEGREES);

	lv_obj_clean(lv_scr_act());
		liftPID = pidInit(1, 0, 0, 99999,999999, 999999, 99999);
		diagPID = pidInit(1, 0, 0, 99999,999999, 999999, 99999);
		vertPID = pidInit(1, 0, 0, 99999,999999, 999999, 99999);

	while (true) {
			pidFix = lift.get_position(); //Sets current lift positions
				//driveFunction
				if (master.get_digital(DIGITAL_DOWN)){ //Mogo Lift Down
					lF.move(60);
			    lB.move(-60);
			    rF.move(60);
			    rB.move(-60);
				}
				else if (master.get_digital(DIGITAL_UP)){ //Mogo Lift Up
					lF.move(-60);
			    lB.move(60);
			    rF.move(-60);
			    rB.move(60);
				}
				else{	//Drive Code
				lF.move(LC*(master.get_analog(ANALOG_LEFT_Y)));
		    lB.move(LC*(master.get_analog(ANALOG_LEFT_Y)));
		    rF.move(RC*(master.get_analog(ANALOG_RIGHT_Y)));
		    rB.move(RC*(master.get_analog(ANALOG_RIGHT_Y)));
			}
				//lift.move((master.get_digital(DIGITAL_R1))? 127: ((master.get_digital(DIGITAL_R2))? -127: 0));

				if (master.get_digital(DIGITAL_R1)){
					lift.move(127);
					double(liftTarget) = lift.get_position();
					lastTime = pros::millis();
				}
				else if (master.get_digital(DIGITAL_R2)){
					lift.move(-127);
					double(liftTarget) = lift.get_position();
					lastTime = pros::millis();
				}
				else if((pros::millis() - lastTime) > 200){
				lift.move(pidCalculate(liftPID, liftTarget, lift.get_position()));
				}

			if (master.get_digital(DIGITAL_LEFT)){liftTarget = two;} //Set Lift Positions
			else if (master.get_digital(DIGITAL_RIGHT)){liftTarget = three;}
			else if (partner.get_digital(DIGITAL_DOWN)){liftTarget = one;}

			if (master.get_digital(DIGITAL_L1)){
				diag.move(127);
				vert.move(127);
				double(aPos) = diag.get_position();
				double(bPos) = vert.get_position();
				lastTimeIntake = pros::millis();
			}
			else if (master.get_digital(DIGITAL_L2)){
				diag.move(-127);
				vert.move(-127);
				double(aPos) = diag.get_position();
				double(bPos) = vert.get_position();
				lastTimeIntake = pros::millis();
			}
			else if((pros::millis() - lastTimeIntake) > 200){
				diag.move(pidCalculate(diagPID, aPos, diag.get_position()));
				vert.move(pidCalculate(vertPID, bPos, vert.get_position()));
			}

			pros::delay(25);
	}
}
