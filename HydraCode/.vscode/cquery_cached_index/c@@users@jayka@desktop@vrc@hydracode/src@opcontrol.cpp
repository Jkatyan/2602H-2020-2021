#include "main/main.h"
#include "misc/PID.hpp"
#include "motors.hpp"
#include "misc/gyro.hpp"

PID liftPID;
PID intaPID;
PID intbPID;

int lastTime = 0;
int lastTimeIntake = 0;

enum positions {
	home = 0, //Home
	one = 0, //Ground Level
	two = 0, //Low Tower
	three = 0 //Mid Tower
} positions;
double liftTarget;

enum aPos {setA = 100} aPos; enum aPos aTarg = setA;
enum bPos {setB = 100} bPos; enum bPos bTarg = setB;

void opcontrol() {

	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Controller partner(pros::E_CONTROLLER_PARTNER);
	pros::Motor lB(L2, pros::LBG, LBREV, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor lF(L1, pros::LFG, LFREV, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rB(R2, pros::RBG, RBREV, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rF(R1, pros::RFG, RFREV, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor mogo(MOGO, pros::MOGOG, MOGOREV, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor lift(LIFT, pros::LIFTG, LIFTREV, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor inta(INTA, pros::INTAG, INTAREV, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor intb(INTB, pros::INTBG, INTBREV, pros::E_MOTOR_ENCODER_DEGREES);

	lv_obj_clean(lv_scr_act());
		liftPID = pidInit(1, 0, 0, 99999,999999, 999999, 99999);
		intaPID = pidInit(1, 0, 0, 99999,999999, 999999, 99999);
		intbPID = pidInit(1, 0, 0, 99999,999999, 999999, 99999);
/*
		lv_obj_t * myLabelGyro;
		char bufG[100];
		Gyro gyro;

		myLabelGyro = lv_label_create(lv_scr_act(), NULL);
		lv_label_set_text(myLabelGyro, "Gyro Value:");
		lv_obj_align(myLabelGyro, NULL, LV_ALIGN_CENTER, 0, 0);
*/
	while (true) {
		//lv_obj_clean(lv_scr_act());


//Chassis
				lF.move(LC*(master.get_analog(ANALOG_LEFT_Y)));
		    lB.move(LC*(master.get_analog(ANALOG_LEFT_Y)));
		    rF.move(RC*(master.get_analog(ANALOG_RIGHT_Y)));
		    rB.move(RC*(master.get_analog(ANALOG_RIGHT_Y)));

//Lift
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

			if (partner.get_digital(DIGITAL_LEFT)){positions = two;} //Set Lift Positions
			else if (partner.get_digital(DIGITAL_UP)){positions = three;}
			else if (partner.get_digital(DIGITAL_DOWN)){positions = one;}

//Intake
			if (master.get_digital(DIGITAL_L1)){
				inta.move(127);
				intb.move(127);
				double(aPos) = inta.get_position();
				double(bPos) = intb.get_position();
				lastTimeIntake = pros::millis();
			}
			else if (master.get_digital(DIGITAL_L2)){
				inta.move(-127);
				intb.move(-127);
				double(aPos) = inta.get_position();
				double(bPos) = intb.get_position();
				lastTimeIntake = pros::millis();
			}
			else if((pros::millis() - lastTimeIntake) > 200){
				inta.move(pidCalculate(intaPID, aPos, inta.get_position()));
				intb.move(pidCalculate(intbPID, bPos, intb.get_position()));
			}

			pros::delay(10);
	}
}
