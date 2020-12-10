#include "autonSel.h"
#include "PID.hpp"
#include "autonomous.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor lB(L2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor lF(L1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rB(R2, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rF(R1, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor door(DOOR, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor lift(LIFT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor diag(INTA, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor vert(INTB, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

int side;

int liftAim = 520;
/*
pros::Motor arF (R1);
pros::Motor arB (R2, true);
pros::Motor alF (L1);
pros::Motor alB (L2, true);

pros::Motor alift (LIFT, true);

pros::Motor adiag(INTA);
pros::Motor avert(INTB);

pros::Motor adoor (DOOR, true);
*/
void autonomous() {
  drivePID = pidInit (0.34, 0, 0.1, 0, 100.0,5,15);
	slowDrivePID = pidInit (0.25, 0, 0.1, 0, 100.0,5,15);
	gyroDrivePID = pidInit(1.2, 0, 0.6, 0, 40,999999,9999999);
	gyroArcPID = pidInit(3, 0, 0.38, 999999,9999999,999999,99999);
	gyroPID = pidInit(2.3,  0, 0.38, 0, 10,99999,999999);
	aLiftPID = pidInit(1, 0, 0, 99999,999999, 999999, 99999);
	gyroPrecisePID = pidInit(2.3,  0.3, 0.1, 0, 3,99999,999999);
	aLiftTarget = 0;

  stop();
  gyro.reset();
  lastSlewTime = pros::millis();
  pros::delay(20);
  side = 1;

  //Motion Profiling
  auto chassis = ChassisControllerFactory::create(
    {L1, L2}, // Left motors
    {R1, R2},   // Right motors
    AbstractMotor::gearset::green, // Torque gearset
    {4_in, 14_in} // 4 inch wheels, 12.5 inch wheelbase width
  );

  auto profileController = AsyncControllerFactory::motionProfile( //Other Option: 1 4 10
    1.0,  // Maximum linear velocity of the Chassis in m/s
    10.0,  // Maximum linear acceleration of the Chassis in m/s/s
    14.0, // Maximum linear jerk of the Chassis in m/s/s/s
    chassis // Chassis Controller
  );

//Move Code
switch(setAuton){
  case 1: //Red 1 1

  break;
  case 2: //Red 1 2

  break;
  case 3: //Red 2 1

  break;
  case 4: //Red 2 2

  break;
  case 5: //Red 3 1

  break;
  case 6: //Red 3 2

  break;
  case 7: //Red 4 1

  break;
  case 8: //Red 4 2

  break;
  case 9: //Blue 1 1

  break;
  case 10: //Blue 1 2

  break;
  case 11: //Blue 2 1

  break;
  case 12: //Blue 2 2

  break;
  case 13: //Blue 3 1

  break;
  case 14: //Blue 3 2

  break;
  case 15: //Blue 4 1

  break;
  case 16: //Blue 4 2

  break;
  case 17: //Programming Skills

  break;
}
}
void moveTime(int left, int right, int time) {
  auxiliary();
  lF.move(-left);
  lB.move(-left);
  rF.move(right);
  rB.move(right);
  int startTime = pros::millis();
  while (pros::millis() - startTime < time) {
    auxiliary();
    pros::delay(10);
  }
//  pros::delay(time);
  lF.move(0);
  lB.move(0);
  rF.move(0);
  rB.move(0);
  pros::delay(500);
}
void driveTarget(int target, float angle, int accuracy, int time) {
	driveTarget(target, angle, accuracy, time, 1, true);
}

void driveTarget(int target, float angle, int accuracy, int time, float max, bool slew) {

	int startTime = pros::millis();
	bool atTarget = false;
	//int targetAngle = gyro.get_value()/10.0;
	int targetAngle = angle;
	int repsAtTarget = 0;
  float driveEnc = 0;
	//go into the loop that will repeat to update motor values and break when at target
	while (!atTarget && (pros::millis()-startTime) < time) {
		auxiliary();
		//pros::lcd::print(0, "Angle: %lf", gyro.get_value());
    driveEnc = -(lB.get_position() + rB.get_position())/2;
		//calculate the value the motors should be set at based on its position relative to the target
		int val = pidCalculate((max==1)? drivePID: slowDrivePID, target, driveEnc)*max;
		val = (slew)? slewRateCalculate(val): val;
		//the left and right drive values should be different in order to correct getting turned as specified by the gyro value
		int rightVal = val;

		int leftVal = val;

		rF.move(rightVal);
    rB.move(rightVal);
	  lF.move(-leftVal);
    lB.move(-leftVal);

		//if the sensor value is within the desired range of the target
		if (fabs(driveEnc-target) < accuracy) {
			//if the sensor value is within the range for multiple iterations of the loop where each loop is approximately 20ms
			if (repsAtTarget > 15) {
				//break out of the while loop
				atTarget = true;
			}
			else {
				repsAtTarget++;
			}
		}
		else {
			repsAtTarget = 0;
		}
		pros::delay(15);
	}
	//zero the value of the encoder so that the next time this procedure is called, the encoder will be starting at zero again

  //LD.set_zero_position(driveEnc-target);
  //RD.set_zero_position(driveEnc-target);
	stop();
}

void driveTurn(int target, float angle, int accuracy, int time) {
	driveTarget(target, angle, accuracy, time, 1, true);
}

void driveTurn(int target, float angle, int accuracy, int time, float max, bool slew) {

	int startTime = pros::millis();
	bool atTarget = false;
	//int targetAngle = gyro.get_value()/10.0;
	int targetAngle = angle;
	int repsAtTarget = 0;
  float driveEnc = 0;
	//go into the loop that will repeat to update motor values and break when at target
	while (!atTarget && (pros::millis()-startTime) < time) {
		auxiliary();
		//pros::lcd::print(0, "Angle: %lf", gyro.get_value());
    driveEnc = (lB.get_position() + rB.get_position())/2;
		//calculate the value the motors should be set at based on its position relative to the target
		int val = pidCalculate((max==1)? drivePID: slowDrivePID, target, driveEnc)*max;
		val = (slew)? slewRateCalculate(val): val;
		//the left and right drive values should be different in order to correct getting turned as specified by the gyro value
		int rightVal = val;

		int leftVal = val;

		rF.move(rightVal);
    rB.move(rightVal);
	  lF.move(leftVal);
    lB.move(leftVal);

		//if the sensor value is within the desired range of the target
		if (fabs(driveEnc-target) < accuracy) {
			//if the sensor value is within the range for multiple iterations of the loop where each loop is approximately 20ms
			if (repsAtTarget > 15) {
				//break out of the while loop
				atTarget = true;
			}
			else {
				repsAtTarget++;
			}
		}
		else {
			repsAtTarget = 0;
		}
		pros::delay(15);
	}
	//zero the value of the encoder so that the next time this procedure is called, the encoder will be starting at zero again

  //LD.set_zero_position(driveEnc-target);
  //RD.set_zero_position(driveEnc-target);
	stop();
}

void gyroTurn(int target, int accuracy, int time) {
 gyroTurn(target, accuracy, time, false);
}

void gyroTurn(int target, int accuracy, int time, bool precise) {
	int startTime = pros::millis();
	bool gyroAtTarget = false;
	int repsAtTarget = 0;
	//go into the loop that will repeat to update motor values and break when at target
	while (!gyroAtTarget  && (pros::millis()-startTime) < time) {
		auxiliary();
		// calculate the desired motor value based on the sensor value relative to the target
		float drive = pidCalculate((precise)? gyroPrecisePID: gyroPID, target, gyro.get_value()/10.0)*((precise)? 0.75: 1);
		drive = ((fabs(gyro.get_value()/10.0-target)>180)? -1 : 1)*drive;
    lF.move(drive);
    lB.move(drive);
    rF.move(-drive);
    rB.move(-drive);
		//if the sensor value is within the desired range of the target
		if (fabs(gyro.get_value()/10.0-target) < accuracy) {
			//if the sensor value is within the range for multiple iterations of the loop where each loop is approximately 20ms
			if (repsAtTarget > 15) {
				//break out of the loop
				gyroAtTarget = true;
			}
			else {
				repsAtTarget++;
			}
		}
		else {
			repsAtTarget = 0;
		}
		pros::delay(15);
	}
	lB.set_zero_position(lB.get_position()-rB.get_position()); //zero the value of the drive encoder
  rB.set_zero_position(lB.get_position()-rB.get_position());
	stop();
}

void auxiliary() {
  lift.move(pidCalculate(aLiftPID, liftAim, lift.get_position()));

}

void driveArc(float a, float b, int direction, int target, float targetAngle, int accuracy, int time, bool slew) {
	driveArc(a,b,direction,target,targetAngle,accuracy, time, slew, target);
}

float
slewRateCalculate (float desiredRate) {
		float deltaTime = pros::millis()-lastSlewTime;
		float desiredAccel = (desiredRate -lastSlewRate)/deltaTime;
		float addedRate;
		float newRate;

		if (fabs(desiredAccel) < maxAccel || (desiredAccel<0 && desiredRate>0) || (desiredAccel>0 && desiredRate<0)) {
		    addedRate = desiredAccel*deltaTime;
		    newRate = addedRate+lastSlewRate;
		}
		else {
		    addedRate = ((desiredAccel>0)? 1: -1)*maxAccel*deltaTime;
        newRate = addedRate+lastSlewRate;
		}
	  lastSlewTime = lastSlewTime+deltaTime;
	  lastSlewRate = newRate;

		float returnVal = newRate;
		return returnVal;
}

void stop() {
	lB.set_zero_position(lB.get_position());
  rB.set_zero_position(rB.get_position());
  lB.move(0);
  lF.move(0);
  rB.move(0);
  rF.move(0);
}
