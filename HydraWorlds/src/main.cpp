#include "main.h"

//Thanks to 2011D for the help!

//Constants
#define LC 1 //Left Side Coefficient
#define RC 1 //Right Side Coefficient

	//Drive PID
	#define DRIVE_KP 400
	#define DRIVE_KI 0.01
	#define DRIVE_KD 1200.001
	#define TURN_D 100.0

	//Turn PID
	#define TURN_KP 125.0
	#define TURN_KI 1.3
	#define TURN_KD 1800.0

//Util
void disabled() {}
void competition_initialize() {}

void resetAll(){
	//Reset Sensors
  rEncoder.reset();
  sEncoder.reset();

	//Reset Motors
	leftBack.tare_position();
	leftFront.tare_position();
	rightBack.tare_position();
	rightFront.tare_position();
	leftIntake.tare_position();
	rightIntake.tare_position();
	midRoller.tare_position();
	topRoller.tare_position();
}

void resetDrive(){
	leftBack.tare_position();
	leftFront.tare_position();
	rightBack.tare_position();
	rightFront.tare_position();
}

void brake(){
	leftBack.move_velocity(0);
	leftFront.move_velocity(0);
	rightFront.move_velocity(0);
	rightBack.move_velocity(0);
}

void setDriveBrakes(pros::motor_brake_mode_e_t mode){
	leftBack.set_brake_mode(mode);
	leftFront.set_brake_mode(mode);
	rightBack.set_brake_mode(mode);
	rightFront.set_brake_mode(mode);
}

void setIntakeBrakes(pros::motor_brake_mode_e_t mode){
	leftIntake.set_brake_mode(mode);
	rightIntake.set_brake_mode(mode);
	midRoller.set_brake_mode(mode);
	topRoller.set_brake_mode(mode);
}

void setDrive(float left, float right){
	if (left == 0 && right == 0){
		brake();
	}
	leftBack.move(left * LC);
	leftFront.move(left * LC);
	rightBack.move(right * RC);
	rightFront.move(right * RC);
}

void setDrive_MV(float left, float right){
	if (left == 0 && right == 0){
		brake();
	}
	leftBack.move_voltage(left * LC);
	leftFront.move_voltage(left * LC);
	rightBack.move_voltage(right * RC);
	rightFront.move_voltage(right * RC);
}

void setDriveVelocity(float left, float right){
	if (left == 0 && right == 0){
		brake();
	}
	leftBack.move_velocity(left * LC);
	leftFront.move_velocity(left * LC);
	rightBack.move_velocity(right * RC);
	rightFront.move_velocity(right * RC);
}

float lVelocity = 0, maxChange = 600.0;
float slew(float velocity){
  float vOut;
  float deriv = velocity - lVelocity;
  if(deriv > maxChange){
    vOut = lVelocity + maxChange;
  }
  else{
    vOut = velocity;
  }
  lVelocity = vOut;
  return vOut;
}

//Roller Functions
int intakeMode = 0;
void setState(int state){
	intakeMode = state;
}

void set_intakes(int intakes){
	if(intakes == 0){
		rightIntake.move_velocity(0);
		leftIntake.move_velocity(0);
	}
	else{
		rightIntake.move(intakes);
		leftIntake.move(intakes);
	}
}

void set_conveyor(int mid, int top){
	if(mid == 0){
		midRoller.move_velocity(0);
	}
	else{
		midRoller.move(mid);
	}
	if(top == 0){
		topRoller.move_velocity(0);
	}
	else{
		topRoller.move(top);
	}
}

void set_rollers(int intakes, int mid, int top){
	set_intakes(intakes);
	set_conveyor(mid, top);
}

int intakeTask(){
	while(true){
		switch(intakeMode){
			case 0:
				set_rollers(0, 0, 0);
			break;
			case 1:
				set_rollers(127, 127, -127);
			break;
			case 2:
				set_rollers(-127, 127, -127);
			break;
			case 3:
				set_rollers(0, 127, -127);
			break;
			case 4:
				set_rollers(-127, -127, -127);
			break;
			case 5:
				set_rollers(127, 127, 127);
			break;
			case 6:
				set_rollers(-127, 127, 127);
			break;
			case 7:
				set_rollers(0, 127, 127);
			break;
			case 8:
				set_rollers(-127, 0, 0);
			break;
			case -1:
				if(master.get_digital(DIGITAL_L1)){
					if(master.get_digital(DIGITAL_L2)){ //Scoring
						set_conveyor(127, 127);
					}
					else{ //Indexing
						set_conveyor(127, -60);
					}
				}
				else if(master.get_digital(DIGITAL_L2)){ //Reverse
					set_conveyor(-127, -127);
				}
				else{ //Stopped
					set_conveyor(0, 0);
				}

				if(master.get_digital(DIGITAL_R1)){
					set_intakes(127);
				}
				else if(master.get_digital(DIGITAL_R2)){
					set_intakes(-127);
				}
				else{
					set_intakes(0);
				}
			break;
		}
		pros::delay(20);
	}
}

//Drive Functions
void drive(float xTarget, float yTarget, float pctCutoff, float maxSpeed, bool reversed, std::string errorType, float angles, float speedX, float timeOut){
	lVelocity = 0;

  float errorX, errorY, errorD, errorR,
  kP = DRIVE_KP,
  kI = DRIVE_KI,
  kD = DRIVE_KD,
  proportion, integral, integralZone, intergralLimit = 6500, derivative, lastErrorD, driveVelocity, finalVL, finalVR, turnVelocity, turnD = TURN_D, dTurn,iR, lastErrorR, turnenable, sTime = pros::millis();

	bool enabled = true, temp = true;
  int direction;

  while(enabled){
    errorX = xTarget - x;
    if(fabs(errorX) < 0.01){
      errorX = 0;
    }

    errorY = yTarget - y;
    if(fabs(errorY) < 0.01){
      errorY= 0;
    }

    errorD = sqrt(errorX * errorX + errorY * errorY);
    while(temp){
      integralZone = errorD * 0.25;
      temp = false;
    }

    if(reversed){
      direction = -1;
    }
    else{
      direction = 1;
    }
    proportion = errorD;

    if(fabs(errorD) < integralZone && errorD != 0){
      integral = integral + errorD;
    }
    else{
      integral = 0;
    }
    if(integral > intergralLimit){
      integral = intergralLimit;
    }

    derivative = errorD - lastErrorD;
    lastErrorD = errorD;

    driveVelocity = (kP*proportion + kI*integral + kD*derivative);
    if(driveVelocity > maxSpeed){
      driveVelocity = maxSpeed;
    }
    if(driveVelocity < -maxSpeed){
      driveVelocity = -maxSpeed;
    }
    driveVelocity *= direction;
    driveVelocity *= speedX;
    errorR = angles -  angle/M_PI * 180.0;

    turnVelocity = errorR * turnD;
    finalVL = slew(driveVelocity);
    finalVL *= speedX;

    if(errorType == "y"){
      if(fabs(errorY) < pctCutoff){
        enabled = false;
      }
    }
    else if(errorType == "x"){
      if(fabs(errorX) < pctCutoff){
        enabled = false;
      }
    }
    else if(errorType == "d"){
      if(fabs(errorD) < pctCutoff){
        enabled = false;
      }
    }

    if(sTime + timeOut < pros::millis()){
      enabled = false;
    }

		setDrive_MV(finalVL+turnVelocity, finalVL-turnVelocity);
    }
}

void driveSetState(float xTarget, float yTarget, float pctCutoff, float maxSpeed, bool reversed, std::string errorType, float angles, float speedX, float timeOut, int state, int intakeDelay){
	lVelocity = 0;

  float errorX, errorY, errorD, errorR,
  kP = DRIVE_KP,
  kI = DRIVE_KI,
  kD = DRIVE_KD,
  proportion, integral, integralZone, intergralLimit = 6500, derivative, lastErrorD, driveVelocity, finalVL, finalVR, turnVelocity, turnD = TURN_D, dTurn,iR, lastErrorR, turnenable, sTime = pros::millis();

	bool enabled = true, temp = true;
  int direction;

  while(enabled){
    errorX = xTarget - x;
    if(fabs(errorX) < 0.01){
      errorX = 0;
    }

    errorY = yTarget - y;
    if(fabs(errorY) < 0.01){
      errorY= 0;
    }

    errorD = sqrt(errorX * errorX + errorY * errorY);
    while(temp){
      integralZone = errorD * 0.25;
      temp = false;
    }

    if(reversed){
      direction = -1;
    }
    else{
      direction = 1;
    }
    proportion = errorD;

    if(fabs(errorD) < integralZone && errorD != 0){
      integral = integral + errorD;
    }
    else{
      integral = 0;
    }
    if(integral > intergralLimit){
      integral = intergralLimit;
    }

    derivative = errorD - lastErrorD;
    lastErrorD = errorD;

    driveVelocity = (kP*proportion + kI*integral + kD*derivative);
    if(driveVelocity > maxSpeed){
      driveVelocity = maxSpeed;
    }
    if(driveVelocity < -maxSpeed){
      driveVelocity = -maxSpeed;
    }
    driveVelocity *= direction;
    driveVelocity *= speedX;
    errorR = angles -  angle/M_PI * 180.0;

    turnVelocity = errorR * turnD;
    finalVL = slew(driveVelocity);
    finalVL *= speedX;

    if(errorType == "y"){
      if(fabs(errorY) < pctCutoff){
        enabled = false;
      }
    }
    else if(errorType == "x"){
      if(fabs(errorX) < pctCutoff){
        enabled = false;
      }
    }
    else if(errorType == "d"){
      if(fabs(errorD) < pctCutoff){
        enabled = false;
      }
    }

    if(sTime + timeOut < pros::millis()){
      enabled = false;
    }

		if((pros::millis() - sTime) > intakeDelay){
			setState(state);
		}

		setDrive_MV(finalVL+turnVelocity, finalVL-turnVelocity);
  }
}

void turn(float desiredAngle, float maxSpeed, float speedX, float errorBreak, float timeOut){
  float errorR, errorRa,
  kP_t = TURN_KP,
  kI_t = TURN_KI,
  kD_t = TURN_KD,
  proportion_t, integral_t, integralZone_t = 10, intergralLimit_t = 10000, integralMin = 1000, derivative_t, lastErrorR, turnVelocity, sTime = pros::millis();

	int direction_t;
  bool enabled = true, start = true;

  while(enabled){
    //Arc length and angle error calculations
    errorR = (desiredAngle * M_PI / 180.0) - angle;
    errorRa = (fabs(errorR))/M_PI * 180.0;
    if(errorR < 0){
      direction_t = -1;
    }
    else{
      direction_t = 1;
    }

    //Turn PID
    proportion_t = errorRa;

    if(errorRa < integralZone_t && errorRa != 0){
      integral_t += (errorRa+ lastErrorR)/2;
    }
    else{
      integral_t = 0;
    }
    if(integral_t > intergralLimit_t){
      integral_t = intergralLimit_t;
    }
    if(integral_t < integralMin){
      integral_t = integralMin;
    }
    derivative_t = errorRa - lastErrorR;
    lastErrorR = errorRa;

    turnVelocity = (kP_t*proportion_t + kI_t*integral_t + kD_t*derivative_t)
                    * direction_t * speedX;

    if(turnVelocity > maxSpeed){
      turnVelocity =  maxSpeed;
    }
    else if(turnVelocity < -maxSpeed){
      turnVelocity = -maxSpeed;
    }

    if(errorRa < errorBreak || sTime + timeOut < pros::millis()){
      turnVelocity = 0;
      enabled = false;
    }

		setDrive_MV(turnVelocity, -turnVelocity);

    pros::delay(10);
  }
}

//Debug
void debugTask(){
	while(true){
		pros::lcd::print(2, "(%f, %f)", x, y);
		pros::lcd::print(3, "Angle: %lf", (angle/M_PI * 180.0));
		pros::delay(20);
	}
}

void initialize() {
	//LCD
	pros::lcd::initialize();
	pros::delay(100);
	pros::lcd::set_text(1, "Hopkinetics Hydra:");
	pros::lcd::set_text(2, "Initializing!");

	//Sensors
	imuA.reset();
	imuB.reset();
	pros::delay(2500);

	sEncoder.reset();
	rEncoder.reset();

	//Tasks
	tracking = new Task(track, TASK_PRIORITY_MAX,TASK_STACK_DEPTH_DEFAULT, "tracking");
	pros::Task intake_task(intakeTask);
	pros::Task debug_task(debugTask);
  pros::delay(200);

	//Brake Modes
	setDriveBrakes(MOTOR_BRAKE_COAST);
	setIntakeBrakes(MOTOR_BRAKE_HOLD);
}

void autonomous() {

	setState(5);
	pros::delay(1000);
	setState(1);
  drive(-45.0, 27.0, 1.0, 12000, false, "y", -70.0, 1.0, 2000);
	drive(-31.0, 18.0, 1.0, 12000, true, "x", -65.0, 1.0, 1000);
	turn(-130, 12000, 1.0, 0.4, 1000);
	drive(-46, 7.0, 1.0, 12000, false, "x", -130.0, 1.0, 2000);
	setState(5);
	pros::delay(650);
	setState(1);
	driveSetState(-22, 36, 1.0, 12000, true, "y", -145, 1.0, 2000, 4, 500);
	turn(-270, 12000, 1.0, 0.4, 2000);
	setState(1);
	drive(10, 36, 1.0, 12000, false, "x", -270, 1.0, 2000);
	/*turn(-180, 12000, 1.0, 0.4, 2000);
	drive(10, 8, 1.0, 12000, false, "y", -180, 1.0, 2000);
	setState(5);
	pros::delay(300);
	setState(1);
	driveSetState(10, 28, 1.0, 12000, true, "y", -180, 1.0, 2000, 4, 700);*/
	turn(-225, 12000, 1.0, 0.4, 2000);
	drive(66, 2, 1.0, 12000, false, "x", -225, 1.0, 2000);
	setState(5);
	pros::delay(300);
	setState(1);
	driveSetState(38, 38, 1.0, 12000, true, "y", -225, 1.0, 2000, 4, 700);
	setState(1);
	turn(-360, 12000, 1.0, 0.4, 2000);
	drive(39, 55, 1.0, 12000, false, "y", -360, 1.0, 2000);

}

void opcontrol() {
	//Intake setting
	intakeMode = -1;

	//Driver Control
	while (true) {
		setDrive(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));
		pros::delay(20);
	}
}
