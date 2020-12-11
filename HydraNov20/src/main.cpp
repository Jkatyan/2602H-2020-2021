#include "main.h"

//Autonomous:
/*
* 0: Skills

* 1: Red Left
* 2: Red Left 2
* 3: Red Left 3

* 4: Blue Right
* 5: Blue Right 2
* 6: Blue Right 3

*/
const int auton = 0;

//Motor Ports
#define LEFT_FRONT 11
#define LEFT_BACK 12
#define RIGHT_FRONT 14
#define RIGHT_BACK 15
#define LEFT_INTAKE 13
#define RIGHT_INTAKE 18
#define MID_ROLLER 20
#define TOP_ROLLER 19

//Sensors
#define RIGHT_TRACKER_A 7
#define RIGHT_TRACKER_B 8
#define LEFT_TRACKER_A 5
#define LEFT_TRACKER_B 6
#define SIDE_TRACKER_A 3
#define SIDE_TRACKER_B 4
#define LINE_SENSOR 2
#define MID_LINE_SENSOR 1
#define CHASSIS_LINE_L 'g'
#define CHASSIS_LINE_R 'h'
#define IMU_PORT 1
#define OPTICAL_PORT 21

//CONSTANTS
#define DRIVEP 0.185
#define DRIVEI 0.01
#define DRIVED 0.04

#define TURNP 0.917
#define TURNI 0
#define TURND 0

#define RC 1 //Right Chassis Speed
#define LC 1 //Left Chassis Speed

#define TURN_CONSTANT 0.307
#define MAX_ACCEL 0.4

#define PURPLE 320
#define ORANGE 30
#define GREEN 100

//INITIALIZE
PID drivePID;
PID turnPID;

//CREATE DEVICES & SENSORS
Controller master (CONTROLLER_MASTER);

Motor LF (LEFT_FRONT, true);
Motor LB (LEFT_BACK, true);
Motor RF (RIGHT_FRONT);
Motor RB (RIGHT_BACK);
Motor leftIntake (LEFT_INTAKE);
Motor rightIntake (RIGHT_INTAKE, true);
Motor midRoller (MID_ROLLER);
Motor topRoller (TOP_ROLLER);

ADIAnalogIn lineSensor (LINE_SENSOR);
ADIAnalogIn midLineSensor (MID_LINE_SENSOR);

ADIAnalogIn chassisLineR (CHASSIS_LINE_R);
ADIAnalogIn chassisLineL (CHASSIS_LINE_L);

ADIEncoder sideEnc (SIDE_TRACKER_B, SIDE_TRACKER_A);
ADIEncoder rightEnc (RIGHT_TRACKER_B, RIGHT_TRACKER_A);
ADIEncoder leftEnc (LEFT_TRACKER_B, LEFT_TRACKER_A);

Optical optical(OPTICAL_PORT);
Imu imu (IMU_PORT);

//SLEW VARS
float lastSlewTime;
float maxAccel = MAX_ACCEL;
float lastSlewRate = 0;

//OPTICAL VARS
int color = -1;
int hue = 0;

//ROLLER VARS
int intakeMode;

//HELPER FUNCTIONS
void brake(){
	LF.move_velocity(0);
	LB.move_velocity(0);
	RF.move_velocity(0);
	RB.move_velocity(0);
}

void set_drive(float left, float right){
	if (left == 0 && right == 0){
		brake();
	}
	LF.move(left * LC);
	LB.move(left * LC);
	RF.move(right * RC);
	RB.move(right * RC);
}

void set_drive_brakes(motor_brake_mode_e_t mode){
	LF.set_brake_mode(mode);
	LB.set_brake_mode(mode);
	RF.set_brake_mode(mode);
	RB.set_brake_mode(mode);
}

void set_intake_brakes(motor_brake_mode_e_t mode){
	leftIntake.set_brake_mode(mode);
	rightIntake.set_brake_mode(mode);
	midRoller.set_brake_mode(mode);
	topRoller.set_brake_mode(mode);
}

void reset_drive(){
	RF.tare_position();
	RB.tare_position();
	LF.tare_position();
	LB.tare_position();
	sideEnc.reset();
}

bool isStopped(){
	if (!RF.is_stopped() || !RB.is_stopped() || !LF.is_stopped() || !LB.is_stopped()){
		return false;
	}
	return true;
}

bool crossedHalf(float distance, float target){
	if(distance < target){
		return (target/2 > distance) ? false : true;
	}
	else{
		return (target/2 < distance) ? false : true;
	}
	return false;
}

float slewRateCalculate (float desiredRate) {
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

//ROLLER FUNCTIONS
void setState(int state){
	intakeMode = state;
	/*
	* 0: Everything Stopped
	* 1: Auto Indexing, Intakes In
	* 2: Auto Indexing, Intakes Out
	* 3: Auto Indexing, Intakes Stopped
	* 4: Reverse Everything
	* 5: Score, Intakes In
	* 6: Score, Intakes Out
	* 7: Score, Intakes Stopped
	*/
}

void set_rollers(int intakes, int mid, int top){
	rightIntake.move(intakes);
	leftIntake.move(intakes);
	midRoller.move(mid);
	topRoller.move(top);
}

int intakeTask(){
	while(true){
		switch(intakeMode){
			case -1:
				continue;
			break;
			case 0:
				set_rollers(0, 0, 0);
			break;
			case 1:
				if(lineSensor.get_value() > 2950){
					set_rollers(127, 45, 60);
				}
				else{
					set_rollers(127, 80, -127);
				}
			break;
			case 2:
				if(lineSensor.get_value() > 2950){
					set_rollers(-127, 45, 60);
				}
				else{
					set_rollers(-127, 80, -127);
				}
			break;
			case 3:
				if(lineSensor.get_value() > 2950){
					set_rollers(0, 45, 60);
				}
				else{
					set_rollers(0, 80, -127);
				}
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
		}
		delay(10);
	}
}

void rollerOp(){
  intakeMode = -1;
  if(midLineSensor.get_value() >= 2950){
    if(master.get_digital(DIGITAL_L1)){
      if(master.get_digital(DIGITAL_L2)){
        midRoller.move(100);
        topRoller.move(127);
      }
      else if(lineSensor.get_value() >= 2950 && !(master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_L2))){
        midRoller.move(45);
        topRoller.move(60);
      }
      else{
        midRoller.move(80);
        topRoller.move(-127);
      }
    }
    else if(master.get_digital(DIGITAL_L2)){
      midRoller.move(-127);
      topRoller.move(-127);
    }
    else{
      midRoller.move_velocity(0);
      topRoller.move_velocity(0);
    }

		if(master.get_digital(DIGITAL_R1)){
      rightIntake.move(127);
      leftIntake.move(127);
    }
    else if(master.get_digital(DIGITAL_R2)){
      rightIntake.move(-127);
      leftIntake.move(-127);
    }
    else{
      rightIntake.move_velocity(0);
      leftIntake.move_velocity(0);
    }
  }
  else{
    if(master.get_digital(DIGITAL_L1)){
      if(master.get_digital(DIGITAL_L2)){
        midRoller.move(127);
        topRoller.move(127);
      }
      else if(lineSensor.get_value() >= 2950 && !(master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_L2))){
        midRoller.move(60);
        topRoller.move(60);
      }
      else{
        midRoller.move(80);
        topRoller.move(-127);
      }
    }
    else if(master.get_digital(DIGITAL_L2)){
      midRoller.move(-127);
      topRoller.move(-127);
    }
    else{
      midRoller.move_velocity(0);
      topRoller.move_velocity(0);
    }

    if(master.get_digital(DIGITAL_R1)){
      rightIntake.move(127);
      leftIntake.move(127);
    }
    else if(master.get_digital(DIGITAL_R2)){
      rightIntake.move(-127);
      leftIntake.move(-127);
    }
    else{
      rightIntake.move_velocity(0);
      leftIntake.move_velocity(0);
    }
  }
}

//DRIVE FUNCTIONS
void driveOp(){
	set_drive(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));
}

void fastDrive(int vel, int time){
	int startTime = pros::millis();
	while((pros::millis()-startTime) < time){
		set_drive(vel, vel);
	}
}

void drive(int target, int time, float speed){
  int atTarget = 0;
  float driveEnc = (((RF.get_position() + RB.get_position()) / 2) + ((LF.get_position() + LB.get_position()) / 2))/2;
	int direction = 1;
  float distance = 0.0;
  int startTime = pros::millis();

	if (driveEnc > target){
		direction = -1;
	}

  while ((atTarget != 1) && (pros::millis()-startTime) < time) {
	  driveEnc = (((RF.get_position() + RB.get_position()) / 2) + ((LF.get_position() + LB.get_position()) / 2))/2;
	  distance = target - driveEnc;

	  float val = pidCalculate(drivePID, target, driveEnc)*speed;
	  val = slewRateCalculate(val);
	  float rightVal = val;
	  float leftVal = val;

	  set_drive(leftVal, rightVal);
		if (direction == 1){
			if(driveEnc >= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
				 printf("Distance: %f\n", distance);
		 		 printf("Right Value : %f\n", ((RF.get_position() + RB.get_position()) / 2));
		 		 printf("Left Value  : %f\n", ((LF.get_position() + LB.get_position()) / 2));
		 		 printf("Back Encoder Value  : %d\n\n", sideEnc.get_value());
			}
		}
		else{
			if(driveEnc <= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
				 printf("Distance: %f\n", distance);
		 		 printf("Right Value : %f\n", ((RF.get_position() + RB.get_position()) / 2));
		 		 printf("Left Value  : %f\n", ((LF.get_position() + LB.get_position()) / 2));
		 		 printf("Back Encoder Value  : %d\n\n", sideEnc.get_value());
			}
		}
    pros::delay(20);
  }
}

void driveRaw(int target, int time, float speed){
  int atTarget = 0;
  float driveEnc = (((RF.get_position() + RB.get_position()) / 2) + ((LF.get_position() + LB.get_position()) / 2))/2;
	int direction = 1;
  float distance = 0.0;
  int startTime = pros::millis();

	if (driveEnc > target){
		direction = -1;
	}

  while ((atTarget != 1) && (pros::millis()-startTime) < time) {
	  driveEnc = (((RF.get_position() + RB.get_position()) / 2) + ((LF.get_position() + LB.get_position()) / 2))/2;
	  distance = target - driveEnc;

	  float val = pidCalculate(drivePID, target, driveEnc)*speed;
	  //val = slewRateCalculate(val);
	  float rightVal = val;
	  float leftVal = val;

	  set_drive(leftVal, rightVal);
		if (direction == 1){
			if(driveEnc >= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
				 printf("Distance: %f\n", distance);
		 		 printf("Right Value : %f\n", ((RF.get_position() + RB.get_position()) / 2));
		 		 printf("Left Value  : %f\n", ((LF.get_position() + LB.get_position()) / 2));
		 		 printf("Back Encoder Value  : %d\n\n", sideEnc.get_value());
			}
		}
		else{
			if(driveEnc <= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
				 printf("Distance: %f\n", distance);
		 		 printf("Right Value : %f\n", ((RF.get_position() + RB.get_position()) / 2));
		 		 printf("Left Value  : %f\n", ((LF.get_position() + LB.get_position()) / 2));
		 		 printf("Back Encoder Value  : %d\n\n", sideEnc.get_value());
			}
		}
    pros::delay(20);
  }
}

void driveLine(int target, int sensor, int time, float speed){ //Sesnor: 1R 2L
  int atTarget = 0;
  float driveEnc = (((RF.get_position() + RB.get_position()) / 2) + ((LF.get_position() + LB.get_position()) / 2))/2;
	int direction = 1;
	float sensorVal = 0;
  float distance = 0.0;
  int startTime = pros::millis();

	if (driveEnc > target){
		direction = -1;
	}

  while ((atTarget != 1) && (pros::millis()-startTime) < time) {
	  if (sensor == 1){
			sensorVal = chassisLineR.get_value();
		}
		else{
			sensorVal = chassisLineL.get_value();
		}

	  driveEnc = (((RF.get_position() + RB.get_position()) / 2) + ((LF.get_position() + LB.get_position()) / 2))/2;
	  distance = target - driveEnc;

	  float val = pidCalculate(drivePID, target, driveEnc)*speed;
	  val = slewRateCalculate(val);
	  float rightVal = val;
	  float leftVal = val;

	  set_drive(leftVal, rightVal);

		if (direction == 1){
			if(sensorVal < 300){
				atTarget = 1;
			}
			if(driveEnc >= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
				 printf("Distance: %f\n", distance);
		 		 printf("Right Value : %f\n", ((RF.get_position() + RB.get_position()) / 2));
		 		 printf("Left Value  : %f\n", ((LF.get_position() + LB.get_position()) / 2));
		 		 printf("Back Encoder Value  : %d\n\n", sideEnc.get_value());
			}
		}
		else{
			if(sensorVal < 300){
				atTarget = 1;
			}
			if(driveEnc <= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
				 printf("Distance: %f\n", distance);
		 		 printf("Right Value : %f\n", ((RF.get_position() + RB.get_position()) / 2));
		 		 printf("Left Value  : %f\n", ((LF.get_position() + LB.get_position()) / 2));
		 		 printf("Back Encoder Value  : %d\n\n", sideEnc.get_value());
			}
		}
    pros::delay(20);
  }
}

void driveSetState(int target, int state, int time, float speed){
	int atTarget = 0;
  float driveEnc = (((RF.get_position() + RB.get_position()) / 2) + ((LF.get_position() + LB.get_position()) / 2))/2;
	int direction = 1;
  float distance = 0.0;
  int startTime = pros::millis();

	if (driveEnc > target){
		direction = -1;
	}

  while ((atTarget != 1) && (pros::millis()-startTime) < time) {
	  driveEnc = (((RF.get_position() + RB.get_position()) / 2) + ((LF.get_position() + LB.get_position()) / 2))/2;
	  distance = target - driveEnc;

	  float val = pidCalculate(drivePID, target, driveEnc)*speed;
	  val = slewRateCalculate(val);
	  float rightVal = val;
	  float leftVal = val;

	  set_drive(leftVal, rightVal);

		if (crossedHalf(distance, target)){
			setState(state);
		}

		if (direction == 1){
			if(driveEnc >= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
				 printf("Distance: %f\n", distance);
				 printf("Right Value : %f\n", ((RF.get_position() + RB.get_position()) / 2));
		 		 printf("Left Value  : %f\n", ((LF.get_position() + LB.get_position()) / 2));
		 		 printf("Back Encoder Value  : %d\n\n", sideEnc.get_value());
			}
		}
		else{
			if(driveEnc <= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
				 printf("Distance: %f\n", distance);
				 printf("Right Value : %f\n", ((RF.get_position() + RB.get_position()) / 2));
		 		 printf("Left Value  : %f\n", ((LF.get_position() + LB.get_position()) / 2));
		 		 printf("Back Encoder Value  : %d\n\n", sideEnc.get_value());
			}
		}
    pros::delay(20);
  }
}

void rotate(float target, int time, float speed){
	int atTarget = 0;
  float driveEnc = 0.0;
  float distance = 0.0;
  int startTime = pros::millis();

	sideEnc.reset();
	while ((atTarget != 1) && (pros::millis()-startTime) < time) {
	  driveEnc = imu.get_yaw();
	  distance = target - driveEnc;

	  float val = pidCalculate(turnPID, target, driveEnc)*speed;
		//val = slewRateCalculate(val);
	  float rightVal = val;
	  float leftVal = val;

	  set_drive(leftVal, -rightVal);
	  if(driveEnc == target || ((pros::millis()-startTime) == time)){
	     atTarget = 1;
		}
    pros::delay(20);
  }
	printf("Angle Value : %lf\n\n", imu.get_yaw());
	reset_drive();
}

void rotateHeading(float target, int time, float speed){
	int atTarget = 0;
  float driveEnc = 0.0;
  float distance = 0.0;
  int startTime = pros::millis();

	sideEnc.reset();
	while ((atTarget != 1) && (pros::millis()-startTime) < time) {
	  driveEnc = imu.get_heading();
	  distance = target - driveEnc;

	  float val = pidCalculate(turnPID, target, driveEnc)*speed;
		//val = slewRateCalculate(val);
	  float rightVal = val;
	  float leftVal = val;

	  set_drive(leftVal, -rightVal);
	  if(driveEnc == target || ((pros::millis()-startTime) == time)){
	     atTarget = 1;
		}
    pros::delay(20);
  }
	printf("Angle Value : %lf\n\n", imu.get_heading());
	reset_drive();
}

//INITIALIZE TASKS
void initRollers(){
	set_intake_brakes(MOTOR_BRAKE_COAST);
	optical.set_led_pwm(100);
	Task intake_task(intakeTask);
}

void initDrive(bool useImu){
	if(useImu){
		imu.reset();
		delay(2500);
	}
	set_drive_brakes(MOTOR_BRAKE_COAST);
}

void initialize() {
	initRollers();
	initDrive(true);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	//Initialize PID Values
	drivePID = pidInit (DRIVEP, DRIVEI, DRIVED, 0, 100.0, 5, 15);
	turnPID = pidInit (TURNP, TURNI, TURND, 0, 10.0, 99999, 99999);

	/* REFERENCE:
	drive(int target, int time, float speed);
	driveSetState(int target, int state, int num, int den, int time, float speed);
	rotate(int target, int time, float speed);
	* 0: Everything Stopped
	* 1: Auto Indexing, Intakes In
	* 2: Auto Indexing, Intakes Out
	* 3: Auto Indexing, Intakes Stopped
	* 4: Reverse Everything
	* 5: Score, Intakes In
	* 6: Score, Intakes Out
	* 7: Score, Intakes Stopped
	*/

	switch(auton){
		case 0: //Skills

			reset_drive();
			set_intake_brakes(MOTOR_BRAKE_COAST);

			//Deploy Score Goal 1
			setState(5);
			delay(200);
			setState(0);
			delay(1000);

			set_intake_brakes(MOTOR_BRAKE_HOLD);

			//Intake Ball One
			setState(1);
			drive(1925, 5000, 1);

			//Turn to Goal 2
			rotate(-89, 2000, 1);

			//Drive to Goal 2
			setState(0);
			drive(1115, 1500, 1);

			//Score Goal 2
			setState(7);
			delay(600);
			setState(0);

			//Back out of Goal
			drive(0, 2000, 1);

			//Turn to Ball
			rotate(93, 2000, 1);

			//Intake Ball 2
			setState(1);
			drive(2390, 3000, 1);

			//Turn to Goal
			rotate(-33, 2000, 1);
			setState(1);

			//Drive to Goal 3 and Intake Ball 3
			drive(2000, 1700, 1);

			//Score Goal 3
			setState(6);
			delay(400);
			setState(0);

			//Back out of Goal
			driveSetState(0, 4, 2000, 1);
			setState(1);

			//Turn to Goal 4
			rotate(38, 2000, 1);

			//Drive to Goal 4
			drive(3450, 3000, 1);

			//Score Goal 4
			setState(6);
			delay(500);
			setState(0);

			//Back out of Goal
			setState(4);
			drive(3050, 4000, 1);
			setState(0);

			//Turn to Ball 4 and Intake Ball 4
			rotateHeading(185, 3000, 1);
			setState(1);
			drive(2875, 5000, 1);

			//Turn to Goal 4
			rotateHeading(54, 2000, 0.7);

			//Drive to Goal 5
			drive(1475, 1000, 1);

			//Score Goal 5
			setState(6);
			delay(800);
			setState(0);

			//Back out of Goal
			drive(1075, 2000, 1);

			//Turn to and intake ball
			rotateHeading(167, 3000, 0.7);
			setState(1);
			drive(2875, 5000, 1);

			//move to goal 6
			rotateHeading(144, 3000, 0.7);
			setState(1);
			drive(950, 2000, 1);

			//Score Goal 5
			setState(6);
			delay(800);
			setState(0);

		break;
		case 1: //Red Left

			reset_drive();
			set_intake_brakes(MOTOR_BRAKE_COAST);

			//Deploy Score Goal 1
			setState(5);
			delay(200);
			setState(0);
			delay(500);

			set_intake_brakes(MOTOR_BRAKE_HOLD);

			//Intake Ball One
			setState(1);
			drive(2000, 5000, 1);

			//Turn to Goal 2
			rotate(-89, 2000, 1);

			//Drive to Goal 2
			drive(1115, 1500, 1);
			setState(0);

			//Score Goal 2
			setState(5);
			delay(900);
			setState(0);
			delay(100);

			//Back out of Goal
			drive(0, 2000, 1);

		break;
		case 2: //Red Left 2

			reset_drive();
			set_intake_brakes(MOTOR_BRAKE_COAST);

			//Deploy Score Goal 1
			setState(5);
			delay(200);
			setState(0);
			delay(500);

			set_intake_brakes(MOTOR_BRAKE_HOLD);

			//Intake Ball One
			setState(1);
			drive(2000, 5000, 1);

			//Turn to Goal 2
			rotate(-89, 2000, 1);

			//Drive to Goal 2
			drive(1115, 1500, 1);
			setState(0);

			//Score Goal 2
			setState(7);
			delay(600);
			setState(0);

			//Back out of Goal
			drive(0, 2000, 1);

		break;
		case 3: //Red Left 3

		break;
		case 4: //Blue Right

			reset_drive();
			set_intake_brakes(MOTOR_BRAKE_COAST);

			//Deploy Score Goal 1
			setState(5);
			delay(200);
			setState(0);
			delay(500);

			set_intake_brakes(MOTOR_BRAKE_HOLD);

			//Intake Ball One
			setState(1);
			drive(2020, 5000, 1);

			//Turn to Goal 2
			rotate(89, 2000, 1);

			//Drive to Goal 2
			drive(1115, 1500, 1);
			setState(0);

			//Score Goal 2
			setState(5);
			delay(900);
			setState(0);
			delay(100);

			//Back out of Goal
			drive(0, 2000, 1);
		break;
		case 5: //Blue Right 2

		reset_drive();
		set_intake_brakes(MOTOR_BRAKE_COAST);

		//Deploy Score Goal 1
		setState(5);
		delay(200);
		setState(0);
		delay(500);

		set_intake_brakes(MOTOR_BRAKE_HOLD);

		//Intake Ball One
		setState(1);
		drive(2020, 5000, 1);

		//Turn to Goal 2
		rotate(87, 2000, 1);

		//Drive to Goal 2
		drive(1115, 1500, 1);
		setState(0);

		//Score Goal 2
		setState(7);
		delay(600);
		setState(0);

		//Back out of Goal
		drive(0, 2000, 1);
		break;
		case 6: //Blue Right 3
		break;

	}
}

void opcontrol() {
	intakeMode = -1;
	set_intake_brakes(MOTOR_BRAKE_HOLD);
	while (true) {
		driveOp();
		rollerOp();
		printf("Right Value : %f\n", ((RF.get_position() + RB.get_position()) / 2));
		printf("Left Value  : %f\n", ((LF.get_position() + LB.get_position()) / 2));
		printf("Back Encoder Value  : %d\n\n", sideEnc.get_value());
		delay(20);
	}
}
