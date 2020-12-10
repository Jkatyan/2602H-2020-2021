#include "main.h"

PID rightPID (DRIVE_RIGHT_KP, DRIVE_RIGHT_KD);
PID leftPID (DRIVE_LEFT_KP, DRIVE_LEFT_KD);
PID turnPID (TURN_KP, TURN_KD);

pros::Motor LF (LEFT_FRONT, MOTOR_GEARSET_18, LF_REVERSED, MOTOR_ENCODER_COUNTS);
pros::Motor LB (LEFT_BACK, MOTOR_GEARSET_18, LB_REVERSED, MOTOR_ENCODER_COUNTS);
pros::Motor RF (RIGHT_FRONT, MOTOR_GEARSET_18, RF_REVERSED, MOTOR_ENCODER_COUNTS);
pros::Motor RB (RIGHT_BACK, MOTOR_GEARSET_18, RB_REVERSED, MOTOR_ENCODER_COUNTS);

pros::ADIEncoder sideEnc (BACK_TRACKER_A, BACK_TRACKER_B);
pros::ADIEncoder rightEnc (RIGHT_TRACKER_A, RIGHT_TRACKER_B);
pros::ADIEncoder leftEnc (LEFT_TRACKER_A, LEFT_TRACKER_B);

pros::Controller master(CONTROLLER_MASTER);
pros::Imu imu (IMU_PORT);

int left_pos(){
  return leftEnc.get_value();
}

int right_pos(){
  return rightEnc.get_value();
}

void left_drive(int power) {
	LF.move(power * LC);
  LB.move(power * LC);
}

void right_drive(int power) {
	RF.move(power * RC);
  RB.move(power * RC);
}

void set_drive(int left, int right){
	if(left == 0 && right != 0){
		LF.move_velocity(0);
	  LB.move_velocity(0);
		right_drive(right);
	}
	else if(left != 0 && right == 0){
		RF.move_velocity(0);
	  RB.move_velocity(0);
		left_drive(left);
	}
	else{
  left_drive(left);
  right_drive(right);
	}
}

void brake(){
  LF.move_velocity(0);
  LB.move_velocity(0);
  RF.move_velocity(0);
  RB.move_velocity(0);
}

void setDriveBrakes (pros::motor_brake_mode_e_t mode){
  LF.set_brake_mode(mode);
  LB.set_brake_mode(mode);
  RF.set_brake_mode(mode);
  RB.set_brake_mode(mode);
}

void reset(){
  LF.tare_position();
  LB.tare_position();
  RF.tare_position();
  RB.tare_position();
  rightPID.reset();
  leftPID.reset();
  turnPID.reset();
}

bool isStopped(){
  if(!RF.is_stopped() || !RB.is_stopped() || !LF.is_stopped() || !LB.is_stopped()){
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

void driveLeft(double target, int time, float speed){

	double lastDistance = 0;
	double distance;
	int reps = 0;
  bool atTarget = false;
	int startTime = pros::millis();

  while(!atTarget && (pros::millis()-startTime) < time){

    distance = left_pos();

    double leftPower = leftPID.calculate(target, distance);

    set_drive(leftPower * speed, 0);

		if(lastDistance == distance){
			reps++;
		}

    if(distance == target || reps >= 5){
      atTarget = true;
    }
		else if(distance > target){
			set_drive(-10, 0);
			pros::delay(50);
			atTarget = true;
		}

		lastDistance = distance;
    pros::delay(10);
  }
	master.print(0, 0, "Distance: %lf\n\n", distance);
  brake();
}

void driveRight(double target, int time, float speed){

	double lastDistance = 0;
	double distance;
	int reps = 0;
  bool atTarget = false;
	int startTime = pros::millis();

  while(!atTarget && (pros::millis()-startTime) < time){

    distance = right_pos();

    double leftPower = rightPID.calculate(target, distance);

    set_drive(leftPower * speed, 0);

		if(lastDistance == distance){
			reps++;
		}

    if(distance == target || reps >= 5){
      atTarget = true;
    }
		else if(distance > target){
			set_drive(0, -10);
			pros::delay(50);
			atTarget = true;
		}

		lastDistance = distance;
    pros::delay(10);
  }
	master.print(0, 0, "Distance: %lf\n\n", distance);
  brake();
}

void drive(double target, int time, float speed){

	double lastDistance = 0;
	double currAngle = imu.get_heading();
	double distance;
	int reps = 0;
  bool atTarget = false;
	int startTime = pros::millis();

  while(!atTarget && (pros::millis()-startTime) < time){

    distance = (left_pos() + right_pos()) / 2.0;

    double rightPower = rightPID.calculate(target, distance);
    double leftPower = leftPID.calculate(target, distance);

		double angleError = currAngle - imu.get_heading();

    set_drive(leftPower * speed - (angleError * ANGLE_KP), rightPower * speed + (angleError * ANGLE_KP));

		if(lastDistance == distance){
			reps++;
		}

    if(distance == target && reps >= 5){
      atTarget = true;
    }
    else if(distance < target && reps < 5 && reps >= 3){
      while(distance < target){
        set_drive(20, 20);
        pros::delay(10);
      }
      set_drive(-10, -10);
			pros::delay(50);
			atTarget = true;
    }
		else if(distance > target){
			set_drive(-10, -10);
			pros::delay(50);
			atTarget = true;
		}

		lastDistance = distance;
    pros::delay(10);
  }
	master.print(0, 0, "R Distance: %lf\n\n", right_pos());
	master.print(1, 0, "L Distance: %lf\n\n", left_pos());
  brake();
}

void driveSetState(double target, int state, int time, float speed){

	double lastDistance = 0;
	double currAngle = imu.get_heading();
	double distance;
	int reps = 0;
  bool atTarget = false;
	int startTime = pros::millis();

  while(!atTarget && (pros::millis()-startTime) < time){

    distance = (left_pos() + right_pos()) / 2.0;

    double rightPower = rightPID.calculate(target, distance);
    double leftPower = leftPID.calculate(target, distance);

		double angleError = currAngle - imu.get_heading();

    set_drive(leftPower * speed - (angleError * ANGLE_KP), rightPower * speed + (angleError * ANGLE_KP));

		if(lastDistance == distance){
			reps++;
		}

		if(crossedHalf(distance, target)){
			setState(state);
		}

    if(distance == target || reps >= 5){
      atTarget = true;
    }
		else if(distance > target){
			set_drive(-10, -10);
			pros::delay(50);
			atTarget = true;
		}

		lastDistance = distance;
    pros::delay(10);
  }
	master.print(0, 0, "Distance: %lf\n\n", distance);
  brake();
}

void fastDrive(double target, int speed){
	if (target < 0){
		speed = -speed;
	}
	reset();
	set_drive(speed, speed);

	if (target > 0){
		while (((left_pos() + right_pos()) / 2.0) < target){
			pros::delay(10);
		}
	}
	else{
		while (((left_pos() + right_pos()) / 2.0) > target){
			pros::delay(10);
		}
	}
}

void rotateRaw(double target, int time, float speed){

	double lastAngle = imu.get_heading();
	int reps = 0;
	double angle;
  bool atTarget = 0;
  int startTime = pros::millis();

  while (!atTarget && (pros::millis()-startTime) < time) {
		angle = imu.get_heading();
		double anglePower = turnPID.calculate(target, angle) * speed;

    int distance = target - angle;

	  set_drive(anglePower, -anglePower);

		if(angle == lastAngle){
			reps++;
		}

	  if(angle == target || reps >= 5){
	     atTarget = 1;
	  }

		lastAngle = angle;
    pros::delay(10);
  }
	printf("Angle: %lf\n\n", angle);
  brake();
	reset();
}

void rotateImu(double degrees, int time, float speed){

	double lastAngle = imu.get_heading();
	double target = imu.get_heading() + degrees;
	int reps = 0;
	double angle;
  bool atTarget = 0;
  int startTime = pros::millis();

  while (!atTarget && (pros::millis()-startTime) < time) {
		angle = imu.get_heading();
		double anglePower = turnPID.calculate(target, angle) * speed;

    int distance = target - angle;

	  set_drive(anglePower, -anglePower);

		if(angle == lastAngle){
			reps++;
		}

	  if(angle == target || reps >= 5){
	     atTarget = 1;
	  }

		lastAngle = angle;
    pros::delay(10);
  }
	printf("Angle: %lf\n\n", angle);
  brake();
	reset();
}

void rotateTracker(double target, int time, float speed){

	sideEnc.reset();

	int reps = 0;
	double lastAngle = 0;
	double angle;
  bool atTarget = 0;
  int startTime = pros::millis();

  while (!atTarget && (pros::millis()-startTime) < time) {
		angle = sideEnc.get_value();
		double anglePower = turnPID.calculate(target, angle * TURN_CONSTANT) * speed;

    int distance = target - (angle * TURN_CONSTANT);

	  set_drive(anglePower, -anglePower);

		if(angle == lastAngle){
			reps++;
		}

	  if(angle == target || reps >= 5){
	     atTarget = 1;
	  }

		lastAngle = angle;
    pros::delay(10);
  }
	printf("Angle: %lf\n\n", angle);
  brake();
	reset();
}


//* okapi drive
okapi::Controller controller;

//Okapi Controllers
MotorGroup leftMotors = {LEFT_FRONT, LEFT_BACK};
MotorGroup rightMotors = {RIGHT_FRONT, RIGHT_BACK};

std::shared_ptr<ChassisController> chassis = ChassisControllerBuilder()
    .withMotors(leftMotors, rightMotors)
    .build();

void okapi_waitUntilSettled(){
  chassis->waitUntilSettled();
}

void okapi_drive(QLength dist, int rpm){
  chassis->setMaxVelocity(rpm);
  chassis->moveDistance(dist);
}

void okapi_driveAsync(QLength dist, int rpm){
  chassis->setMaxVelocity(rpm);
  chassis->moveDistanceAsync(dist);
}

void okapi_turn(QAngle deg, int rpm){
  chassis->setMaxVelocity(rpm);
  chassis->turnAngle(deg);
}

void initDrive(){
	imu.reset();
  pros::delay(2000);
  setDriveBrakes(MOTOR_BRAKE_COAST);
}

void tankDrive(){
  chassis->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY) * LC,
                            controller.getAnalog(ControllerAnalog::rightY) * RC);
}

void arcadeDrive(){
  chassis->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY) * LC,
                              controller.getAnalog(ControllerAnalog::leftX) * RC);
}
