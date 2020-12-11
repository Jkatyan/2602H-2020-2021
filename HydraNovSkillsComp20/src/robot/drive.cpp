#include "main.h"

PID rightPID (DRIVE_RIGHT_KP, DRIVE_RIGHT_KD);
PID leftPID (DRIVE_LEFT_KP, DRIVE_LEFT_KD);
PID turnPID (TURN_KP, TURN_KD);

pros::Motor LF (LEFT_FRONT, MOTOR_GEARSET_18, true, MOTOR_ENCODER_COUNTS);
pros::Motor LB (LEFT_BACK, MOTOR_GEARSET_18, true, MOTOR_ENCODER_COUNTS);
pros::Motor RF (RIGHT_FRONT, MOTOR_GEARSET_18, MOTOR_ENCODER_COUNTS);
pros::Motor RB (RIGHT_BACK, MOTOR_GEARSET_18, MOTOR_ENCODER_COUNTS);

pros::ADIEncoder sideEnc (BACK_TRACKER_A, BACK_TRACKER_B);
pros::ADIEncoder rightEnc (RIGHT_TRACKER_A, RIGHT_TRACKER_B);
pros::ADIEncoder leftEnc (LEFT_TRACKER_A, LEFT_TRACKER_B);

pros::Controller master(CONTROLLER_MASTER);
pros::Imu imu (IMU_PORT);

int left_pos(){
  master.print(0, 0, "Distance: %d\n\n", leftEnc.get_value());
  return leftEnc.get_value();
}

int right_pos(){
  master.print(0, 0, "Distance: %d\n\n", rightEnc.get_value());
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
  rightEnc.reset();
  leftEnc.reset();
  sideEnc.reset();
}

void resetPID(){
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

void tankDrive(){
  LF.move(LC * master.get_analog(ANALOG_LEFT_Y));
  LB.move(LC * master.get_analog(ANALOG_LEFT_Y));
  RF.move(RC * master.get_analog(ANALOG_RIGHT_Y));
  RB.move(RC * master.get_analog(ANALOG_RIGHT_Y));
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

    if(distance == target || reps >= 15){
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

    if(distance == target || reps >= 15){
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

void driveForward(double target, int time, float speed){

	double lastDistance = 0;
	double distance;
	int reps = 0;
  bool atTarget = false;
	int startTime = pros::millis();

  while(!atTarget && (pros::millis()-startTime) < time){

    distance = (right_pos() + left_pos()) / 2;

    double rightPower = rightPID.calculate(target, right_pos());
    double leftPower = leftPID.calculate(target, left_pos());

    set_drive(leftPower * speed, rightPower * speed);

		if(lastDistance == distance){
			reps++;
		}
    else{
      reps = 0;
    }

    if(distance == target || reps >= 15){
      atTarget = true;
    }
		else if(distance > target){
			set_drive(-10, -10);
			pros::delay(50);
      set_drive(0,0);
			atTarget = true;
		}

		lastDistance = distance;
    pros::delay(20);
  }
  master.print(0, 0, "Distance: %lf\n\n", distance);
  brake();
}

void driveBackwards(double target, int time, float speed){

	double lastDistance = 0;
	double distance;
	int reps = 0;
  bool atTarget = false;
	int startTime = pros::millis();

  while(!atTarget && (pros::millis()-startTime) < time){

    distance = (right_pos() + left_pos()) / 2;

    double rightPower = rightPID.calculate(target, right_pos());
    double leftPower = leftPID.calculate(target, left_pos());

    set_drive(leftPower * speed, rightPower * speed);

		if(lastDistance == distance){
			reps++;
		}
    else{
      reps = 0;
    }

    if(distance == target && reps >= 15){
      atTarget = true;
    }
		else if(distance < target){
			set_drive(10, 10);
			pros::delay(50);
			atTarget = true;
		}

		lastDistance = distance;
    pros::delay(20);
  }
  master.print(0, 0, "Distance: %lf\n\n", distance);
  brake();
}

void driveForwardSetState(double target, int state, int time, float speed){

	double lastDistance = 0;
	double distance;
	int reps = 0;
  bool atTarget = false;
	int startTime = pros::millis();

  while(!atTarget && (pros::millis()-startTime) < time){

    distance = (right_pos() + left_pos()) / 2;

    double rightPower = rightPID.calculate(target, right_pos());
    double leftPower = leftPID.calculate(target, left_pos());

    set_drive(leftPower * speed, rightPower * speed);

		if(lastDistance == distance){
			reps++;
		}
    else {
      reps = 0;
    }

		if(crossedHalf(distance, target)){
			setState(state);
		}

    if(distance == target && reps >= 15){
      atTarget = true;
    }
		else if(distance > target){
			set_drive(-10, -10);
			pros::delay(50);
      set_drive(0, 0);
			atTarget = true;
		}

		lastDistance = distance;
    pros::delay(10);
  }
	master.print(0, 0, "Distance: %lf\n\n", distance);
  brake();
}

void driveBackwardsSetState(double target, int state, int time, float speed){

	double lastDistance = 0;
	double currAngle = imu.get_heading();
	double distance;
	int reps = 0;
  bool atTarget = false;
	int startTime = pros::millis();

  while(!atTarget && (pros::millis()-startTime) < time){

    distance = (right_pos() + left_pos()) / 2;

    double rightPower = rightPID.calculate(target, right_pos());
    double leftPower = leftPID.calculate(target, left_pos());

    set_drive(leftPower * speed, rightPower * speed);

		if(lastDistance == distance){
			reps++;
		}
    else {
      reps = 0;
    }

		if(crossedHalf(distance, target)){
			setState(state);
		}

    if(distance == target && reps >= 15){
      atTarget = true;
    }
		else if(distance < target){
			set_drive(10, 10);
			pros::delay(50);
      set_drive(0,0);
			atTarget = true;
		}

		lastDistance = distance;
    pros::delay(10);
  }
	master.print(0, 0, "Distance: %lf\n\n", distance);
  brake();
}

void slowDrive(double target, int time, float speed, float endSpeed){

	double lastDistance = 0;
	double initDistance = (right_pos() + left_pos()) / 2;
  double distance = 0;
	int reps = 0;
  bool atTarget = false;
	int startTime = pros::millis();

  while(!atTarget && (pros::millis()-startTime) < time){

    distance = (right_pos() + left_pos()) / 2;

    if (distance > ((4*(target - initDistance))/5)){
      speed = endSpeed;
    }

    double rightPower = rightPID.calculate(target, right_pos());
    double leftPower = leftPID.calculate(target, left_pos());

    set_drive(leftPower * speed, rightPower * speed);

		if(lastDistance == distance){
			reps++;
		}

    if(distance == target && reps >= 15){
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
    else {
      reps = 0;
    }

	  if(angle == target || reps >= 15){
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

	  if(angle == target || reps >= 15){
	     atTarget = 1;
	  }

		lastAngle = angle;
    pros::delay(10);
  }
	printf("Angle: %lf\n\n", angle);
  brake();
	reset();
}

void initDrive(){
  imu.reset();
  pros::delay(2500);
  setDriveBrakes(MOTOR_BRAKE_COAST);
}
