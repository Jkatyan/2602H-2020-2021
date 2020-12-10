#include "main.h"

PID rightPID (DRIVE_RIGHT_KP, DRIVE_RIGHT_KD);
PID leftPID (DRIVE_LEFT_KP, DRIVE_LEFT_KD);
PID turnPID (TURN_KP, TURN_KD);

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

void drive(double target, int time, float speed){

	double lastDistance = 0;
	double distance;
	int reps = 0;
  bool atTarget = false;
	int startTime = pros::millis();

  while(!atTarget && (pros::millis()-startTime) < time){

    distance = (left_pos() + right_pos()) / 2.0;

    double rightPower = rightPID.calculate(target, distance);
    double leftPower = leftPID.calculate(target, distance);

    set_drive(leftPower * speed, rightPower * speed);

		if(lastDistance == distance){
			reps++;
		}
    if(distance == target || reps >= 15){
      atTarget = true;
    }

		lastDistance = distance;
    pros::delay(20);
  }
	master.print(0, 0, "Distance: %lf\n\n", distance);
  brake();
}
