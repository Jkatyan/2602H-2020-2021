#include "autonSel.h"
#include "PID.hpp"
#include "autonomous.hpp"

// AUTON SETTINGS *** IMPORTANT ***
#define RC 0.9927 //Right Chassis Coefficient
#define LC 1 //Left Chassis Coefficient

lv_obj_t * myLabelGyro;
char bufG[100];

lv_obj_t * myLabelEnc;
char bufE[100];

lv_obj_t * myLabelTarget;
char bufT[100];

pros::Controller amaster (CONTROLLER_MASTER);

pros::Motor alB(L2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor alF(L1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor arB(R2, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor arF(R1, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor adoor(DOOR, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor alift(LIFT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor adiag(INTA, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor avert(INTB, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

void autonomous() {
  if(setAuton == 0){}
  else{
  lv_obj_clean(lv_scr_act());

  myLabelGyro = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(myLabelGyro, "Gyro Value:");
  lv_obj_align(myLabelGyro, NULL, LV_ALIGN_CENTER, 0, 0);

  myLabelEnc = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(myLabelEnc, "Drive Encoder Value:");
  lv_obj_align(myLabelEnc, NULL, LV_ALIGN_CENTER, 0, 50);

  myLabelTarget = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(myLabelTarget, "Distance From Target:");
  lv_obj_align(myLabelTarget, NULL, LV_ALIGN_CENTER, 0, -50);

  stop();
  reset();
  lastSlewTime = pros::millis();

drivePID = pidInit (0.61, 0.01, 0.1, 0, 100.0,5,15);
slowDrivePID = pidInit (0.21, 0, 0.1, 0, 100.0,5,15);

pros::delay(20);

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
  driveTarget(525, 3000, 0, 0.8, false, true, RC, LC);
  reset();
  driveTarget(360, 1000, 1, 0.8, false, true, RC, LC);
  reset();
  driveTarget(500, 3000, 0, 0.8, false, true, RC, LC);
  driveTarget(0, 3000, 0, 0.8, false, true, RC, LC);
  reset();
  driveTarget(360, 1000, 2, 0.8, false, true, RC, LC);
  reset();
  driveTarget(-525, 3000, 0, 0.8, false, true, RC, LC);
  pros::delay(99999);
  break;
}
}
}//Void Auton
void driveTarget(int target, int time, int turn, float speed, bool slow, bool slew, float rightCorrection, float leftCorrection){
  int atTarget = 0;
  int driveEnc = 0;
  int distance = 0;
  int startTime = pros::millis();
    /*if(turn != 0){
      reset();
      stop();
    }*/
  	while ((atTarget != 1) && (pros::millis()-startTime) < time) {
      if(turn != 0){
         driveEnc = ((abs((alB.get_position() + alF.get_position())/2))+(abs((arB.get_position() + arF.get_position()))/2))/2;
      }
      else{
       driveEnc = (((alB.get_position() + alF.get_position())/2)+((arB.get_position() + arF.get_position())/2))/2;
    }
    distance = target - driveEnc;
sprintf(bufG, "Gyro Value: GYRO DISABLED");
lv_label_set_text(myLabelGyro, bufG);

sprintf(bufE, "Drive Encoder Value: %i", driveEnc);
lv_label_set_text(myLabelEnc, bufE);

sprintf(bufT, "Distance From Target: %i", distance);
lv_label_set_text(myLabelTarget, bufT);

      if (!slow){
      int val = pidCalculate(drivePID, target, driveEnc);
      val = (slew)? slewRateCalculate(val): val;
      int rightVal = val;
      int leftVal = val;
      switch(turn){
        case 2:
        arF.move(speed*rightCorrection*rightVal);
        arB.move(speed*rightCorrection*rightVal);
        alF.move(speed*leftCorrection*-leftVal);
        alB.move(speed*leftCorrection*-leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 0:
        arF.move(speed*rightCorrection*rightVal);
        arB.move(speed*rightCorrection*rightVal);
        alF.move(speed*leftCorrection*leftVal);
        alB.move(speed*leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 1:
        arF.move(speed*rightCorrection*-rightVal);
        arB.move(speed*rightCorrection*-rightVal);
        alF.move(speed*leftCorrection*leftVal);
        alB.move(speed*leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
      }
      }
      else{
      int val = pidCalculate(slowDrivePID, target, driveEnc);
      val = (slew)? slewRateCalculate(val): val;
      int rightVal = val;
      int leftVal = val;
      switch(turn){
        case 2:
        arF.move(speed*rightCorrection*rightVal);
        arB.move(speed*rightCorrection*rightVal);
        alF.move(speed*leftCorrection*-leftVal);
        alB.move(speed*leftCorrection*-leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 0:
        arF.move(speed*rightCorrection*rightVal);
        arB.move(speed*rightCorrection*rightVal);
        alF.move(speed*leftCorrection*leftVal);
        alB.move(speed*leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 1:
        arF.move(speed*rightCorrection*-rightVal);
        arB.move(speed*rightCorrection*-rightVal);
        alF.move(speed*leftCorrection*leftVal);
        alB.move(speed*leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
      }
      }
      pros::delay(15);
    } //While Loop
    /*if(turn != 0){
      reset();
      stop();
    }
    else{*/
    stop();
  //}
} //Drive Target
void stop(){
  alB.set_zero_position(alB.get_position());
  alF.set_zero_position(alF.get_position());
  arB.set_zero_position(arB.get_position());
  arF.set_zero_position(arB.get_position());
  alB.move(0);
  alF.move(0);
  arB.move(0);
  arF.move(0);
}
void reset(){
  alB.tare_position();
  alF.tare_position();
  arB.tare_position();
  arF.tare_position();
  alB.move(0);
  alF.move(0);
  arB.move(0);
  arF.move(0);
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
