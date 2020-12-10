#include "main.h"
#include "misc/autonSel.h" //Auton Selector Globals
#include "screen/resources.hpp" //Field Elements Display
#include "screen/field.hpp" //Field Display
#include "motors.hpp" //Motor Settings
#include "misc/PID.hpp" //PID Functions
#include "misc/GYRO.hpp" //GYRO Functions
#include "autonomous.hpp" //Auton Functions
#include "initialize.hpp" //Init Variables
#include "opcontrol.hpp" //OP Variables
#include "misc/odometry.hpp"
using namespace okapi; //Okapi? Nani

//MOTORS (Edit in motors.hpp)
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
pros::ADIAnalogIn traypot (TRAYPOT);

/* OKAPILIB Notes
  Notes on how to use:
    -intakeAController.setTarget(200); // Move 200 motor degrees
    -intakeBController.setTarget(200); // Move 200 motor degrees
    -intakeAController.waitUntilSettled();
    -intakeBController.waitUntilSettled();

    -intakeAController.setMaxVelocity(double rpm); (100-600)
*/
// [START] OKAPILIB
const int INTAKE_A_MOTOR_PORT = INTA;
const int INTAKE_B_MOTOR_PORT = INTB;
const int LIFT_MOTOR_PORT = LIFT;
auto intakeAController = AsyncControllerFactory::posPID(INTAKE_A_MOTOR_PORT, INTAKEAKP, INTAKEAKI, INTAKEAKD);
auto intakeBController = AsyncControllerFactory::posPID(INTAKE_B_MOTOR_PORT, INTAKEBKP, INTAKEBKI, INTAKEBKD);
auto liftController = AsyncControllerFactory::posPID(LIFT_MOTOR_PORT, ARMKP, ARMKI, ARMKD);
// [END] OKAPILIB

// [START] INIT
void setIntBrakes(pros::motor_brake_mode_e_t mode){
  inta.set_brake_mode(mode);
  inta.set_brake_mode(mode);
}
void setArmBrakes(pros::motor_brake_mode_e_t mode){
  lift.set_brake_mode(mode);
}
void setMogoBrakes(pros::motor_brake_mode_e_t mode){
  mogo.set_brake_mode(mode);
}
void setDriveBrakes(pros::motor_brake_mode_e_t mode){
  lB.set_brake_mode(mode);
  lF.set_brake_mode(mode);
  rB.set_brake_mode(mode);
  rF.set_brake_mode(mode);
}
static lv_res_t btn_click_action(lv_obj_t * btn) //Auton Selector Buttons Code
{
    uint8_t id = lv_obj_get_free_num(btn);

    if(id == 0)
    {
      color++;
      xA++;
      if(color == 2){
        color = 0;
      }
      if (color == 0){
        char buffer[100];
		sprintf(buffer, "Color: Red");
		lv_label_set_text(myLabel, buffer);
    colorLock = 0;
  }
  else{
    char buffer[100];
sprintf(buffer, "Color: Blue");
lv_label_set_text(myLabel, buffer);
colorLock = 1;
  }
    }
    else if (id == 1)
    {
    auton++;
    xA++;
    if(auton == 9){
      auton = 1;
    }
      char buffer[100];
  sprintf(buffer, "Auton: %i",auton);
  lv_label_set_text(myLabel2, buffer);
    }
else if (id == 2){
  if(xA==0){}
  else{
if (colorLock == 0){
  setAuton = auton;
  if(auton == 1){
    char buffer[100];
    sprintf(buffer, "Auton Selected: R1 ID > %i", setAuton);
    lv_label_set_text(myLabel3, buffer);
    selected = 1;

  }
  else if(auton == 2){
    char buffer[100];
    sprintf(buffer, "Auton Selected: R2 ID > %i", setAuton);
    lv_label_set_text(myLabel3, buffer);
    selected = 1;

  }
  else if(auton == 3){
    char buffer[100];
    sprintf(buffer, "Auton Selected: R3 ID > %i", setAuton);
    lv_label_set_text(myLabel3, buffer);
    selected = 1;

  }else if(auton == 4){
    char buffer[100];
    sprintf(buffer, "Auton Selected: R4 ID > %i", setAuton);
    lv_label_set_text(myLabel3, buffer);
    selected = 1;

  }else if(auton == 5){
    char buffer[100];
    sprintf(buffer, "Auton Selected: R5 ID > %i", setAuton);
    lv_label_set_text(myLabel3, buffer);
    selected = 1;

  }else if(auton == 6){
    char buffer[100];
    sprintf(buffer, "Auton Selected: R6 ID > %i", setAuton);
    lv_label_set_text(myLabel3, buffer);
    selected = 1;

  }else if(auton == 7){
    char buffer[100];
    sprintf(buffer, "Auton Selected: R7 ID > %i", setAuton);
    lv_label_set_text(myLabel3, buffer);
    selected = 1;

  }
  else{
    char buffer[100];
    sprintf(buffer, "Auton Selected: R8 ID > %i", setAuton);
    lv_label_set_text(myLabel3, buffer);
    selected = 1;

  }
}
else{
if(auton == 1){
  setAuton = 9;
  char buffer[100];
  sprintf(buffer, "Auton Selected: B1   ID > %i", setAuton);
  lv_label_set_text(myLabel3, buffer);
  selected = 1;

}
else if(auton == 2){
  setAuton = 10;
  char buffer[100];
  sprintf(buffer, "Auton Selected: B2   ID > %i", setAuton);
  lv_label_set_text(myLabel3, buffer);
  selected = 1;

}
else if(auton == 3){
  setAuton = 11;
  char buffer[100];
  sprintf(buffer, "Auton Selected: B3   ID > %i", setAuton);
  lv_label_set_text(myLabel3, buffer);
  selected = 1;

}
else if(auton == 4){
  setAuton = 12;
  char buffer[100];
  sprintf(buffer, "Auton Selected: B4   ID > %i", setAuton);
  lv_label_set_text(myLabel3, buffer);
  selected = 1;

}
else if(auton == 5){
  setAuton = 13;
  char buffer[100];
  sprintf(buffer, "Auton Selected: B5   ID > %i", setAuton);
  lv_label_set_text(myLabel3, buffer);
  selected = 1;

}
else if(auton == 6){
  setAuton = 14;
  char buffer[100];
  sprintf(buffer, "Auton Selected: B6   ID > %i", setAuton);
  lv_label_set_text(myLabel3, buffer);
  selected = 1;

}
else if(auton == 7){
  setAuton = 15;
  char buffer[100];
  sprintf(buffer, "Auton Selected: B7   ID > %i", setAuton);
  lv_label_set_text(myLabel3, buffer);
  selected = 1;

}
else{
  setAuton = 16;
  char buffer[100];
  sprintf(buffer, "Auton Selected: B8   ID > %i", setAuton);
  lv_label_set_text(myLabel3, buffer);
  selected = 1;
}
}
}
}
else{
  //PROG SKILLS SELECT
  setAuton = 17;
  char buffer[100];
  sprintf(buffer, "Auton Selected: Prog Skills   ID > %i", setAuton);
  lv_label_set_text(myLabel3, buffer);

  char color[100];
sprintf(color, "Color: Red");
lv_label_set_text(myLabel, color);

char prog[100];
sprintf(prog, "Auton: PROG");
lv_label_set_text(myLabel2, prog);

selected = 1;
}
    return LV_RES_OK;
}
void initialize() {
  pros::delay(1000);
	//Gyro Init Code
 //gyroCalibrate();
 //pros::delay(1510);
 pros::delay(20);
setIntBrakes(MOTOR_BRAKE_COAST);
setArmBrakes(MOTOR_BRAKE_HOLD);
setMogoBrakes(MOTOR_BRAKE_HOLD);
setDriveBrakes(MOTOR_BRAKE_COAST);
} //Initialize Gyro / Set Brakes
void disabled() {}
void competition_initialize() {
	screen::initializeStyles();

	 lv_obj_t *scr = lv_obj_create(NULL, NULL);
	 lv_scr_load(scr);

	 screen::Field field(scr);

	 field.setPos( 315 , 70 );

	 field.setSideLength( 160 );

	 //field.drawRobot(true, 160);

	 field.finishDrawing();

		 lv_style_copy(&myButtonStyleREL, &lv_style_plain);
		 myButtonStyleREL.body.main_color = LV_COLOR_MAKE(0, 0, 0);
		 myButtonStyleREL.body.grad_color = LV_COLOR_MAKE(0, 0, 150);
		 myButtonStyleREL.body.radius = 0;
		 myButtonStyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

		 lv_style_copy(&myButtonStylePR, &lv_style_plain);
		 myButtonStylePR.body.main_color = LV_COLOR_MAKE(66, 244, 238);
		 myButtonStylePR.body.grad_color = LV_COLOR_MAKE(0, 0, 255);
		 myButtonStylePR.body.radius = 0;
		 myButtonStylePR.text.color = LV_COLOR_MAKE(255, 255, 255);

		 myButton = lv_btn_create(lv_scr_act(), NULL);
		 lv_obj_set_free_num(myButton, 0);
		 lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action);
		 lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL);
		 lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR);
		 lv_obj_set_size(myButton, 135, 50);
		 lv_obj_align(myButton, NULL, LV_ALIGN_IN_TOP_LEFT, 20, 10);

		 myButton2 = lv_btn_create(lv_scr_act(), NULL);
		 lv_obj_set_free_num(myButton2, 1);
		 lv_btn_set_action(myButton2, LV_BTN_ACTION_CLICK, btn_click_action);
		 lv_btn_set_style(myButton2, LV_BTN_STYLE_REL, &myButtonStyleREL);
		 lv_btn_set_style(myButton2, LV_BTN_STYLE_PR, &myButtonStylePR);
		 lv_obj_set_size(myButton2, 135, 50);
		 lv_obj_align(myButton2, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);

		 myButton3 = lv_btn_create(lv_scr_act(), NULL);
		 lv_obj_set_free_num(myButton3, 2);
		 lv_btn_set_action(myButton3, LV_BTN_ACTION_CLICK, btn_click_action);
		 lv_btn_set_style(myButton3, LV_BTN_STYLE_REL, &myButtonStyleREL);
		 lv_btn_set_style(myButton3, LV_BTN_STYLE_PR, &myButtonStylePR);
		 lv_obj_set_size(myButton3, 135, 50);
		 lv_obj_align(myButton3, NULL, LV_ALIGN_IN_TOP_RIGHT, -20, 10);

		 myButton4 = lv_btn_create(lv_scr_act(), NULL);
		 lv_obj_set_free_num(myButton4, 3);
		 lv_btn_set_action(myButton4, LV_BTN_ACTION_CLICK, btn_click_action);
		 lv_btn_set_style(myButton4, LV_BTN_STYLE_REL, &myButtonStyleREL);
		 lv_btn_set_style(myButton4, LV_BTN_STYLE_PR, &myButtonStylePR);
		 lv_obj_set_size(myButton4, 135, 30);
		 lv_obj_align(myButton4, NULL, LV_ALIGN_IN_TOP_LEFT, 20, 70);

		 myButtonLabel = lv_label_create(myButton, NULL);
		 lv_label_set_text(myButtonLabel, "Color");

		 myLabel = lv_label_create(lv_scr_act(), NULL);
		 lv_label_set_text(myLabel, "Color: None");
		 lv_obj_align(myLabel, NULL, LV_ALIGN_IN_LEFT_MID, 10, 10);

		 myButtonLabel2 = lv_label_create(myButton2, NULL);
		 lv_label_set_text(myButtonLabel2, "Auton");

		 myLabel2 = lv_label_create(lv_scr_act(), NULL);
		 lv_label_set_text(myLabel2, "Auton: None");
		 lv_obj_align(myLabel2, NULL, LV_ALIGN_CENTER, -15, 10);

		 myButtonLabel3 = lv_label_create(myButton3, NULL);
		 lv_label_set_text(myButtonLabel3, "Select");

		 myButtonLabel4 = lv_label_create(myButton4, NULL);
		 lv_label_set_text(myButtonLabel4, "Skills");

		 myLabel3 = lv_label_create(lv_scr_act(), NULL);
		 lv_label_set_text(myLabel3, "Auton Selected:");
		 lv_obj_align(myLabel3, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 10, -40);


 while(selected == 0){

 if(colorLock==1){
	 switch(auton){
		 case 1:
		 tileVar = 9;
		 break;
		 case 2:
		 tileVar = 10;
		 break;
		 case 3:
		 tileVar = 11;
		 break;
		 case 4:
		 tileVar = 12;
		 break;
		 case 5:
		 tileVar = 13;
		 break;
		 case 6:
		 tileVar = 14;
		 break;
		 case 7:
		 tileVar = 15;
		 break;
		 case 8:
		 tileVar = 16;
		 break;
	 }
 }
 else{
	 switch(auton){
		 case 1:
		 tileVar = 1;
		 break;
		 case 2:
		 tileVar = 2;
		 break;
		 case 3:
		 tileVar = 3;
		 break;
		 case 4:
		 tileVar = 4;
		 break;
		 case 5:
		 tileVar = 5;
		 break;
		 case 6:
		 tileVar = 6;
		 break;
		 case 7:
		 tileVar = 7;
		 break;
		 case 8:
		 tileVar = 8;
		 break;
	 }
 }
 switch(tileVar){
	 case 1:
	 field.clean();
	 field.drawRobot(true, 69);
	 field.finishDrawing();
	 break;
	 case 2:
	 field.clean();
	 field.drawRobot(true, 69);
	 field.finishDrawing();
	 break;
	 case 3:
	 field.clean();
	 field.drawRobot(true, 105);
	 field.finishDrawing();
	 break;
	 case 4:
	 field.clean();
	 field.drawRobot(true, 105);
	 field.finishDrawing();
	 break;
	 case 5:
	 field.clean();
	 field.drawRobot(true, 142);
	 field.finishDrawing();
	 break;
	 case 6:
	 field.clean();
	 field.drawRobot(true, 142);
	 field.finishDrawing();
	 break;
	 case 7:
	 field.clean();
	 field.drawRobot(true, 179);
	 field.finishDrawing();
	 break;
	 case 8:
	 field.clean();
	 field.drawRobot(true, 179);
	 field.finishDrawing();
	 break;
	 case 9:
	 field.clean();
	 field.drawRobot(false, 69);
	 field.finishDrawing();
	 break;
	 case 10:
	 field.clean();
	 field.drawRobot(false, 69);
	 field.finishDrawing();
	 break;
	 case 11:
	 field.clean();
	 field.drawRobot(false, 105);
	 field.finishDrawing();
	 break;
	 case 12:
	 field.clean();
	 field.drawRobot(false, 105);
	 field.finishDrawing();
	 break;
	 case 13:
	 field.clean();
	 field.drawRobot(false, 142);
	 field.finishDrawing();
	 break;
	 case 14:
	 field.clean();
	 field.drawRobot(false, 142);
	 field.finishDrawing();
	 break;
	 case 15:
	 field.clean();
	 field.drawRobot(false, 179);
	 field.finishDrawing();
	 break;
	 case 16:
	 field.clean();
	 field.drawRobot(false, 179);
	 field.finishDrawing();
	 break;
 }
  pros::delay(20);
 }
 if(selected != 0){
 }

} //Auton Selector
// [END] INIT

// [START] AUTON
void autonomous() {
	if(setAuton == 0){} //But it isn't :P
	else{
  lv_obj_clean(lv_scr_act());
	//Auton PIDs
	drivePID = pidInit (DRIVEKP, DRIVEKI, DRIVEKD, 0, 100.0,5,15);
	slowDrivePID = pidInit (DRIVEKP, DRIVEKI, DRIVEKD, 0, 100.0,5,15);
	gyroDrivePID = pidInit(GYRODKP, GYRODKI, GYRODKD, 0, 40, LARGE, LARGE);
	gyroPID = pidInit(GYROKP, GYROKI, GYROKD, 0, 10, LARGE, LARGE);
//  trayPID = pidInit(TRAYKP, TRAYKI, TRAYKD, LARGE, LARGE, LARGE, LARGE);
  armPID = pidInit(ARMKP, ARMKI, ARMKD, LARGE, LARGE, LARGE, LARGE);
  intAPID = pidInit(INTAKEAKP, INTAKEAKI, INTAKEAKD, LARGE, LARGE, LARGE, LARGE);
  intBPID = pidInit(INTAKEBKP, INTAKEBKI, INTAKEBKD, LARGE, LARGE, LARGE, LARGE);

  myLabelGyro = lv_label_create(lv_scr_act(), NULL);lv_label_set_text(myLabelGyro, "Gyro Value:");lv_obj_align(myLabelGyro, NULL, LV_ALIGN_CENTER, 0, 0);
  myLabelEnc = lv_label_create(lv_scr_act(), NULL);lv_label_set_text(myLabelEnc, "Drive Encoder Value:");lv_obj_align(myLabelEnc, NULL, LV_ALIGN_CENTER, 0, 50);
  myLabelTarget = lv_label_create(lv_scr_act(), NULL);lv_label_set_text(myLabelTarget, "Distance From Target:");lv_obj_align(myLabelTarget, NULL, LV_ALIGN_CENTER, 0, -50);
  stop();
  reset();
  lastSlewTime = pros::millis();

	switch(setAuton){ //Create autons here!
	  case 1: //Red 1 1
	  enablePreset(1);
    targetTurn(2000,5000,0,1);
    targetTurn(-2000,5000,0,1);
	  break;
	  case 2: //Red 1 2
	  enablePreset(1);

	  break;
	  case 3: //Red 2 1
	  enablePreset(1);

	  break;
	  case 4: //Red 2 2
	  enablePreset(1);

	  break;
	  case 5: //Red 3 1
	  enablePreset(1);

	  break;
	  case 6: //Red 3 2
	  enablePreset(1);

	  break;
	  case 7: //Red 4 1
	  enablePreset(1);

	  break;
	  case 8: //Red 4 2
	  enablePreset(1);

	  break;
	  case 9: //Blue 1 1
	  enablePreset(2);

	  break;
	  case 10: //Blue 1 2
	  enablePreset(2);

	  break;
	  case 11: //Blue 2 1
	  enablePreset(2);

	  break;
	  case 12: //Blue 2 2
	  enablePreset(2);

	  break;
	  case 13: //Blue 3 1
	  enablePreset(2);

	  break;
	  case 14: //Blue 3 2
	  enablePreset(2);

	  break;
	  case 15: //Blue 4 1
	  enablePreset(2);

	  break;
	  case 16: //Blue 4 2
	  enablePreset(2);

	  break;
	  case 17: //Programming Skills
	  enablePreset(1);

	  break;
	}
	}
  pros::delay(20);
}
void enablePreset(int preset){
  if(preset == 1){
    preset1 = 1;
    preset2 = 0;
  }
  else if(preset == 2){
    preset2 = 1;
    preset1 = 0;
  }
}
void rawDriveTarget(int side, float angle, int target, int time, int turn, float speed, bool slow, bool slew, float rightCorrection, float leftCorrection){
  angle = angle*side;
  int atTarget = 0;
  int driveEnc = 0;
  int distance = 0;
  int startTime = pros::millis();

  	while ((atTarget != 1) && (pros::millis()-startTime) < time) {
      if(turn == 0){
         driveEnc = ((abs((lB.get_position() + lF.get_position())/2))+(abs((rB.get_position() + rF.get_position()))/2))/2;
      }
      else{
         driveEnc = (((lB.get_position() + lF.get_position())/2)+((rB.get_position() + rF.get_position())/2))/2;
    }
    distance = target - driveEnc;
sprintf(bufG, "Gyro Value: %lf", gyroGetRate()/10.0);
lv_label_set_text(myLabelGyro, bufG);

sprintf(bufE, "Drive Encoder Value: %i", driveEnc);
lv_label_set_text(myLabelEnc, bufE);

sprintf(bufT, "Distance From Target: %i", distance);
lv_label_set_text(myLabelTarget, bufT);
//********** CALCULATIONS

      if (!slow){
      float val = pidCalculate(drivePID, target, -driveEnc)*speed;
      val = (slew)? slewRateCalculate(val): val;
      int rightVal = val - pidCalculate(gyroDrivePID, angle, gyroGetRate()/10.0);
      int leftVal = val + pidCalculate(gyroDrivePID, angle, gyroGetRate()/10.0);

      switch(turn){
        case 2:
        rF.move(rightCorrection*rightVal);
        rB.move(rightCorrection*rightVal);
        lF.move(leftCorrection*-leftVal);
        lB.move(leftCorrection*-leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 0:
        rF.move(rightCorrection*rightVal);
        rB.move(rightCorrection*rightVal);
        lF.move(leftCorrection*leftVal);
        lB.move(leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 1:
        rF.move(rightCorrection*-rightVal);
        rB.move(rightCorrection*-rightVal);
        lF.move(leftCorrection*leftVal);
        lB.move(leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
      }
      }
      else{
//********** CALCULATIONS

      float val = pidCalculate(slowDrivePID, target, driveEnc)*speed;
      val = (slew)? slewRateCalculate(val): val;
      int rightVal = val - pidCalculate(gyroDrivePID, angle, gyroGetRate()/10.0);
      int leftVal = val + pidCalculate(gyroDrivePID, angle, gyroGetRate()/10.0);

      switch(turn){
        case 2:
        rF.move(rightCorrection*rightVal);
        rB.move(rightCorrection*rightVal);
        lF.move(leftCorrection*-leftVal);
        lB.move(leftCorrection*-leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 0:
        rF.move(rightCorrection*rightVal);
        rB.move(rightCorrection*rightVal);
        lF.move(leftCorrection*leftVal);
        lB.move(leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 1:
        rF.move(rightCorrection*-rightVal);
        rB.move(rightCorrection*-rightVal);
        lF.move(leftCorrection*leftVal);
        lB.move(leftCorrection*leftVal);
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

    distance = target - driveEnc;
    lF.set_zero_position(driveEnc-target);
    lB.set_zero_position(driveEnc-target);
    rF.set_zero_position(driveEnc-target);
    rB.set_zero_position(driveEnc-target);
    stop();

}
void motorTarget(int port, float p, float i, float d, int special, int target, int time, float speed, float accel, bool slew){
  PID taskPID = pidInit(p, i, d, LARGE, LARGE, LARGE, LARGE);
  pros::Motor motor(port);
  int atTarget = 0;
  int encoder = 0;
  int startTime = pros::millis();
	while ((atTarget != 1) && (pros::millis()-startTime) < time) {
  switch(special){case 0:encoder = motor.get_position();break;case 1:encoder = traypot.get_value();break;default:break;}
  sprintf(bufE, "Encoder Value: %i", encoder);
  lv_label_set_text(myLabelEnc, bufE);

  float val = pidCalculate(taskPID, target, encoder)*speed;
  val = (slew)? motorSlew(val, accel): val;
  motor.move(val);
  if(encoder == target){
    atTarget = 1;
  }
  pros::delay(15);
  }
  motor.move(0);

}
void rawTargetTurn(int target, int time, int turn, float speed, bool slow, bool slew, float rightCorrection, float leftCorrection){
  //angle = angle*side;
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
         driveEnc = ((abs((lB.get_position() + lF.get_position())/2))+(abs((rB.get_position() + rF.get_position()))/2))/2;
      }
      else{
         driveEnc = (((lB.get_position() + lF.get_position())/2)+((rB.get_position() + rF.get_position())/2))/2;
    }
    distance = target - driveEnc;
lv_label_set_text(myLabelGyro, bufG);

sprintf(bufE, "Drive Encoder Value: %i", driveEnc);
lv_label_set_text(myLabelEnc, bufE);

sprintf(bufT, "Distance From Target: %i", distance);
lv_label_set_text(myLabelTarget, bufT);
//********** CALCULATIONS

      if (!slow){
      float val = pidCalculate(drivePID, target, driveEnc)*speed;
      val = (slew)? slewRateCalculate(val): val;
      int rightVal = val;// - pidCalculate(gyroDrivePID, angle, gyro.get_value()/10.0);
      int leftVal = val;// + pidCalculate(gyroDrivePID, angle, gyro.get_value()/10.0);

      switch(turn){
        case 2:
        rF.move(rightCorrection*rightVal);
        rB.move(rightCorrection*rightVal);
        lF.move(leftCorrection*-leftVal);
        lB.move(leftCorrection*-leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 0:
        rF.move(rightCorrection*rightVal);
        rB.move(rightCorrection*rightVal);
        lF.move(leftCorrection*leftVal);
        lB.move(leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 1:
        rF.move(rightCorrection*-rightVal);
        rB.move(rightCorrection*-rightVal);
        lF.move(leftCorrection*leftVal);
        lB.move(leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
      }
      }
      else{
//********** CALCULATIONS

      float val = pidCalculate(slowDrivePID, target, driveEnc)*speed;
      val = (slew)? slewRateCalculate(val): val;
      int rightVal = val;// - pidCalculate(gyroDrivePID, angle, gyro.get_value()/10.0);
      int leftVal = val;// + pidCalculate(gyroDrivePID, angle, gyro.get_value()/10.0);

      switch(turn){
        case 2:
        rF.move(rightCorrection*rightVal);
        rB.move(rightCorrection*rightVal);
        lF.move(leftCorrection*-leftVal);
        lB.move(leftCorrection*-leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 0:
        rF.move(rightCorrection*rightVal);
        rB.move(rightCorrection*rightVal);
        lF.move(leftCorrection*leftVal);
        lB.move(leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 1:
        rF.move(rightCorrection*-rightVal);
        rB.move(rightCorrection*-rightVal);
        lF.move(leftCorrection*leftVal);
        lB.move(leftCorrection*leftVal);
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
}
void gyroTurn(int target, int accuracy, int time, float speed) {
	int startTime = pros::millis();
	bool gyroAtTarget = false;
	int repsAtTarget = 0;
	//go into the loop that will repeat to update motor values and break when at target
	while (!gyroAtTarget  && (pros::millis()-startTime) < time) {
		// calculate the desired motor value based on the sensor value relative to the target
		float drive = pidCalculate(gyroPID, target, gyroGetRate()/10.0)*speed;
		drive = ((fabs(gyroGetRate()/10.0-target)>180)? -1 : 1)*drive;
    lF.move(drive);
    lB.move(drive);
    rF.move(-drive);
    rB.move(-drive);
		//if the sensor value is within the desired range of the target
		if (fabs(gyroGetRate()/10.0-target) < accuracy) {
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
	stop();
}
int driveTarget(int target, float angle, int time, float speed){
  if(preset1 == 1){
    rawDriveTarget (p1.side , angle, target, time, 0, speed, p1.slow , p1.slew , p1.rightCorrection , p1.leftCorrection);
    return 0;
  }
  else if(preset2 == 1){
    rawDriveTarget (p2.side , angle, target, time, 0, speed, p2.slow , p2.slew , p2.rightCorrection , p2.leftCorrection);
    return 0;
  }
  else{
    return 1;
  }} //Preset Code
int targetTurn(int target, int time, int turn, float speed){
  if(preset1 == 1){
    rawTargetTurn (target, time, turn, speed, p1.slow , p1.slew , p1.rightCorrection , p1.leftCorrection);
    return 0;
  }
  else if(preset2 == 1){
    rawTargetTurn (target, time, turn, speed, p2.slow , p2.slew , p2.rightCorrection , p2.leftCorrection);
    return 0;
  }
  else{
    return 1;
  }} //Preset Code
void stop(){
  lB.set_zero_position(lB.get_position());
  lF.set_zero_position(lF.get_position());
  rB.set_zero_position(rB.get_position());
  rF.set_zero_position(rB.get_position());
  lB.move(0);
  lF.move(0);
  rB.move(0);
  rF.move(0);} //Sets zero position / stops motors.
  void mogoStop(){
    //mogo.set_zero_position(mogo.get_position());
    mogo.move(0);} //Sets zero position / stops motors.
void reset(){
  stop();
  lB.tare_position();
  lF.tare_position();
  rB.tare_position();
  rF.tare_position();
  lB.move(0);
  lF.move(0);
  rB.move(0);
  rF.move(0);} //Resets Drive encoder values, Encoder error, and stops motors.
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
		return returnVal;} //1D Motion Profiler
float motorSlew (float desiredRate, float maxAccelTray) {

    		float deltaTime = pros::millis()-lastSlewTime;
    		float desiredAccel = (desiredRate -lastSlewRate)/deltaTime;
    		float addedRate;
    		float newRate;

    		if (fabs(desiredAccel) < maxAccelTray || (desiredAccel<0 && desiredRate>0) || (desiredAccel>0 && desiredRate<0)) {
    		    addedRate = desiredAccel*deltaTime;
    		    newRate = addedRate+lastSlewRate;
    		}
    		else {
    		    addedRate = ((desiredAccel>0)? 1: -1)*maxAccelTray*deltaTime;
            newRate = addedRate+lastSlewRate;
    		}
    	  lastSlewTime = lastSlewTime+deltaTime;
    	  lastSlewRate = newRate;

    		float returnVal = newRate;
    		return returnVal;}
void moveToPoint(float targetX, float targetY) {
          PID turnPID = pidInit (TURNKP, TURNKI, TURNKD, 0, 100.0,5,15);
        	bool atPoint = false;
        	float targetAngle =0;
        	float power =0;
        	float turnPower =0;
        	lastSlewTime = pros::millis();

        	while (!atPoint) {
        		updatePosition();

        		power = -pidCalculate(drivePID, 0, sqrt(pow(targetY-getY(),2) + pow(targetX-getX(),2)));
        		power = slewRateCalculate(power);

        		targetAngle = atan2f((targetY-getY()),(targetX-getX()))-M_PI/2;
        		if (targetAngle >= M_PI) {
        	    targetAngle-=2*M_PI;
        	  }
        	  else if (targetAngle <= -M_PI) {
        	    targetAngle+=2*M_PI;
        	  }

        		turnPower = ((fabs(targetAngle-getAngle())>M_PI)? -1: 1)*pidCalculate(turnPID, targetAngle, getAngle());

        		lF.move((power + turnPower)*LC);
            lB.move((power + turnPower)*LC);
            rF.move((power - turnPower)*RC);
            rB.move((power - turnPower)*RC);

        		pros::lcd::print(0, "X: %f", getX());
        		pros::lcd::print(1, "Y: %f", getY());
        		pros::lcd::print(2, "Angle: %f", getAngleDegrees());

        		pros::lcd::print(7, "Target Angle: %f", targetAngle*180/M_PI);


        		if (sqrt(pow(targetY-getY(),2) + pow(targetX-getX(),2)) < 3 || master.get_digital(DIGITAL_B)) {
        			atPoint = true;
        		}
        		pros::delay(10);
        	}
        }
// [END] AUTON

// [START] OP
void opcontrol() {
    int trayRunning = 0;
		lv_obj_clean(lv_scr_act());
    drivePID = pidInit (DRIVEKP, DRIVEKI, DRIVEKD, 0, 100.0,5,15);
  	slowDrivePID = pidInit (DRIVEKP, DRIVEKI, DRIVEKD, 0, 100.0,5,15);
  	gyroDrivePID = pidInit(GYRODKP, GYRODKI, GYRODKD, 0, 40, LARGE, LARGE);
  	gyroPID = pidInit(GYROKP, GYROKI, GYROKD, 0, 10, LARGE, LARGE);
    trayPID = pidInit(TRAYKP, TRAYKI, TRAYKD, LARGE, LARGE, LARGE, LARGE);
    armPID = pidInit(ARMKP, ARMKI, ARMKD, LARGE, LARGE, LARGE, LARGE);
    intAPID = pidInit(INTAKEAKP, INTAKEAKI, INTAKEAKD, LARGE, LARGE, LARGE, LARGE);
    intBPID = pidInit(INTAKEBKP, INTAKEBKI, INTAKEBKD, LARGE, LARGE, LARGE, LARGE);
		while(Brian <3){ //ALWAYS true
      do { //One-Time init loop for auton
      } while(setMotor < 0);
			//Chassis Control (Tray Control)
      if(master.get_digital(DIGITAL_X)){
      mogo.move(master.get_analog(ANALOG_RIGHT_Y));
      }
      else{
			lF.move(LC*(master.get_analog(ANALOG_LEFT_Y)));
			lB.move(LC*(master.get_analog(ANALOG_LEFT_Y)));
			rF.move(RC*(master.get_analog(ANALOG_RIGHT_Y)));
			rB.move(RC*(master.get_analog(ANALOG_RIGHT_Y)));
      }
      if(master.get_digital(DIGITAL_A)){trayRunning = 1;}
      if(trayRunning == 1){
        motorTarget(MOGO, TRAYKP, TRAYKI, TRAYKD, 1, 1085, 1000, 1, 0.11, true);
        trayRunning = 0;
      }
      if (master.get_digital(DIGITAL_L1)){
				lift.move(127);
				//double(positionsTray) = mogo.get_position();
				lastTime = pros::millis();
			}
			else if (master.get_digital(DIGITAL_L2)){
				lift.move(-127);
				//double(positionsTray) = mogo.get_position();
				lastTime = pros::millis();
			}
		/*	else if((pros::millis() - lastTime) > 200){
			lift.move(pidCalculate(liftPID, lift.get_position(), lift.get_position()));
    }*/
      if (master.get_digital(DIGITAL_R1)){
				inta.move(127);
        intb.move(-127);
				//double(positionsTray) = mogo.get_position();
				lastTime = pros::millis();
			}
			else if (master.get_digital(DIGITAL_R2)){
				inta.move(-127);
        intb.move(127);
				//double(positionsTray) = mogo.get_position();
				lastTime = pros::millis();
			}
		/*	else if((pros::millis() - lastTime) > 200){
			lift.move(pidCalculate(intAPID, inta.get_position(), lift.get_position()));
    }*/
		pros::delay(20);
		}
	} //Driver Control Code
// [END] OP
