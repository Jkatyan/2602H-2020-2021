#include "main.h"
#include "initialize.hpp"
#include "autonomous.hpp"

//Initialize
void initialize() {
	driveBrakeMode(MOTOR_BRAKE_COAST);
	liftBrakeMode(MOTOR_BRAKE_HOLD);
	clawBrakeMode(MOTOR_BRAKE_HOLD);
	pros::ADIGyro gyro (GYROPORT, GC);
	pros::delay(2000);
}

//Disabled Mode
void disabled() {}

//Competition Initialize Mode
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

}

//Autonomous Control
void autonomous() {
	drivePID = pidInit (0.95, 0.01, 0.1, 0, 100.0,5,15);
	gyroDrivePID = pidInit(1.2, 0, 0.6, 0, 40, 5, 15);
	stop();
	reset();
	lv_obj_clean(lv_scr_act()); //Clear Display
	switch(setAuton){ //Create autons here!
		case 1: //Red 1 1
		enablePreset(1);

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
	pros::delay(20);
}

//Driver Control
void opcontrol() {
	while (true) {
		tankDrive(); //Tank Drive Control
		clawControl(); //Claw Control
		liftControl(); //Lift Control
		pros::delay(20);
	}
}
