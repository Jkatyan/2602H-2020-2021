#include "main.h"
#include "autonSel.h"
#include "screen/resources.hpp"
#include "screen/field.hpp"

extern pros::ADIGyro gyro;

int color = 0;
int colorLock = 0;
int auton = 0;
int xA = 0;
int setAuton = 0; //Extern
int tileVar = 0;
int selected = 0;

lv_obj_t * myButton;
lv_obj_t * myButtonLabel;
lv_obj_t * myLabel;

lv_obj_t * myButton2;
lv_obj_t * myButtonLabel2;
lv_obj_t * myLabel2;

lv_obj_t * myButton3;
lv_obj_t * myButtonLabel3;
lv_obj_t * myLabel3;

lv_obj_t * myButton4;
lv_obj_t * myButtonLabel4;
lv_obj_t * myLabel4;

lv_obj_t * myLabel5;

lv_style_t myButtonStyleREL;
lv_style_t myButtonStylePR;

//extern const lv_img_t screenBack;

static lv_res_t btn_click_action(lv_obj_t * btn)
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

void initialize()
{
pros::ADIGyro gyro ('H');

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor lB(L2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor lF(L1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rB(R2, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rF(R1, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor door(DOOR, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor lift(LIFT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor diag(INTA, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor vert(INTB, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

}
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
  pros::delay(10);
  }
  if(selected != 0){
  //  lv_obj_clean(lv_scr_act());

  }
}
