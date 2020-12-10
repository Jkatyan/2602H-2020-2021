#ifndef INITIALIZE_HPP
#define INITIALIZE_HPP

#include "main.h"
#include "screen/field.hpp"
#include "screen/resources.hpp"

extern int setAuton;

int color = 0;
int colorLock = 0;
int auton = 0;
int xA = 0;
int setAuton = 0;
int selected = 0;
 int tileVar = 0; //Auton Selector Variables

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

#endif
