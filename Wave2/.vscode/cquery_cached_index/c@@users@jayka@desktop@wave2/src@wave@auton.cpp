#include "main.h"

void skills() {}

void redRight() {}

void redLeft() {}

void blueRight() {}

void blueLeft() {}

void nothing(){
  pros::delay(60000);
}

void runAuton(int auton){
  switch(auton){
    case 0: //Skills
      skills();
    break;
    case 1: //Red Right
      redRight();
    break;
    case 2: //Red Left
      redLeft();
    break;
    case 3: //Do Nothing
      nothing();
    break;
    case -1: //Blue Right
      blueRight();
    break;
    case -2: //Blue Left
      blueLeft();
    break;
    case -3: //Do Nothing
      nothing();
    break;
  }
}
