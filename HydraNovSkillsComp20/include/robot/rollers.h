#ifndef _ROLLERS_H_
#define _ROLLERS_H_

#include "api.h"

void setState(int state);
  /*0 = Everything stopped
  * 1 = Front rollers in full speed, everything else stopped.
  * 2 = Front rollers out full speed, everything else stopped.
  * 3 = Front rollers in full speed, rollers rolling up to indexer
  * 4 = Everything out full speed
  * 5 = Everything in full speed
  * 6 = Front rollers out full speed, rollers in full speed
  * 7 = Front rollers opposite directions
  * 8 = Mid and front rollers stopped, Top roller in
  * 9 = Mid roller stopped, front rollers and top in
  * 10 = Top roller stopped, front and mid rollers out
  * 11 = Mid and Top rollers in, front rollers stopped
  * 12 = Mode 3 but without front rollers
  * 13 = Scores, delays the front rollers
  * 14 = Intakes until it sees a red ball, at which point it spins the intakes out and stops intaking.
  * 15 = Mid and Top rollers in, Intakes until it sees a red ball, at which point it spins the intakes out and stops intaking.
  * 16 = Front rollers stopped, rollers rolling up to indexer
  */

void setIntakeBrakes(pros::motor_brake_mode_e_t mode);

void initIntakes();

void intakeOp();

#endif
