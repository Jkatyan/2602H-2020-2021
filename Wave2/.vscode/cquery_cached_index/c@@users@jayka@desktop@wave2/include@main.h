#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"
#include "okapi/api.hpp"

// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;

#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */

#include "wave/auton.hpp"
#include "wave/config.hpp"
#include "wave/globals.hpp"
#include "wave/selection.hpp"
#include "wave/PID.hpp"
#include "wave/odom.hpp"

#endif

#endif
