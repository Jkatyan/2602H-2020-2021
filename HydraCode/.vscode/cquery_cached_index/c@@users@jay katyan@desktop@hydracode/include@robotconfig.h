#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#include "okapi/api.hpp"
using namespace okapi;

extern Controller okapiMaster;
extern Controller okapiPartner;

//Drive Motor
extern Motor okapiLF;
extern Motor okapiRF;
extern Motor okapiLB;
extern Motor okapiRB;

//Okapi controllers
extern ChassisControllerIntegrated drive;

#endif
