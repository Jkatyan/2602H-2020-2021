#include "main.h"

Controller AokapiMaster = ControllerId::master;
Controller AokapiPartner = ControllerId::partner;

Motor AokapiLF = 20_rmtr;
Motor AokapiLB = 10_mtr;
Motor AokapiRF = 1_rmtr;
Motor AokapiRB = 3_mtr;

//Drive Control
ChassisControllerIntegrated Adrive = ChassisControllerFactory::create(
	{AokapiLF, AokapiLB},
	{AokapiRF, AokapiRB},
	AbstractMotor::gearset::green,
	{4_in, 14_in}
);
