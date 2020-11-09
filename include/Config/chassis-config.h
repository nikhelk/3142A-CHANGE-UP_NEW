#include "ChassisSystems/chassisGlobals.h"

using namespace vex;

extern FourMotorDrive testchassis;
extern FourMotorDrive chassis;

// VEXcode devices
extern encoder testEncoder;
extern line intakeDetect;
extern brain Brain;
extern encoder leftEncoder;
extern encoder rightEncoder;
extern encoder backEncoder;

/**
 * Used to initialize code/tasks/devices in the chassis
 */
void initChassis(void);