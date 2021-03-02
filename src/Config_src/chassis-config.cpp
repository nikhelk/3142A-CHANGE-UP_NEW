#include "Util/vex.h"
#include "ChassisSystems/chassisGlobals.h"
#include "Util/literals.h"
#include "Impl/api.h"
#include "ChassisSystems/odometry.h"
#include "Config/other-config.h"
using namespace vex;

/// Our FourMotorDrive implementation. Inspiried by OkapiLib (c) Ryan Benesautti WPI
FourMotorDrive chassis = FourMotorDrive::FourMotorDriveBuilder{}
                          .withMotors({PORT2, PORT1}, {PORT3, PORT4})
                          .withGearSetting(ratio18_1)
                          .withGearRatio(1.6666667)
                          .withDimensions({12.0_in, 3.25_in, 26})
                          .withLinearLimits({1.2_mps, 1.9_mps2})
                          .withAngularLimits( {1.0_radps,3.0_radps2} )
                          .withPDGains( {
                                        {0, 0},  //Distance PD (deprecated thanks to feedforwards control)
                                        {0, 0},  //Angle PD (deprecated thanks to feedforwards control)
                                        {25, 65} //Turn PD (used for inertial sensor based turns))
                                                }) 
                          .buildChassis();

triport Expander19Enconder = triport(PORT19);
encoder leftEncoder = encoder(Expander19Enconder.E);
encoder rightEncoder = encoder(Brain.ThreeWirePort.A);
encoder backEncoder = encoder(Expander19Enconder.G);


line intakeDetect = line(Brain.ThreeWirePort.G);
encoder testEncoder = encoder(Brain.ThreeWirePort.A);

inertial inert = inertial(PORT9);
// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices in the chassis
 */
void initChassis(void)
{
  //right side of bot reversed and left is not
  chassis.setReverseSettings( {false, false} , {true, true} );
  // chassis.setReverseSettings({false, false}, {true, true});
  
  chassis.resetPosition();
  chassis.resetRotation();
  
  leftEncoder.resetRotation();
  rightEncoder.resetRotation();
  backEncoder.resetRotation();
  
  setOdomOrigin(0, 0, 0);

  //poseTracker.inert.calibrate();

  /*do {

    BigBrother.Screen.print("Calibrating Inert");
    task::sleep(200);

    BigBrother.Screen.clearLine(3);
  } while((poseTracker.inert.isCalibrating()) );

  

  BigBrother.Screen.print("DONE!");*/
}