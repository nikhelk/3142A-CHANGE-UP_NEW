/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       nikhelkrishna                                             */
/*    Created:      Mon May 18 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "Impl/api.h"
#include "Impl/auto_skills.h"
#include "Impl/usercontrol.h"
#include "Impl/match_auton.h"
#include "Util/vex.h"
#include "ChassisSystems/posPID.h"
using namespace vex;

void pre_auto(void) {

  initChassis(); //initalizing chassis (see Config_src/chassis-config.cpp)

  Brain.Screen.pressed( selector3142a::userTouchCallbackPressed ); // set up callback for brain screen press
  Brain.Screen.released( selector3142a::userTouchCallbackReleased ); // set up callback for brain screen release

  // make background
  selector3142a::makeBackground();

  // auton selector task
  task autonSelect( selector3142a::makeDisplay );

}

void autonomous(void) {
  if(!selector3142a::allianceBlue) {
    redAuton();
  }
  else {
    blueAuton();
  }
}

int main() {

  pre_auto();

  //BigBrother.ButtonA.pressed( runAutoSkills ); //Run autonomous skills when button "A" is pressed on controller
  //usercontrol();
  task::sleep(10000);
  autonomous();
  while (true) {
    
   // LOG(leftEncoder.position(degrees),rightEncoder.position(degrees));
    this_thread::sleep_for(20);
  }
}
