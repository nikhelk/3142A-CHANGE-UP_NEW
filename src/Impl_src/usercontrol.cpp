#include "Impl/usercontrol.h"
#include "Impl/api.h"

void unlockRobot() {
  Flywheel.spin(forward,6,volt);
  task::sleep(1000);
  Flywheel.stop();
}

double cubic(double input) { 

  //Used to ramp the turning speed, for finer control at lower speeds.

  if (input != 0) {return (pow(input,3))/(100*std::abs(input));}
  else {return 0;}
}

void usercontrol() {

  unlockRobot();
  while(true) {

  //DRIVING
    double throttle = BigBrother.Axis3.value();
    double strafe = BigBrother.Axis4.value();
    double turn = cubic(BigBrother.Axis1.value());

    if(std::abs((throttle+strafe+turn)) <5) {
      chassis.leftFront.spin(directionType::fwd, 0, velocityUnits::pct);
    }
    else {
      chassis.leftFront.spin(directionType::fwd, (throttle+strafe+turn), velocityUnits::pct);
    }

    if(std::abs((throttle-strafe+turn)) <5) {
      chassis.leftBack.spin(directionType::fwd, 0, velocityUnits::pct);
    }
    else {
      chassis.leftBack.spin(directionType::fwd, (throttle-strafe+turn), velocityUnits::pct);
    }

    if(std::abs((throttle+strafe-turn)) <5) {
      chassis.rightBack.spin(directionType::fwd, 0, velocityUnits::pct);
    }
    else {
      chassis.rightBack.spin(directionType::fwd, (throttle+strafe-turn), velocityUnits::pct);
    }

    if(std::abs((throttle-strafe-turn)) <5) {
      chassis.rightFront.spin(directionType::fwd, 0, velocityUnits::pct);
    }
    else {
      chassis.rightFront.spin(directionType::fwd, (throttle-strafe-turn), velocityUnits::pct);
    }

    // intake
    if(BigBrother.ButtonL1.pressing()&& !BigBrother.ButtonL2.pressing())
    {
      IntakeL.spin(fwd,600,rpm);
      IntakeR.spin(fwd,600,rpm);
    }

    // outtake
    else if (BigBrother.ButtonL2.pressing()&& !BigBrother.ButtonL1.pressing())
    {
      IntakeL.spin(fwd,-600,rpm);
      IntakeR.spin(fwd,-600,rpm);
    }

    // stop intakes
    else
    {
      IntakeL.spin(fwd,0,rpm);
      IntakeR.spin(fwd,0,rpm);
    }


    // indexing controls (R1 indexes the first ball to the shooting bay and the
    // remaning accordingly)

    // we know the ball is indexed if the line tracker reports <= INDEX_THRESHOLD
    if (BigBrother.ButtonR1.pressing())
    {
      Indexer.spin(fwd,500,rpm);
    }

    else if(BigBrother.ButtonB.pressing()) {
      Indexer.spin(fwd,-300,rpm);
    }

    else
    {
      Indexer.spin(fwd,0,rpm);

    }

    // shooting controls (R2 fires balls)
    if (BigBrother.ButtonR2.pressing())
    {
      // shoots indexed ball
      Flywheel.spin(fwd,500,rpm);
    }
    
    else if (BigBrother.ButtonL1.pressing() && BigBrother.ButtonL2.pressing())
    {
      Flywheel.spin(fwd,-300,rpm);
    }

    else
    {
      Flywheel.spin(fwd,0,rpm);
    }

    // discarding controls



  //INTAKES
  //   if(BigBrother.ButtonR1.pressing()){
  //     IntakeL.spin(fwd,12,volt);
  //     IntakeR.spin(fwd,12,volt); 
  //     Indexer.spin(fwd,12,volt);                                                                 
  //   }
  //   else if(BigBrother.ButtonR2.pressing()){
  //     IntakeL.spin(reverse,12,volt);
  //     IntakeR.spin(reverse,12,volt);
  //     Indexer.spin(reverse,12,volt);
  //   }
  //   else{
  //     IntakeL.stop();
  //     IntakeR.stop();
  //     Indexer.stop();
  //   }
  
  // //INDEXER
  //   if(BigBrother.ButtonR1.pressing()==false) {
  //   if(BigBrother.ButtonL1.pressing()){
  //     Indexer.spin(fwd,12,volt);
  //   }
  //   else if(BigBrother.ButtonL2.pressing()){
  //     Indexer.spin(reverse,12,volt);
  //   }
  //   else {
  //     Indexer.stop();
  //   }
  //   }


  // //FLYWHEEL
  //   if(BigBrother.ButtonA.pressing()) {
  //     Flywheel.spin(forward,12,volt);
  //   }
  //   else if(BigBrother.ButtonB.pressing()) {
  //     Flywheel.spin(reverse,12,volt);
  //   }
  //   else{
  //     Flywheel.stop();
  //   }

  task::sleep(20);
  
  }
}