#include "Impl/match_auton.h"
#include "Impl/api.h"


void unlockRobotAuton() {
  Flywheel.spin(forward,6,volt);
  task::sleep(1000);
  Flywheel.stop();
}

void redAuton() {
  LOG("Running Red Auton!");
  if(!selector3142a::skills) {
    LOG("dumping in middle goal!");
    unlockRobotAuton();
    task::sleep(400);

    Flywheel.spin(fwd,12,volt);
    Indexer.spin(fwd,10,volt);
    task::sleep(4000);

    chassis.leftFront.spin(reverse,100,rpm);
    chassis.leftBack.spin(reverse,100,rpm);
    chassis.rightBack.spin(reverse,100,rpm);
    chassis.rightFront.spin(reverse,100,rpm);
    task::sleep(3000);
    chassis.leftFront.spin(fwd,100,rpm);
    chassis.leftBack.spin(fwd,100,rpm);
    chassis.rightBack.spin(fwd,100,rpm);
    chassis.rightFront.spin(fwd,100,rpm);
    task::sleep(2000);
    chassis.setDrive(0, 0, 0, 0);
  }

}


void blueAuton() {

  if(!selector3142a::skills) {

    unlockRobotAuton();
    task::sleep(400);

    Flywheel.spin(fwd,12,volt);
    Indexer.spin(fwd,10,volt);
    task::sleep(4000);
;
    chassis.setDrive(-6, -6, -6 , -6 );
    task::sleep(3000);
    chassis.setDrive(6, 6, 6 , 6 );
    task::sleep(2000);
    chassis.setDrive(0, 0, 0, 0);

  }
}