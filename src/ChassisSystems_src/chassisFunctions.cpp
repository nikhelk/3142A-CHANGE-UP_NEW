#include "ChassisSystems/chassisGlobals.h"
#include "Impl/api.h"
FourMotorDrive::moveToPoint(double x2, double y2, double heading) {
  int ballShot = 0;
  int maxTime = 0;



  while (true)
  {
    // run the intake indexing procedure if commanded to do so
    // if any intake / conveying function is called
 
      // as long as max time for descore is not exceeded


    /*
    The below procedure translates the robot from point x1 and y1 (its current)
    location on the field to point x2 and y2 while adjusting the current robot
    heading to the target robot heading
    */

    // current robot x and y position is given by odometry (global variables)
    long double x1 = positionArray[ODOM_X};
    long double y1 = positionArray[ODOM_Y];

    // step 1, using x1, y1, x2, y2, and heading, let us calculate the STANDARD FORM equation of the lines
    // this can be done by simply finding a second pair of coordinates, as two points define a line

    // how far away in euclidean distance the second point we want is, can be
    // any arbitrary number
    const long double l = 100.0;

    // [ROBOT POSITION POINT] the first line is defined by x1 and y1, and the
    // heading—we now use this to define a second point on the line
    long double x1Other = x1 + l * sin(positionArray[ODOM_THETA]);
    long double y1Other = y1 + l * cos(ositionArray[ODOM_THETA]);

    // [ROBOT POSITION LINE PARALLEL TO HEADING (line NT)]
    // reversed because we have to assign negative sign to either numerator
    // or denominator
    long double A1 = y1 - y1Other;
    long double B1 = x1Other - x1;
    long double C1 = -(A1 * x1 + B1 * y1);

    // [ROBOT DESTINATION LINE PERPENDICULAR PREVIOUS LINE (line MT)]
    long double A2 = B1;
    long double B2 = -A1;
    long double C2 = -(A2 * x2 + B2 * y2);

    // with the STANDARD FORM equations for both lines, let us calculate the
    // intersection point of the two; let (x, y) denote the coordinate
    // intersection of the two lines, represented in STANDARD FORM
    long double x = (B1 * C2 - B2 * C1) / (B2 * A1 - B1 * A2);
    long double y = (A1 * C2 - A2 * C1) / (A2 * B1 - A1 * B2);

    // since Euclidean distance is always a positive value, we must know the direction in which the error is
    // to do this, we need to define another line
    // LINE: going through x1, y1, heading rotated 90 degrees anticlockwise

    // define another point on the line perpendicular to line NT in calculations
    long double x3Other = x1 + l * sin(heading - M_PI / 2.0);
    long double y3Other = y1 + l * cos(heading - M_PI / 2.0);

    // with this information, we can now determine the signage of the vertical
    // and horizontal components (relative to heading)
    int horizontalSign = 1;
    int verticalSign = 1;

    // determine the relative direction of the horizontal component
    long double d1 = (x2 - x1) * (y1Other - y1) - (y2 - y1) * (x1Other - x1);
    if (d1 < 0)
      horizontalSign = -1;
    cout << "D1: " << d1 << '\n';

    // same for the vertical component
    long double d2 = (x - x1) * (y3Other - y1) - (y - y1) * (x3Other - x1);
    if (d2 < 0)
      verticalSign = -1;
    cout << "D2: " << d2 << '\n';

    // with the intersection point and direction, we can now calculate the
    // Euclidean distance forwards relative to the robot's direction and
    // horizontally relative to the robot heading to get our x and y components
    // (errors) for our overall translation vector
    long double yComponent = abs(sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1))) * verticalSign;
    long double xComponent = abs(sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2))) * horizontalSign;
    cout << "COMPONENT VECTORS: " << yComponent << ' ' << xComponent << '\n';

    // to send this information to the motors, we pass the original encoder
    // position, and the destination encoder position plus their respective
    // local components for the PID loop, and we use ticks to inch conversion
    // as function arguments are in inches
    long double leftDest = leftEncoder.position(degrees) + yComponent * (360 / (2.75 * pi));
    long double rightDest =rightEncoder.position(degrees) + yComponent * (360 / (2.75 * pi));
    long double horizontalDest = backEncoder.position(degrees) + xComponent * (360 / (2.75 * pi));
    cout << "DESTINATIONS: " << leftDest << ' ' << rightDest << ' ' << horizontalDest << '\n';

    // begin PID controllers

    // [y component] forward backward (relative to heading) voltage output
    long double yVoltageLeft = leftdrivebasePIDController->update(leftDest, verticalEncoder1.get_value(), -1);
    long double yVoltageRight = rightdrivebasePIDController->update(rightDest, verticalEncoder2.get_value(), -1);
    cout << "FB Volt: " << yVoltageLeft << ' ' << yVoltageRight << '\n';

    // [x component] side to side (relative to heading) voltage output
    long double xVoltage = strafePIDController->update(horizontalDest, horizontalEncoder.get_value(), -1);
    cout << "H Volt: " << xVoltage << '\n';
    pros::lcd::set_text(5, "lDest: " + to_string(leftDest));
    pros::lcd::set_text(6, "rDest: " + to_string(rightDest));
    pros::lcd::set_text(7, "hDest: " + to_string(horizontalDest));

    // [angle] angle heading correction with voltage output
    long double angleVoltageLeft = turningPIDController->update(heading, lastAngle, -1);
    long double angleVoltageRight = -turningPIDController->update(heading, lastAngle, -1);
    cout << "Ang Volt: " << angleVoltageLeft << ' ' << angleVoltageRight << '\n';

    /*
    To allow for a smoother acceleration gradient, time is considered in
    scaling the voltage outputs. So, we limit speed the robot can travel in the
    first accelerationTime ms of acceleration. accelerationTime, like other
    tunable constants, is adjusted in globals.cpp.
    */
    long double currentTime = pros::millis() - time;
    long double coefTime = 1.0;

    if (currentTime < accelerationTime)
    {
      // based off of the square root curve approaching the target voltage
      coefTime = sqrt(currentTime) / sqrt(accelerationTime);
    }

    // the left side motors are positive power for forwards movement and the
    // right side motors are negative power for forwards movement
    long double finalVoltageLeftFront = (yVoltageLeft + xVoltage + angleVoltageLeft) * coefTime;
    long double finalVoltageLeftBack = (yVoltageLeft - xVoltage + angleVoltageLeft) * coefTime;
    long double finalVoltageRightFront = -(yVoltageRight - xVoltage + angleVoltageRight) * coefTime;
    long double finalVoltageRightBack = -(yVoltageRight + xVoltage + angleVoltageRight) * coefTime;

    // prevent exceeding the MAX_VOLTAGE value by scaling the motors accordingly
    long double maxOutput = max({finalVoltageLeftFront, finalVoltageLeftBack, finalVoltageRightFront, finalVoltageRightBack});
    if (maxOutput > maxVolt)
    {
      // scale down all of the motor outputs by a fraction >= 0 < 1 such that
      // the max voltage does not exceed MAX_VOLTAGE
      long double scaling = maxVolt / maxOutput;
      finalVoltageLeftFront *= scaling;
      finalVoltageLeftBack *= scaling;
      finalVoltageRightFront *= scaling;
      finalVoltageRightBack *= scaling;
    }

    cout << "FINAL OUTPUT: " << finalVoltageLeftFront << ' ' << finalVoltageLeftBack << ' ' << finalVoltageRightFront << ' ' << finalVoltageRightBack << '\n';

    // stop the robot if the maximum allocated time is exceeded
    if (pros::millis() - time >= timeAllocated)
    {
      cout << "PID STOPPED" << '\n';
      leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

      leftFront.move_velocity(0);
      rightFront.move_velocity(0);
      leftBack.move_velocity(0);
      rightBack.move_velocity(0);

      // stop the indexing and intakes
      leftIntake.move_velocity(0);
      rightIntake.move_velocity(0);
      indexer.move_velocity(0);
      shooter.move_velocity(0);
      return;
    }

    // move motors with final voltages
    leftFront.move_voltage(finalVoltageLeftFront);
    leftBack.move_voltage(finalVoltageLeftBack);
    rightFront.move_voltage(finalVoltageRightFront);
    rightBack.move_voltage(finalVoltageRightBack);
    pros::delay(20);
  }

  return;
}