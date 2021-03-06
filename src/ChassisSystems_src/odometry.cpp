#include "ChassisSystems/odometry.h"
#include "ChassisSystems/chassisGlobals.h"
#include "Util/mathAndConstants.h"
#include <cmath>


/*
Copyright (c) 2018 5225A E-bot PiLons
Modifications nikhelkrishna
2020-31-7: Modify constants for bot use

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


/// WE WOULD LIKE TO THANK 5225A FOR SHARING AND EXPLAINING THEIR ODOM SYSTEM AND CODE
int trackPosition()
{
  sPos position;
  position.leftLst = 0;
  position.rightLst = 0;
  position.backLst = 0;
  position.x = positionArray[ODOM_X];
  position.y = positionArray[ODOM_Y];
  position.a = positionArray[ODOM_THETA] * (M_PI / 180);
  while (true)
  {
    int left = leftEncoder.position(degrees);
    int right = rightEncoder.position(degrees); 
    int back = backEncoder.position(degrees);;
    double deltaL = (left - position.leftLst) * SPIN_TO_IN_LR ; // The amount the left side of the robot moved
    double deltaR = -1* (right - position.rightLst) * SPIN_TO_IN_LR; // The amount the right side of the robot moved (reversed)
    double deltaB = (back - position.backLst) * SPIN_TO_IN_S;                // The amount the back side of the robot moved

    // Update the last values
    position.leftLst = left;
    position.rightLst = right;
    position.backLst = back;
    double h;                                                       // The hypotenuse of the triangle formed by the middle of the robot on the starting position and ending position and the middle of the circle it travels around
    double i;                                                       // Half on the angle that I've traveled
    double h2;                                                      // The same as h but using the back instead of the side wheels
    double a = (deltaL - deltaR) / (L_DISTANCE_IN + R_DISTANCE_IN); // The angle that I've traveled
    if (a)
    {
      double r = deltaR / a; // The radius of the circle the robot travel's around with the right side of the robot
      i = a / 2.0;
      double sinI = sin(i);
      h = ((r + L_DISTANCE_IN) * sinI) * 2.0;

      double r2 = deltaB / a; // The radius of the circle the robot travel's around with the back of the robot
      h2 = ((r2 + S_DISTANCE_IN) * sinI) * 2.0;
    }
    else
    {
      h = deltaR;
      i = 0;

      h2 = deltaB;
    }
    double p = i + inert.rotation()*M_PI/180; // The global ending angle of the robot
    double cosP = cos(p);
    double sinP = sin(p);

    // conversion from polar to cartesian
    position.y += h * cosP;
    position.x += h * sinP;

    position.x += h2 * cosP;
    position.y += h2 * -sinP;

    //position.a += a;

    //cout << "hi" <<SPIN_TO_IN_LR <<endl;
    positionArray[ODOM_X] = position.x;
    positionArray[ODOM_Y] = position.y;
    positionArray[ODOM_THETA] = inert.rotation()*M_PI/180;
    //std::cout << "ODOM: " <<  positionArray[ODOM_X] << " " << positionArray[ODOM_Y] << " " <<positionArray[ODOM_THETA]*180/M_PI <<std::endl;
    //cout << positionArray[ODOM_X] << "," << positionArray[ODOM_Y] << " " <<positionArray[ODOM_THETA] <<" " << left<< " " <<right<<endl;
    //cout << positionArray[ODOM_X] << "," << positionArray[ODOM_Y] <<endl;;
    //cout << positionArray[ODOM_X] << "," << positionArray[ODOM_Y] << " " << positionArray[ODOM_THETA] << " " << leftFront.position(degrees)<< " " << rightFront.position(degrees)<< " "<<
    //    (leftFront.position(degrees)/rightFront.position(degrees)) <<endl;
    task::sleep(15);
  }
  return 1;
}


void setOdomOrigin(double x, double y, double a)
{
  positionArray[ODOM_X] = x;
  positionArray[ODOM_Y] = y;
  positionArray[ODOM_THETA] = a;
}

void printPosition()
{
  std::cout << positionArray[ODOM_X] << " "
            << positionArray[ODOM_Y] << " " << positionArray[ODOM_THETA] << std::endl;
}




/*
Copyright (c) 2016 BCI Module
Modifications nikhelkrishna
2020-31-7: Modify constants for bot use

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


void computeDistanceAndAngleToPoint(const double x, const double y, pointVals *out)
{

  //Compute difference in distance

  const double xDiff = x - positionArray[ODOM_X], yDiff = y - positionArray[ODOM_Y];
  out->length = sqrt((xDiff * xDiff) + (yDiff * yDiff));

  //Compute difference in angle
  out->theta = ((atan2(yDiff, xDiff) * (180 / M_PI))) - positionArray[ODOM_THETA];
}

