#include "ChassisSystems/posPID.h"
#include "Util/premacros.h"
posPID::posPID() {}

posPID::posPID(double kP, double kD) : m_kP(kP), m_kD(kD) {}

void posPID::setPD(double kP, double kD) {
  m_kP = kP;
  m_kD = kD;
}

double posPID::calculatePower(double targetPos, double currentPos) {

  m_error = targetPos - currentPos;

  m_derivative = m_error - m_prevError;


  m_power = (m_error * m_kP) + (m_derivative * m_kD); // error*kP + deltaError*kD


  //limit our voltage to the V5 constraints
  if (m_power > m_upperBound) {
    m_power = m_upperBound;
  }

  else if (m_power < m_lowerBound) {
    m_power = m_lowerBound;
  }

  m_prevError = m_error;
  return (m_power);
}

double posPID::calculatePower(double error) {

  m_error = error;

  m_derivative = m_error - m_prevError;


  m_power = (m_error * m_kP) + (m_derivative * m_kD); // error*kP + deltaError*kD


  //limit our voltage to the V5 constraints
  if (m_power > m_upperBound) {
    m_power = m_upperBound;
  }

  else if (m_power < m_lowerBound) {
    m_power = m_lowerBound;
  }

  m_prevError = m_error;
  return (m_power);
}