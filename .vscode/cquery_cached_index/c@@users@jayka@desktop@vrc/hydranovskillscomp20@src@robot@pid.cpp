#include "main.h"

using namespace okapi;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

PID::PID(double kP, double kD, int minDt) :
m_kP(kP), m_kD(kD), m_minDt(minDt) {
  m_lastTime = m_timer.millis().convert(millisecond);
}

double PID::calculateErr(double ierror) {
  m_error = ierror;

  //calculate delta time
  double dT = m_timer.millis().convert(millisecond) - m_lastTime;
  //abort if dt is too small
  if(dT < m_minDt) return m_output;

  //calculate derivative
  m_derivative = (m_error - m_lastError) / dT;

  //calculate output
  m_output = (m_error * m_kP) + (m_derivative * m_kD);
  //limit output
  if(std::abs(m_output) > 127) m_output = sgn(m_output) * 127;

  //save values
  m_lastTime = m_timer.millis().convert(millisecond);
  m_lastError = m_error;

  return m_output;
}

double PID::calculate(double target, double current) {
  return calculateErr(target - current);
}

double PID::getError() {
  return m_error;
}

void PID::reset() {
  m_error = 0;
  m_lastError = 0;
  m_lastTime = m_timer.millis().convert(millisecond);
  m_derivative = 0;
  m_output = 0;
}
