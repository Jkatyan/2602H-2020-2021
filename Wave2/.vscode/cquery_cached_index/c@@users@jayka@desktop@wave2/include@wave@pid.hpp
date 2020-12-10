#ifndef _PID_HPP_
#define _PID_HPP_

#include "okapi/api.hpp"

class PID {
  private:
    double m_kP = 0;
    double m_kD = 0;
    int m_minDt = 10;

    okapi::Timer m_timer;
    double m_error = 0;
    double m_lastError = 0;
    double m_lastTime = 0;
    double m_derivative = 0;
    double m_output = 0;

  public:
    PID(double kP, double kD, int minDt = 10);
    double calculateErr(double);
    double calculate(double, double);
    double getError();
    void reset();
};

#endif
