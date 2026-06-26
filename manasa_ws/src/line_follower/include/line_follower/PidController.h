/*
Name: PidController.h
Author: Manasa
Date: 2026-06-24
Version: 2.0
Description: PID controller with diagnostic getters and runtime gain adjustment.
*/

#ifndef LINE_FOLLOWER__PID_CONTROLLER_H_
#define LINE_FOLLOWER__PID_CONTROLLER_H_

#include <algorithm>

namespace line_follower
{

class PidController
{
public:
  PidController();
  ~PidController();

  void init(double p, double i, double d, double iMin, double iMax);
  void setGains(double p, double i, double d);
  void reset();
  double compute(double error, double dt);

  // Diagnostic getters
  double getLastPTerm() const { return m_lastPTerm; }
  double getLastITerm() const { return m_lastITerm; }
  double getLastDTerm() const { return m_lastDTerm; }
  double getLastError() const { return m_prevError; }
  double getIntegral() const { return m_integral; }

private:
  double m_p, m_i, m_d;
  double m_iMax, m_iMin;
  double m_integral;
  double m_prevError;
  double m_lastPTerm, m_lastITerm, m_lastDTerm;
};

} // namespace line_follower
#endif
