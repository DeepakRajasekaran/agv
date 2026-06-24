/*
Name: PidController.cpp
Author: ANSCER Robotics
Date: 2026-06-24
Version: 2.0
Description: PID controller with diagnostics.
*/

#include "line_follower/PidController.h"

namespace line_follower
{

PidController::PidController()
: m_p(0), m_i(0), m_d(0), m_iMax(0), m_iMin(0),
  m_integral(0), m_prevError(0),
  m_lastPTerm(0), m_lastITerm(0), m_lastDTerm(0)
{}

PidController::~PidController() {}

void PidController::init(double p, double i, double d, double iMin, double iMax)
{
  m_p = p; m_i = i; m_d = d; m_iMin = iMin; m_iMax = iMax;
  reset();
}

void PidController::setGains(double p, double i, double d)
{
  m_p = p; m_i = i; m_d = d;
}

void PidController::reset()
{
  m_integral = 0; m_prevError = 0;
  m_lastPTerm = 0; m_lastITerm = 0; m_lastDTerm = 0;
}

double PidController::compute(double error, double dt)
{
  if (dt <= 0.0) return 0.0;

  m_lastPTerm = m_p * error;

  m_integral += error * dt;
  m_integral = std::clamp(m_integral, m_iMin, m_iMax);
  m_lastITerm = m_i * m_integral;

  m_lastDTerm = m_d * (error - m_prevError) / dt;
  m_prevError = error;

  return m_lastPTerm + m_lastITerm + m_lastDTerm;
}

} // namespace line_follower
