/*
 * Name:        FaultMonitor.cpp
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-12
 * Version:     1.0
 * Description: Implements the FaultMonitor class for managing safety states and logs.
 */

#include "FaultMonitor.h"
#include <cassert>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace path_follower {

/**
 * @brief  Constructor initializing the safety parameters and state variables.
 * @param  lostThreshold  Maximum lateral deviation allowed before line loss detection (meters).
 * @param  maxFrozenSteps Number of consecutive cycles with identical values triggering sensor dropout.
 */
FaultMonitor::FaultMonitor(double lostThreshold, int maxFrozenSteps)
    : m_lostThreshold(lostThreshold),
      m_maxFrozenSteps(maxFrozenSteps),
      m_hasFault(false),
      m_faultType("NONE"),
      m_estopActive(false),
      m_lastTrackPos(0.0),
      m_frozenStepsCount(0),
      m_lineLostTimerActive(false)
{
    // Assert parameters are positive and valid
    assert(lostThreshold > 0.0);
    assert(maxFrozenSteps > 0);
}

/**
 * @brief  Updates the safety checks for line loss, sensor freeze, and motor saturation.
 * @param  trackPos     Current track lateral position value.
 * @param  trackDetect  True if line track is physically detected.
 * @param  leftRpm      Command speed of the left wheel in RPM.
 * @param  rightRpm     Command speed of the right wheel in RPM.
 * @param  maxRpm       Maximum RPM capacity before saturation warnings.
 */
void FaultMonitor::update(double trackPos, bool trackDetect, double leftRpm, double rightRpm, double maxRpm)
{
    // Preconditions
    assert(std::isfinite(trackPos));
    assert(std::isfinite(leftRpm));
    assert(std::isfinite(rightRpm));

    if (m_estopActive) {
        m_hasFault = true;
        m_faultType = "E_STOP";
        return;
    }

    // 1. Check Sensor Dropout (Frozen error signal)
    if (trackDetect) {
        if (std::abs(trackPos - m_lastTrackPos) < 1e-7) {
            m_frozenStepsCount++;
        } else {
            m_frozenStepsCount = 0;
        }
        m_lastTrackPos = trackPos;
    } else {
        m_frozenStepsCount = 0;
    }

    if (m_frozenStepsCount > m_maxFrozenSteps) {
        m_hasFault = true;
        m_faultType = "SENSOR_DROPOUT";
    }

    // 2. Check Line Lost (deviation exceeding threshold or track lost -> stop immediately)
    bool outOfBounds = std::abs(trackPos) > m_lostThreshold;
    if (outOfBounds || !trackDetect) {
        m_hasFault = true;
        m_faultType = "LINE_LOST";
    }

    // 3. Check Motor Saturation
    if (std::abs(leftRpm) > maxRpm || std::abs(rightRpm) > maxRpm) {
        m_hasFault = true;
        m_faultType = "MOTOR_SATURATION";
    }

    // Postconditions
    assert(m_frozenStepsCount >= 0);
}

/**
 * @brief  Checks for update timeout between consecutive simulator updates.
 * @param  lastUpdateTime  Time point of the last update received.
 */
void FaultMonitor::checkTimeout(const std::chrono::steady_clock::time_point& lastUpdateTime)
{
    // Precondition: Time point must not be uninitialized
    assert(lastUpdateTime.time_since_epoch().count() >= 0);

    if (m_estopActive) {
        m_hasFault = true;
        m_faultType = "E_STOP";
        return;
    }

    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUpdateTime).count();

    if (duration > 500) {  // 500 ms timeout
        m_hasFault = true;
        m_faultType = "TIMEOUT";
    }
}

/**
 * @brief  Sets or clears external E-stop fault.
 * @param  active  True to trigger E-stop.
 */
void FaultMonitor::injectEStop(bool active)
{
    m_estopActive = active;
    if (active) {
        m_hasFault = true;
        m_faultType = "E_STOP";
    }
}

/**
 * @brief  Resets safety monitor state.
 */
void FaultMonitor::reset()
{
    m_hasFault = false;
    m_faultType = "NONE";
    m_estopActive = false;
    m_frozenStepsCount = 0;
    m_lineLostTimerActive = false;
    
    // Postcondition
    assert(!m_hasFault);
}

/**
 * @brief  Checks if any fault is active.
 * @return True if in fault state.
 */
bool FaultMonitor::hasFault() const
{
    return m_hasFault;
}

/**
 * @brief  Gets current active fault type.
 * @return String description of fault.
 */
std::string FaultMonitor::getFaultType() const
{
    return m_faultType;
}

/**
 * @brief  Formats and returns structured log entry of fault state.
 * @param  currentState String representation of the current state.
 * @return Formatted log string.
 */
std::string FaultMonitor::getFaultLog(const std::string& currentState) const
{
    // Precondition
    assert(!currentState.empty());

    auto now = std::chrono::system_clock::now();
    auto timeT = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&timeT), "%Y-%m-%d %H:%M:%S")
       << " | " << m_faultType
       << " | STATE: " << currentState
       << " | SENSOR_VALS: last_err=" << m_lastTrackPos;
       
    return ss.str();
}

void FaultMonitor::setLostThreshold(double lostThreshold)
{
    assert(lostThreshold > 0.0);
    m_lostThreshold = lostThreshold;
}

void FaultMonitor::setMaxFrozenSteps(int maxFrozenSteps)
{
    assert(maxFrozenSteps > 0);
    m_maxFrozenSteps = maxFrozenSteps;
}

} // namespace path_follower
