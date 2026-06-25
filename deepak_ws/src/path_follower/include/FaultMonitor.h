/*
 * Name:        FaultMonitor.h
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-12
 * Version:     1.0
 * Description: Declares the FaultMonitor class that monitors the AGV's safety states 
 *              including sensor dropout, motor saturation, line loss, and timeouts.
 */

#ifndef FAULT_MONITOR_H
#define FAULT_MONITOR_H

#include <string>
#include <chrono>

namespace path_follower {

class FaultMonitor {
public:
    FaultMonitor(int lineLostGraceSteps, int maxFrozenSteps);
    ~FaultMonitor() = default;

    // Disable copy/move constructors for safety
    FaultMonitor(const FaultMonitor&) = delete;
    FaultMonitor& operator=(const FaultMonitor&) = delete;

    void update(double trackPos, bool trackDetect, double leftRpm, double rightRpm, double maxRpm);
    void checkTimeout(const std::chrono::steady_clock::time_point& lastUpdateTime);
    void injectEStop(bool active);
    void reset();
    void setLineLostGraceSteps(int lineLostGraceSteps);
    void setMaxFrozenSteps(int maxFrozenSteps);

    bool hasFault() const;
    std::string getFaultType() const;
    std::string getFaultLog(const std::string& currentState) const;

private:
    // Constants & Configuration
    int m_lineLostGraceSteps;
    int m_maxFrozenSteps;

    // Safety State Variables
    bool m_hasFault;
    std::string m_faultType;
    bool m_estopActive;

    // Sensor dropout tracking
    double m_lastTrackPos;
    int m_frozenStepsCount;

    // Line loss tracking
    int m_lineLostCount;
};

} // namespace path_follower

#endif // FAULT_MONITOR_H
