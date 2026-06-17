/*
 * Name:        NavigationStateMachine.h
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-12
 * Version:     1.0
 * Description: Declares the NavigationStateMachine class to manage AGV navigation states.
 */

#ifndef NAVIGATION_STATE_MACHINE_H
#define NAVIGATION_STATE_MACHINE_H

#include <string>

namespace line_follower {

enum class State {
    IDLE,
    INITIALIZE,
    FOLLOW_LINE,
    JUNCTION_DETECTED,
    READ_TAG,
    EXECUTE_TURN,
    RESUME_TRACKING,
    STOP,
    ERROR
};

std::string stateToString(State state);

class NavigationStateMachine {
public:
    NavigationStateMachine();
    ~NavigationStateMachine() = default;

    // Disable copy/move constructors for safety
    NavigationStateMachine(const NavigationStateMachine&) = delete;
    NavigationStateMachine& operator=(const NavigationStateMachine&) = delete;

    void transitionTo(State newState, const std::string& trigger);
    State getCurrentState() const;
    std::string getCurrentStateString() const;
    void reset();

private:
    State m_currentState;
};

} // namespace line_follower

#endif // NAVIGATION_STATE_MACHINE_H
