/*
 * Name:        NavigationStateMachine.cpp
 * Author:      Deepak Rajasekaran
 * Date:        2026-06-12
 * Version:     1.0
 * Description: Implements the NavigationStateMachine class and state helper functions.
 */

#include "NavigationStateMachine.h"
#include <cassert>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>

namespace path_follower {

/**
 * @brief  Helper function to convert a State enum to a human-readable string.
 * @param  state  The State enum value.
 * @return String representation of the state.
 */
std::string stateToString(State state)
{
    using custom_interfaces::msg::ControllerState;
    switch (state) {
        case ControllerState::IDLE:              return "IDLE";
        case ControllerState::INITIALIZE:        return "INITIALIZE";
        case ControllerState::FOLLOW_LINE:       return "FOLLOW_LINE";
        case ControllerState::JUNCTION_DETECTED: return "JUNCTION_DETECTED";
        case ControllerState::READ_TAG:          return "READ_TAG";
        case ControllerState::RESUME_TRACKING:   return "RESUME_TRACKING";
        case ControllerState::STOP:              return "STOP";
        case ControllerState::ERROR:             return "ERROR";
    }
    return "UNKNOWN";
}

/**
 * @brief  Constructor initializing the state machine to IDLE.
 */
NavigationStateMachine::NavigationStateMachine()
    : m_currentState(custom_interfaces::msg::ControllerState::IDLE)
{
}

/**
 * @brief  Transitions the state machine to a new state and logs the transition.
 * @param  newState  The target state to transition to.
 * @param  trigger   The event/condition triggering this transition.
 */
void NavigationStateMachine::transitionTo(State newState, const std::string& trigger)
{
    // Preconditions
    assert(!trigger.empty());
    assert(m_currentState != newState); // Should not transition to the same state

    State prevState = m_currentState;
    m_currentState = newState;

    // Transition Logging format: timestamp | prev_state -> new_state | trigger
    auto now = std::chrono::system_clock::now();
    auto timeT = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&timeT), "%Y-%m-%d %H:%M:%S")
       << " | " << stateToString(prevState) << " -> " << stateToString(newState)
       << " | TRIGGER: " << trigger;
       
    // Print/log the state transition (throttling not needed since transitions are sparse)
    std::cout << "[STATE_TRANSITION] " << ss.str() << std::endl;

    // Postcondition
    assert(m_currentState == newState);
}

/**
 * @brief  Gets the current state enum value.
 * @return The current state.
 */
State NavigationStateMachine::getCurrentState() const
{
    return m_currentState;
}

/**
 * @brief  Gets the current state as a human-readable string.
 * @return String representation of current state.
 */
std::string NavigationStateMachine::getCurrentStateString() const
{
    return stateToString(m_currentState);
}

/**
 * @brief  Resets the state machine back to IDLE.
 */
void NavigationStateMachine::reset()
{
    m_currentState = custom_interfaces::msg::ControllerState::IDLE;
    
    // Postcondition
    assert(m_currentState == custom_interfaces::msg::ControllerState::IDLE);
}

} // namespace path_follower
