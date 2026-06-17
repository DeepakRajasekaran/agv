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

namespace line_follower {

/**
 * @brief  Helper function to convert a State enum to a human-readable string.
 * @param  state  The State enum value.
 * @return String representation of the state.
 */
std::string stateToString(State state)
{
    switch (state) {
        case State::IDLE:              return "IDLE";
        case State::INITIALIZE:        return "INITIALIZE";
        case State::FOLLOW_LINE:       return "FOLLOW_LINE";
        case State::JUNCTION_DETECTED: return "JUNCTION_DETECTED";
        case State::READ_TAG:          return "READ_TAG";
        case State::EXECUTE_TURN:      return "EXECUTE_TURN";
        case State::RESUME_TRACKING:   return "RESUME_TRACKING";
        case State::STOP:              return "STOP";
        case State::ERROR:             return "ERROR";
    }
    return "UNKNOWN";
}

/**
 * @brief  Constructor initializing the state machine to IDLE.
 */
NavigationStateMachine::NavigationStateMachine()
    : m_currentState(State::IDLE)
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
    m_currentState = State::IDLE;
    
    // Postcondition
    assert(m_currentState == State::IDLE);
}

} // namespace line_follower
