PID Controller for Magnetic based AGV.
Starting date : 10/6/2026

End Date : 17/6/2026

Objective

Develop a complete PC-side software application for controlling and guiding an
Automated Guided Vehicle (AGV) using CAN communication.

The AGV consists of:

● Roboteq Motor Controller
● Differential Drive Mechanism
● Magnetic Line Sensor
● Tag Reader
● CAN Communication Interface

The goal is to make the AGV follow a magnetic path and make navigation decisions at
junctions using magnetic tags.

Hardware Description

Available Components

Motor Controller

● Roboteq Motor Controller
● Communication Interface: CAN Bus

Drive System

● Differential Drive
● Left Wheel Motor

● Right Wheel Motor

Sensors

Magnetic Line Reader

Tag Reader

Task 1 – CAN Driver Development

Develop a CAN communication driver on the PC.

Requirements:

Receive

Receive and decode:

● Motor feedback
● Motor RPM
● Motor current
● Fault status
● Sensor messages

Transmit

Transmit:

● Left wheel speed command
● Right wheel speed command
● Controller enable/disable
● Emergency stop

Software Requirements

● Object-oriented design
● Separate CAN abstraction layer
● Logging support
● Error handling
● Thread-safe implementation

Expected Deliverables:

● CAN Driver Library
● Test Application
● Documentation

Task 2 – Differential Drive Control

Implement differential drive kinematics.

Inputs:

● Linear velocity (V)
● Angular velocity (W)

Outputs:

● Left wheel speed
● Right wheel speed

Requirements:

● Forward motion
● Reverse motion
● In-place rotation
● Curved motion

Task 3 – Magnetic Line Following

Implement line following using the magnetic sensor.

Requirements:

● Read sensor position error
● Maintain robot on magnetic tape
● Smooth steering corrections

Expected Behaviour:

● Robot should continuously track the tape.
● Robot should recover from small disturbances.

Task 4 – PID Controller

Implement a PID controller for path guidance.

Controller Input:

Error = Tape Center Position - Robot Position

Controller Output:

Steering Correction

PID Parameters:

● Kp
● Ki
● Kd

Requirements:

● Runtime parameter adjustment
● Integral windup protection
● Output limiting
● Logging of controller values

Performance Evaluation:

● Settling time
● Overshoot
● Steady-state error
● Path tracking accuracy

Task 5 – Junction Handling

The magnetic tape contains junctions.

For example it is having a junction which is leading to different ways.
so the robot should be go to particulate locations accordingly.

Task 6 – Navigation State Machine

Implement a navigation state machine.

Recommended States:

Idle
1.
2.
Initialize
3. Follow Line
4. Junction Detected
5. Read Tag
6. Execute Turn
7. Resume Tracking
8. Stop
9. Error

Requirements:

● State transition logging
● Fault recovery
● Safe stopping

Task 7 – Safety

Implement:

● Communication timeout detection
● Motor fault handling
● Emergency stop
● Sensor fault detection

Behaviour:

● Stop robot safely on fault.
● Generate fault log.

