Precision
Magnetic
Track Following
Sensor

The MGS1600 is a magnetic guide sensor capable of detect-
ing and reporting the position of a magnetic field along its
horizontal axis. The sensor is intended for line following robotic
applications, using a magnetic tape to form a track guide on
the floor.

The sensor uses advanced signal processing to accurately mea-
sure its lateral distance from the center of the track, with milli-
meter resolution, resulting in nearly 160 points end to end. Tape
position information can be output in numerical format on the
sensor’s RS232 or USB ports. The position is also reported as a
0 to 3V voltage output and as a variable PWM output. Addition-
ally, the sensor supports a dedicated MultiPWM mode allowing
seamless communication with all Roboteq motor controllers
using only one wire.

The sensor will detect and manage 2-way forks. It can be
instructed to follow the left or right track using commands
issued via the serial/USB ports, or using the state of two
digital inputs. All of the sensor’s operating parameters and
commands are also accessible via its CAN bus interface.

In addition to detecting a track to follow, the sensor will detect
and report the presence of magnetic markers that may be posi-
tioned on the left or right side of the track. The sensor is equip-
ped with four LED indicator lights for easy monitoring and
diagnostics.

The sensor incorporates a high performance, Basic-like scripting
language that allows users to add customized functionality to the
sensor. A PC utility is provided for configuring the sensor, capturing
and plotting the sensor data on a strip chart recorder, and visualizing
in real time the magnetic field as it is seen by the sensor.

Applications

 • Automatic Guided Vehicles

 • Automated warehouses

 • Automated shelf restocking systems

 • Material conveying robots

 • Flexible assembly  lines

Key Features

 • Detects and measures position of magnetic track

along horizontal axis

 • Optimized for use with 25mm or 50mm wide adhesive

magnetic tape

 • 10mm to 60mm operating height

 • 160mm sensing width with 1mm resolution

 • Selectable, North or South on top, magnetic polarity

of track

 • Capable of detecting and managing 2-way fork/merges

 • Detection of magnetic “markers” of inverted polarity

at left or right of main track

 • Simple interface to most PLC brands and to micro-

computers

 • Direct and seamless interface to Roboteq motor controllers

 • 100Hz update rate

 • Status LEDs for tape and marker detection

 • Digital outputs for “tape present” and left/right marker

detection

 • Numerical Tape position data output on RS232 or USB ports

 • Tape position on PWM output at 250Hz or 500Hz

The sensor firmware can be updated in the field to take advan-
tage of new features as they become available.

 • Tape position on 0-3V analog output

 • CAN interface up to 1Mbit/s

MGS1600 Magnetic Sensor Datasheet

1

MGS1600• Built-in programming language for optional local pro-

• Delivered with 2 meters multi-conductor cable for all

cessing of tape and marker data

connections

• Easy configuration, testing and monitoring using

• Wide range 4.5V to 30V DC operation

provided PC utility

• Field upgradeable software for installing latest fea-

tures via the Internet

• 165 mm wide x 30 mm deep x 25 mm tall

• -40o to +85o C operating environment

• IP40 rated enclosure. Resistant to water splash

Orderable Product References

Reference

MGS1600

MTAPE25NR

MTAPE50NR

Description

Magnetic guide sensor with serial, USB, analog, PWM and CAN output

25 mm wide magnetic tape for MGS1600 with North top side. 50m (150ft) roll

50 mm wide magnetic tape for MGS1600 with North top side. 50m (150ft) roll

MGS1600 Magnetic Sensor Datasheet

2

Benefits of Magnetic Line Tracking

Because they are totally passive, magnetic tracks are easy to lay and modify. They are dirt immune and can be
made totally invisible under carpet, tile or other flooring cover. The table below lists the differences between
the three major line following technologies used in the industry today.

TABLE 1.

External Variable

Magnetic

Track type

Track shape

Track laying

Laying forks & merges

Dirt immune

Sensible to light conditions

Invisible track

Markers

Passive

Flat tape

Easy

Easy

Yes

No

Yes (3)

Yes (4)

Optical

Passive

Flat trace

Easy

Easy

No

Yes

No

No

Induction

Active (1)

Wire

Difficult (2)

Difficult (2)

Yes

No

Yes

No

Note 1: Requires high frequency current to flow in wire.

Note 2: Forks and merges must not disrupt current flow.

Note 3: Magnetic tape may be hidden under carpet or other non ferrous floor covering.

Note 4: Markers use tape of inverted magnetic polarity and therefore very distinctive to the sensor.

Magnetic Tape Selection & Installation

The sensor is factory calibrated for use with 25mm or 50mm wide tape from Roboteq, but may be used with
tape from other suppliers as well. Only unipolar tape can be used, where one side is all of one magnetic po-
larity and the other of the other polarity. In the default configuration, the sensor expects South on the top
side for the track and North on the top side for markers. The sensor can be configured to operate with tape of
inverted polarity. The sensor will not work with tape of alternating polarity. To determine the tape orientation,
point compass towards the top (non adhesive) side of the tape.  The north pointing needle will be attracted to
the south side of the tape, and the south pointing needle will be attracted to the north side of the tape.

NNNNNNNNNNN

SSSSSSSSSSSS

NSNSNSNSNSN

SSSSSSSSSSSS

NNNNNNNNNNN

SNSNSNSNSNS

FIGURE 1. Magnetic Tape

Operating height is up to 50mm when used with 25mm wide tape and 60mm when used with 50mm wide
tape. At greater heights, the magnetic field of the tape is weaker and the sensor will be less immune to noise.
For best results, operate at 20 to 30mm with 25mm tape and 20 to 40mm with 50mm tape.

Sensor Installation

The sensor must be mounted so that it is parallel with the floor and the magnetic track. Two mounting holes
are  provided at both ends of the enclosure. When installing, allow room the accessing the USB connector
under the plug.

3

MGS1600 Magnetic Sensor Datasheet

Version 1.4  July 14, 2023

USB

Power &
IO Cable

Power

Right Side
Relative to Forward
Direction

Forward
Reference
Direction

Left Side
Relative to Forward
Direction

I/O and Power Cable

Left Marker

Right Marker

Tape Detect

FIGURE 2. Sensor Layout

I/O and Power Cable

The MGS1600 comes with a 15-pin DSub connector at the end of 2.0 meter multi-conductor cables for power-
ing the sensor and accessing all the I/O signals. The connector can be cut off and the connections done directly
on the wires. The connector pins and wire colors are identified in Table 2, below.

FIGURE 3. Connector Pin Locations

TABLE 2.

Wire color

braid

black

red

Signal

Ground

Ground

Power In

Type

Power

Power

Power

yellow + black

Power Control

Input

light green + black

red + black

CANL

CANH

purple

pink

yellow

blue

brown

orange

green

grey

white

I/O

I/O

Input

Input

Output

Output

Output

Fork Right

Fork Left

Analog Out

PWM Out

Left Marker

Right Marker

Output

Track Present

Output

RxData

TxData

Input

Output

white + black

Reserved

N/A

DSub pin Description

5

5

14

10

6

7

8

1

4

15

9

11

13

3

2

12

Ground

Ground

4.5V to 30V DC Power supply input

Power down

CAN Low

CAN High

Select right track

Select left track

0-3V (1.5V center) Analog track position

Track position PWM output

Left marker detected

Right marker detected

Track detected

RS232 receive data

RS232 transmit data

Do not connect

MGS1600 Magnetic Sensor Datasheet

4

Powering the sensor

Apply a 4.5V to 30V Max voltage between the ground wire (black) or braid, and the power input
wire (red). Be careful not to confuse the solid red power wire with the red/black wire. If need-
ed, the sensor can be powered down by connecting the Power Down wire (yellow & black) to
ground, or applying a logic 0 signal. If the Power Down wire is floating, or pulled above 1.5V,
the sensor will turn on. The sensor will also be powered if it is connected to a PC via the
USB connector. The Power Down wire will not turn off the sensor if powered from the USB.

Important Warning

Only ground or float to the Power Down signal. Never apply a voltage higher than 5V to this wire. Product
damage can occur.

RS232 Connection

Serial communication with the sensor is done using the RxData (grey) and RxData (white) signals. The ground
wire (black or braid) must be connected in order to provide a reference to the RxData and TxData signal. Serial
communication will not work with microcomputers equipped with TTL-levels serial ports.

PWM Output

The PWM Output wire (blue) is always active. In default configuration, multiple pulses of variable widths are
used to carry all sensor information, including tape detect and marker position to Roboteq motor controllers.
The output can also be configured to carry the tape position by varying the duty cycle of a single, continuous
pulse from 50%, when the tape is centered to 25% and 75% duty cycle when the tape is at one end or the
other of the sensor. The PWM output is centered at 50% when no tape is detected.

Analog Output

The Analog Output wire (yellow) is always active and will give the tape position by varying the voltage from
1.50V, when the tape is centered, to 0 and 3V when the tape is at one end or the other of the sensor. The Ana-
log output is centered at 1.50V when no tape is detected.

Track Present Outputs

The Track Present wire will output a 5V level when a magnetic tape is within the sensor’s range. If no tape is
detected, the output will be set to 0V.

Left and Right Markers Outputs

The Left Marker wire (brown) and Right Marker wire (orange) will output a 5V level when a left or right marker
is detected by the sensor. If no marker is detected, the output will be set to 0V. These outputs mirror the state
of the left and right marker detect LEDs.

Fork Left and Fork Right Inputs

The Fork Left wire (pink) and Fork Right wire (purple) are used to select which of the Left or Right tape capture
must be output on the PWM and Analog wires.

CAN Low and CAN High

The CAN Low wire (light green and black) and CAN High wire (red and black) are used to connect the sensor to
a CAN network. Do not confuse the solid red wire (Power supply) with the red/black wire (CAN High). The sen-
sor does not include a 120 ohm termination resistor.

5

MGS1600 Magnetic Sensor Datasheet

Version 1.4  July 14, 2023

Serial Port Settings

Serial Port Settings

The baud rate and communication settings on the sensor are set as follows:

 • 115200 bits/s

 • 8-bit data

 • No parity

 • No flow control

The baud rate can be changed to different values but only while the controller is connected to the configuration
PC utility via USB. Beware that once the baud rate is changed, it will no longer be possible to have the PC utili-
ty communicate with the sensor via the serial port until the speed is changed back to 115200 bit/s.

Track information

The presence and position of a magnetic track is output on the I/O connector, and/or transmitted via the serial
communication port or USB. When the sensor detects the presence of a magnetic track it will activate the
Track Present output on the I/O connector. The track position information is also output as a 0-3V analog signal,
and a PWM pulse of a user definable period and duty cycle range. The track detect and position are reported
on the RS232 or USB ports. The position is reported as a signed value, in millimeters, using the center of the
sensor as the 0 reference.

Fork and Merge Management

The sensor has an algorithm for detecting and managing up to 2-way forks and merges along the track. Inter-
nally, the controller always assumes that two tracks are present: a left track and a right track. When following
a single track, the sensor considers that the two tracks are superimposed. When entering forks, the track wid-
ens, as does the distance between the left and right tracks.

L=0, R=0

L=-0, R=50

L=-0, R=20

L=0, R=0

FIGURE 4. Fork Management

When approaching merges, the sensor will report a sudden spread of the left and right tracks, but will other-
wise operate the same way as at forks.

MGS1600 Magnetic Sensor Datasheet

6

L=0, R=0

L=-0, R=20

L=-0, R=50

L=0, R=0

L=0, R=0

L=-0, R=50

L=-0, R=20

L=0, R=0

L=0, R=0

L=-0, R=20

L=-0, R=50

L=0, R=0

FIGURE 5. Merge Management

Both track positions can be read via the serial port. Using the state of the Fork Left and Fork Right digital in-
puts, the sensor will send the left or right track information to the analog and PWM outputs, according to Table
3, below.

TABLE 3.

Fork Left

Fork Right

Analog and PWM Output

Low

High

Low

High

Low

Low

High

High

No change

Left track position

Right track position

Left or right track position depending on command received on
RS232/USB

When both inputs are high or unconnected, the selected track will be based on RxData digital input if config-
ured, otherwise the selected track will be based on command received via the sensor’s serial/USB port, or set
using the sensor’s scripting language.

Marker Detection

Markers are pieces of magnetic tape that are affixed on the left or/and right side of the main track. To differen-
tiate them from the track, markers have opposite magnetic polarity. These markers can be used to inform the
AGV of special areas along the track, such as forks or merges ahead, high or low speed zones, charge stations,
etc. Markers must be positioned 15 to 30mm away from the edge of the main track for proper operation.

Sensor

15-30mm

North

South

North

Left Marker

Main Track

Right Marker

FIGURE 6. Direction Markers

7

MGS1600 Magnetic Sensor Datasheet

Version 1.4  July 14, 2023

Simple Left Marker

Simple Right Marker

2 Dimensional Markers

Sensor

15-30mm

North

South

North

Left Marker

Main Track

Right Marker

Absolute with Markers Detection Mode

The figure below shows example of a simple marker (i.e. marker present or absent) and 2 dimensional markers
where a pattern is used to encode more complex information. In this example, using the built in scripting lan-
guage, the sensor can be made to count the number of right markers while a left marker is present.

Simple Left Marker

Simple Right Marker

2 Dimensional Markers

FIGURE 7. Markers Usage Examples

Absolute with Markers Detection Mode

The sensor has two modes of operation. In Absolute mode, the field is measured relative to a reference ambi-
ent 0 level. A little above this level, the signal will be considered as being from the Track. At 3 user selectable
sensitivity levels below the zero line, the signal will be considered as a Marker.

Track

Zero Level

Marker

Figure 8 Detection Levels in Absolute Mode

Track Detection Threshold

High
Medium
Low

}

Marker Detection
Sensitivity

This mode is therefore dependent on the ambient magnetic field to be quite stable throughout the path of the
AGV, and that the zero level be calibrated. After calibration if no track or marker is present, the level should
hover around the 0 level. It is recommended to survey the site with the sensor around 25mm all around the
projected path to verify that there are no local disturbance by metal part in the floor.

If the zero level is higher in some areas, it may cross the Track detection threshold and detect a track where there
is none. This can be corrected by adding a correction that has the effect of shifting the entire field capture up or
down. Use the ZADJ configuration command to make this correction, as seen on Table 7. Configuration Com-
mands.

MGS1600 Magnetic Sensor Datasheet

8

Track

Zero Level

Track Detection Threshold

High
Medium
Low

}

Marker Detection
Sensitivity

Marker

FIGURE 9. Absolute Capture Shifted Using ZADJ Configuration Command

Be aware that if the sensor capture is shifted too low, this could then trigger false Marker detections. This can
be alleviated by selecting a lower marker sensitivity level.

Relative without Markers Mode

In the Relative mode, the sensor evaluates the shape of the curve independently of its position relative the 0
level. It then sets the detection level to around the middle of the curve.

Track

Max

Min

Track Detection Threshold

FIGURE 10.  Relative Capture is Independent of the Ambient 0 Level

This technique is therefore a lot less sensitive to variations to the ambient level. However, it does not permit
the use of Markers.

Diagnostic LEDs

Since magnetic fields are invisible, the sensor is equipped with four LED indicator lights to help with setup and
troubleshooting. The LED positions are shown in Figure 2, on Page 4 of this Datasheet. The Power LED will be
lit when the sensor is on. The Track Detect/Track Position LED is a dual usage LED that will illuminate when a
track is present. The LED is bi-color and will gradually shift to red when the track is at the left of the sensor, and
to green as the track moves to the right. Two additional LEDs will turn on when left or right markers are detect-
ed.

Interfacing the Sensor to PLCs

The sensor can be fully interfaced to a PLC with only three wires as shown in Figure 11, below. The PWM
method is preferred to analog as it is more accurate and less vulnerable to interference.

9

MGS1600 Magnetic Sensor Datasheet

Version 1.4  July 14, 2023

Interfacing the Sensor to Roboteq Motor Controllers

4.5V to 30V

Sensor
4.5V to 30V

4.5V to 30V

FIGURE 11. PLC Interfacing
Sensor

Fork Left

Fork Right

Analog or PWM

Fork Left

Fork Right

Fork Left
Analog or PWM
Fork Right

PLC

PLC

PLC
Interfacing the Sensor to Roboteq Motor Controllers

Sensor

Analog or PWM

The MGS1600 will interface directly and seamlessly with all Roboteq models of controllers for Brushed DC,
Brushless DC motors and AC Induction motors. The sensor can be powered from the controller’s 5V output.
The left, right, tape detect and marker information is sent from the sensor using the PWM Output configured
as “Roboteq MultiPWM”. The signal must be connected to any of the controller’s Pulse Inputs configured with
the PC utility as “Magsensor”. The data is sent continuously with a 10ms update rate. Roboteq provides script
examples that run in the motor controller for implementing basic line following AGV functionality.

Sensor

MultiPWM

5V

M

Roboteq
Motor
Controller

M

Sensor

Sensor

5V

MultiPWM

5V

MultiPWM

Roboteq
Motor
Controller
Roboteq
Motor
Controller

M

M

M

M

FIGURE 12. Roboteq Motor Controllers Interfacing

Interfacing the Sensor to PCs or Microcomputers

Sensor

USB

PC

Interfacing the sensor to a PC requires a simple USB connection. The sensor will be powered via the 5V pres-
ent on the USB.

Sensor

Sensor

USB

USB

PC

PC

FIGURE 13. PC Interfacing

If no USB is available, interfacing can be done using the PC or Microcomputer RS232 port and a separate 4.5V
to 30V power supply.

MGS1600 Magnetic Sensor Datasheet

10

Using the PC Utility

A powerful utility is available for download from Roboteq’s website. The PC Utility can assist in setting up, monitor-
ing and performing maintenance functions. While the sensor is delivered ready to use right off the box, it contains
many parameters that can easily be changed via the user-friendly PC Utility menus. For testing and troubleshooting,
the utility includes a graph that plots in real time the shape and strength of the magnetic field as it is seen by the
sensor. A strip chart recorder allows the user to plot the track and marker information, and save the data in an excel
spreadsheet for analysis. The utility is also used for performing field updates of the sensor firmware and for editing
and running scripts.

FIGURE 14. MagSensor Control Utility

MicroBasic Scripting

The MGS1600 features the ability for the user to write programs that are permanently saved into, and run from
the sensor’s Flash Memory. This capability is the equivalent of combining the functionality of a PLC or Single Board
Computer directly into the sensor. The language is a very simple, yet powerful language that resembles Basic.
Scripts can be simple or elaborate, and can be used for various purposes. For example sensor data manipulation
and conversion, two dimension marker processing, or even the full motion and steering control for a simple line
following robot. See the Microbasic manual for details on the language.

11

MGS1600 Magnetic Sensor Datasheet

Version 1.4  July 14, 2023

Sensor Calibration

Sensor Calibration

The sensor is factory calibrated for 25mm and 50mm wide magnetic tapes available from Roboteq. If tapes of dif-
ferent width or magnetic strength are used, the sensor can be re-calibrated by the user. The sensor is also factory
calibrated to compensate for the natural ambient magnetic field. For best results, the ambient “zero” must be
reset in every new installation. This is done by clicking on the “Calibrate Zero” button on the Setup tab of the PC
utility. Make sure that the sensor is away from any magnetic material when doing the zero calibration.

When the calibration takes place, an integrity test is executed in order to detect if any of the 16 IC sensors has
failed. If the test detects an error, a respective message is printed, the LEDs flash and the bit 8 MGS is set. If
this happens make sure the sensor is not close to any magnetic field and retry sensor calibration. If the prob-
lem remains, then most probably the sensor is broken.

Field Sensor Calibration

Sensor calibration can also be done automatically at sensor start-up without having to access the interface via
Magsensor Utility. This can be achieved by enabling the field calibration flag (FCAL) and saving the configuration
to flash memory and restarting the sensor. When the Sensor powers up, it waits for half a second to detect four
markers (Tape Detect LED is Red). If markers are detected then it waits for 1,5 seconds in order to remove the
markers (Tape Detect LED is Green). When the markers are removed the sensor is calibrated.

To quickly calibrate the sensor, take a piece of carton and attach four markers at equal intervals within
the range of the length of the sensor (16cm). Put the carton under the sensor (as close as possible) and
power up the sensor. When the Tape Detect LED illuminates green, remove the carton. The Tape Detect
LED turns red during the calibration. IF the Tape Detect LED start flashing green every 1 second then the
calibration is over and the sensor is ready to be used.

Note: Begining with v3.0 of the MGS firmware, the Field Sensor Calibration has been implemented within
the firmware.This applies only to firmare v3.0 and later.

Command Reference Summary

The sensor accepts a number of commands via its RS232 and USB ports for reading operational data, sending
commands, setting configuration, and performing maintenance.

Real Time Queries

These are commands for reading sensor data. They begin with the question mark character. Table 4 shows the
list of supported queries.

Each time a query is executed, it is stored in a history buffer and may therefore be automatically repeated
at a periodic rate using the # character with the following syntax:

#
# nn

# C

repeat last query in queue
repeat last queries every nn ms.
Example: # 100 to execute one query from the history queue every 100ms
clear queue

MGS1600 Magnetic Sensor Datasheet

12

B

MGD

MGM

MZ

T

MGT

VAR

MGS

MGM

B

R

TV

VAR

TX

ZER

TABLE 4.

Command Arguments

Description

Index Value

Read User Boolean Variable

None

Read Track Detect

Examples

?B 1

?MGD

[MarkerNumber] Read all markers, or one of the 2

?MGM, ?MGM 2

[SensorNumber]

Read all internal sensor values, or one of the 16

?MZ, ?MZ 16

None

Read selected track

?T

[TrackNumber]

Read both the left and right tracks, or one of the 2

?MGT, ?MGT 2

Index Value

Read User Integer Variable

None

Read MagSensor Status

?VAR 5

?MGS

[MarkerNumber] Read all markers, or one of the 2 and Cross Tape flag

?MGM, ?MGM 2

MGX (1)

None

Read Tape Cross Detection

?MGX

Note 1: This feature is available begining with firmware v3.0. It is not available with older firmware versions.

Real Time Commands

These are commands used to instruct the sensor to do something. They begin with the exclamation mark char-
acter. Table 5 shows the list of supported commands.

TABLE 5.

Command Arguments

Description

Index Value

Set User Boolean Variable

Example

!B 1 1

Option

none

Run/Stop/Resume MicroBasic scripts

!R = Run/Resume, !R 0 = Stop, !R 2 = Restart

Follow Right track

!TV

Index Value

Set User Integer Variable

!VAR 5 12345

none

none

Follow Left track

!TX

Set zero calibration level for magnetic sensors

!ZER

Configuration Commands

These commands are used to read or modify sensor configuration parameters. They begin with the ~ character
for reading and the ^ character for writing. Table 6 shows the list of supported configuration commands.
However, it is easier and preferable to use the PC utility menus for inspecting and changing configurations. If
changing manually, remember to save the new configuration to flash with the %EESAV. Otherwise, the sensor
will revert to the previously active configuration next time it is powered on.

13

MGS1600 Magnetic Sensor Datasheet

Version 1.4  July 14, 2023

Command Reference Summary

TABLE 6.

Command Arguments Range

Default Description

ANAM (1)

Value

0 = Selected Track (0-3V), 1= Tape
Detection, 2=Tape & Marker Detect

BADJ

BRUN

DIM (1)

FCAL (1)

MMOD

Value

Value

Value

None

Value

PWMM

Value

RSBR(2)

Mode

+/- 100

0 = disable, 1 = enable

0 = disable, 1 = enable

0 = disable, 1 = enable

0=Absolute (w/ Markers), 1=Rela-
tive (w/0 Markers)

0 = Roboteq MultiPWM
1= Selected Track at 250Hz
2= Selected Track at 500Hz
3=Tape Detection (1)
4=Tape & Marker Detect (1)

0 = 115.2K
1= 57.6K
2 = 38.4K
3 = 19.2K
4 = 9600

SCRO

ScriptOutput

0 = last port used, 1 = RS232,
2 = USB

TINV

TMS

TPOL

TWDT

TXOF

Value

Value

Value

Value

Value

0 = Left - to Right +,
1= Left + to Right -

0= High, 1= Med, 2= Low

0 = South top, 1= North top

0 = 25mm, 1= 50mm

-100 to +100

ZADJ

Ch Value

+/- 1000

0

0

0

0

0

0

0

0

0

0

0

0

0

0

0

Analog Output mode

Correction to Left/Right tape reading

Auto start MicroBasic script at power up

RxData as digital Input

Field Calibration

Tape Detection Mode

PWM Output mode

Set serial port bit rate

Output port for MicroBasic print com-
mands

Change sign of position values

Select Marker Sensitivity

Select magnetic tape width

Select magnetic tape polarity

Offset added/subtract to track position
values

Zero Level User Offset for each of the
16 internal sensors. Send ^ZADJ 0 nn to
change all sensors at once.

Note 1: This feature is available begining with firmware v3.0 and is not available in older versions of the firmware.

Note 2: Serial port bit rate can only be changed while the sensor is connected to the PC via USB

MGS1600 Magnetic Sensor Datasheet

14

Maintenance Commands

These commands are used to perform maintenance functions on the sensor. They begin with the % character.
Table 7 shows the list of supported configuration commands.

TABLE 7.

Command

Arguments

Description

CLSAV

CLRST

EELD

EERST

EESAV

ZERO

None

Key (1)

None

Key (1)

None

None

Save calibration to EEPROM

Load factory default calibration

Load configuration from EEPROM

Load factory default configuration

Save configuration to EEPROM

Set zero calibration level for magnetic sensors

Note 1: To prevent accidental entry, the command must be followed by the key 321654987

Note 2: This feature is available begining with firmware v3.0 and is not available in older versions of the firmware.

CANbus Communication

The sensor supports the following four CAN protocols:

RoboCAN: a simple meshed networking structure to exchange commands and queries with any other Roboteq
motor controller or sensor.
RawCAN: a low level structure that allows to build and parse CAN frames using the MicroBasic scripting
language
MiniCAN: a structure that borrows CANOpen’s TPDO and RPDO mechanisms for sending and capturing frames
with fixed content.
CANOpen: an industry standard system ensuring interoperability with other vendor’s PLCs and devices (this
feature is available begining with firmware v3.0 and is not available in older versions of the firmware.).

Details on these protocols can be found in the separate Roboteq CAN Communication manual.

The structure and content of the TPDO and RPDO frames is the same in both MiniCAN and CANOpen and is
shown in the table below.

Header: TPD01: 0x180 + NodeID

              TPD02: 0x280 + NodeID

Byte1

Byte2

Byte3

Byte4

Byte5

Byte6

Byte7

Byte8

TPDO1

TPDO2

Left Track

VAR 1

Right Track

Flags

VAR 2

CANOpen Bits:

Bit8

Bit7

Bit6

Bit5

-

-

Sensor
Failure

-

MiniCAN Bits:

Bit4

Right
Marker

Bit3

Left
Marker

Bit2

Tape
Detect

Bit1

Tape
Cross

Bit8

Bit7

Bit6

Bit5

Sensor
Failure

-

-

-

Bit4

Tape
Cross

Bit3

Right-
Marker

Bit2

Left
Marker

Bit1

Tape
Detect

15

MGS1600 Magnetic Sensor Datasheet

Version 1.4  July 14, 2023

CANbus Communication

Header: RPD01: 0x200 + NodeID
             RPD02: 0x300 + NodeID

Byte1

Byte2

Byte3

Byte4

Byte5

Byte6

Byte7

Byte8

RPD01

RPD02

VAR 2

VAR 4

VAR 3

VAR 5

In CAN Open the sensors Real-time Commands and Queries are mapped as shown in the Object Dictionary
below. Configuration commands are not directly accessible via CANOpen.

TABLE 8.

Index

Sub (Hex)

Entry Name

Runtime Commands

Set User Integer Variable n

Set User Bool Variable n

Save Config to Flash

MicroBasic Run

Follow Left track

Follow Right track

01 to 10

01 to 32

00

00

00

00

00

0x2005

0x2015

0x2017

0x2018

0x201A

0x201B

0x2020

TABLE 9.

Set zero calibration level for magnetic sensors

U8 WO

Data
Type &
Access

Command Name

S32 WO

VAR

S32 WO

B

U8 WO

U8 WO

U8 WO

U8 WO

EESAV

BRUN

TX

TV

ZER

Index

Sub (Hex)

Entry Name

Runtime Queries

Data
Type &
Access

Command Name

0x2106

0x210F

0x2115

0x211D

0x211E

0x211F

0x2120

0x212D

0x212E

0x2138

1 to 10

00

01-10

01

01

02

03

01

02

01

01 -10

01 -10

01

Read User Integer Variable n

S32 RO

VAR

Read Dominant Track

Read User Bool Variable n

Read Track Detect

Read Left Track

Read Right Track

Read Selected Track

Read Left Marker

Read Right Marker

Read Status

Read Raw Sensor N

Read Zero Adjusted Raw Sensor n

Read Cross Tape Detection

S8 RO

U8 RO

U8 RO

S16 RO

T

B

MGD

MGT

U8 RO

MGM

U16 RO

U32 RO

S32 RO

U8RO

MGS

MRS

MZ

MGX

MGS1600 Magnetic Sensor Datasheet

16

USB Communication

Use USB only for configuration, monitoring and troubleshooting. USB is not a reliable communication method when
used in electrically noisy environments. Further, communication will not always recover after it is lost without un-
plugging and replugging the connector, or restarting the controller. RS232 is the preferred method of communication
when interfacing with a computer. USB and CAN are able to operate at the same time on the MGS1600. Connect-
ing to a computer via USB will not disable the CAN interface.

Sensor Characteristics

TABLE 10.

Parameter

Capture width

Resolution

Operating height with 25mm track

Operating height with 50mm track

Update rate

Min

1

10

20

Type

160

1

30

30

100

Max

Units

2

50 (1)

60 (1)

mm

mm

mm

mm

Hz

Note 1: Ambient magnetic fields may impair sensor data at its highest clearance. Greater clearances can be reached with
doubled tape, or by using stronger magnetic material.

Electrical Characteristics

Absolute Maximum Values

The values in the table below should never be exceeded. Permanent damage to the controller may result.

TABLE 11.

Parameter

Measure point

Min

Type

Max

Units

Power Supply Input Voltage

Ground to Red wire

Digital Input Voltage

Fork Left and Right inputs

Digital Output Current

Digital and PWM outputs sink

Analog Output Current

Analog Output

-1

-1

CAN Input Voltage

Ground to CAN-H and CAN-L pins

RS232 I/O pins Voltage

External voltage applied to Rx/Tx pins

35

15

20

10

40

25

Volts

Volts

mA

mA

Volts

Volts

Power Stage Electrical Specifications (at 25oC ambient)

TABLE 12.

Parameter

Input Voltage on 5V inputs

Power consumption

Measure point

Ground to Red wire

Power supply input

Min

Type

Max

Units

4.5

120 (1)

30

Volts

20 (1)

mA

Note 1: Consumption is lower as the power supply voltage is higher.

17

MGS1600 Magnetic Sensor Datasheet

Version 1.4  July 14, 2023

Electrical Characteristics

Command, I/O and Sensor Signals Specifications

TABLE 13.

Parameter

Measure point

Min

Type

Max

Units

Digital Output Current

Output pins, sink/source current

Digital Input 0 Level

Digital Input 1 Level

Ground to Input pins

Ground to Input pins

Analog Output Range

Ground to Output pin

-1

3

0

TABLE 14.

Parameter

Measure point

Min

Type

Analog Output Current

Ground to Output pin

PWM Frequency

PWM Duty Cycle

PWM Output

PWM Output

Note 1: 250 or 500Hz user selectable

250 (1)

25

20

1

15

3

mA

Volts

Volts

Volts

Max

10

500 (1)

75

Units

mA

Hz

%

Scripting

TABLE 15.

Parameter

Measure Point

Min

Type

Max

Units

Scripting Flash Memory

Internal

Max Basic Language programs

Internal

Integer Variables

Boolean Variables

Execution Speed

Note 1: 32-bit words

Internal

Internal

Internal

Environmental & Mechanical Specifications

TABLE 16.

2048

500

50 000

750

1024

1024

Bytes

Lines

Words (1)

Symbols

Lines/s

Parameter

Measure Point

Min

Type

Max Units

Operating Temperature

Weight

Protection

Cable Diameter

Cable Length

Note 1: Weight includes cable

Sensor

Sensor

Case

Cable

Cable

-20

85

oC

250 (.55) (1)

g (lbs)

IP40

7.0

2.0

mm

m

MGS1600 Magnetic Sensor Datasheet

18

165.0

25.0

82.5

6.5

30.0

6.5

10.0

FIGURE 15. MGS1600 front view and dimensions

165.0

30.0

FIGURE 16. MGS1600 top view and dimensions

25.0

82.5

5.0

25.0

4.0

25

10.0

5.0

25.0

6.5

10.0

6.5

4.0

25

10.0

19

MGS1600 Magnetic Sensor Datasheet

Version 1.4  July 14, 2023

