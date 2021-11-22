# Aerial-Manipulator-Control

This code was developed by me, with help from my team, for a proprietary aerial manipulator. The codes were programmed in C for Arduino and produced results that verified our design.

Two microcontrollers linked via I2C were used for the setup. This was done to address incompatability between the PID control of the counter-balance system and the manipulator control.

# Manipulator_Link_Control_Sender.ino

This code was developed for the main Arduino board. It was resposible for everything except for counter-balance system control.

This code:

Initializes all known values

Controls the manipulator links

Communicates to a remote control for manual control

Computes inverse kinematics for automated motions for testing

Calculats counter-balance system values for any motion

Sends counter-balance system values via I2C to another Arduino board

Receives accelerometer data and translates to pitch and roll for testing
