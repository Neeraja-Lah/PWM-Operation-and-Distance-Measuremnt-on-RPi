
.. _assignment2:

PWM Operation and Distance Measurement on RPi
#############################################

Overview
********

This is an application which demonstrates intensity control of a RGB Led.
It uses the RPi's hardware PWM Chip channel A and B and software PWM using
a GPIO pin. The software PWM is generated using POSIX Timer. We also use a
distance sensor to measure distance using two different aproaches. First
approach uses Linux CLOCK_MONOTONIC clock to calculate the elapsed time.
Second approach uses the RPi's ARM processor's Cycle Count Register (CCNT)
to count clocks and convert it to time using current CPU Clock Frequency.

Requirements
************

Raspberry Pi Model 3B+ or higher
GCC compiler
make
Raspberry Pi Kernel Headers
RGB Led Module
HC-SR04 Ultrasonic Distance Sensor
Jumper Wires
Breadboard

Connections
***********
: Connect R, G, B, GND from RGB Module to PWM0 (pin 32), GPIO25 (pin 22), PWM1 (pin 33), GND(pin 20) respectively of the RPi header
: Connect VCC, TRIG, ECHO, GND from HC-SR04 distance sensor to 5V (pin 4), GPIO22 (pin 15), GPIO23 (pin 16), GND (pin 6) respectively of the RPi header

Building and Running
********************

To Build:
: Open the RPi's terminal and navigate to project folder
: Make sure raspberry pi kernel headers are installed. If not, run command 'sudo apt-get install raspberrypi-kernel-headers' to install it
: Use command 'make'. This will compile assignment2.c file and create an executable assignment2 along with kernel object files
: Use command 'sudo insmod cycle_count_mod.ko' to load into the kernel module. We need this to use ARM's CCNT register

To Run:
: Use command sudo ./assignment2 to run the program.
: We need root access to open system files, so don't forget to use sudo.
: To begin the Led sequence, type command 'rgb x y z', where x, y and z are duty cycle percentages from 0 to 100.
: To measure distance, type command 'dist n mode', where n is the number of measurements user wants to perfrom and mode is 0 or 1
: Mode = 0 uses CLOCK_MONOTONIC and Mode = 1 uses ARM's CCNT Register.
: Program shows all the measurements along with average of n measurements. Do not tyoe any commands when the measurement is going on.
: Type 'exit' to stop the RGB sequence and terminate the program gracefully.
: Use command 'sudo rmmod cycle_count_mode' to remove the module.
: Use command 'make clean' to remove all build files.
: If distance does not calculate fisrt time. Terminate program using Ctlr+C and run sudo ./assignment2 again.
: Take multiple distance measurements to check accuracy.
