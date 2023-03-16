# Line-follower-in-C-C-
Code of a line follower vehicle, made in AVR C for a final project in university.

## Introduction
For ths project we used a microcontroller Atmega328 to control a autonomous line follower device. This device included all the required sensors and actuators for the movement, and the program inlcuded everything in a single code. 

This device is capable of doing the following tasks: 
* Use two infrared sensors and two DC motors to control the movement of the device as a line follower
* Communicate to a station using a integrated UART blueooth device. 
* Use a ultrasonic sensor to detect obstacles in front of it, and stop if needed.
* Two RGB Leds to show the state of the vehicle. Red if stopped, Blue if in movement, Green if waiting for instructions. 
* A 8x16 Led screen on the top, thats shows the direction it's going.

The entire program was made using MicroChip studio, in a program with assembly, C++ and C code. The code used for this project was around 600 lines long, and included some advanced topics like pointers, OOP in C++, ascii tables, modular programming and bluetooth serial comunication. 

## CODE

The main code is in the file 'QBOT.C', and it's dependancies are included. 
