/**
 * @file RobotControl.cpp
 * @author Adrian Chong
 * @brief A library for controlling a robot created for Mini Robocon 2018
 * @version 0.1
 * @date 2018-10-07
 * @copyright Copyright (c) 2018
 * 
 * This Arduino library is a collection of functions used to control the robot, created for a competition, Mini Robocon 2018.
 * The aim of the competition is to throw bouncing balls through hoops and land on nets with spins.
 * 
 * The Aiming System consists of the following parts:
 * - Angle adjustment
 * - Upper and lower motor speeds, for adjusting spin
 * - Distance between the vehicle and the hoop
 * 
 * The movement system uses four Mecanum wheels.
 * This allows omni-directional movement without turning the entire vehicle.
 * The type of wheel requires special operaion of the direction of the motors.
 * Stated in this documentation: http://files.andymark.com/PDFs/MecanumWheelTutorial.pdf
 * 
 */

#pragma once

#include <Arduino.h>
#include <Servo.h>

// Aiming System - Ball Shooting
const int UPPER_MOTOR_PIN = 2; // < PWM pin location for controlling the upper motor with 12V
const int LOWER_MOTOR_PIN = 3; // < PWM pin location for controlling the lower motor with 12V
int upperSpeed = 85;           // < PWM value for controlling the speed of the upper motor
int lowerSpeed = 60;           // < PWM value for controlling the speed of the lower motor

// Aiming System - Angle Control
const int A_1 = 22, A_2 = 23, B_1 = 24, B_2 = 25, STEPPER_TOP = 42, STEPPER_BOTTOM = 43;        // < Pin location for stepper
const int STATE_NUM = 8, REV_NUM_STATES = 400;                                                  // < State constants
const int STATES[STATE_NUM] = {0b1000, 0b1010, 0b0010, 0b0110, 0b0100, 0b0101, 0b0001, 0b1001}; // < Diff. States
const int DELAY_US = 500;                                                                       // < Delay in between states

int steps = 6400, stateCounter = 0; // < Var for which state the stepper is in

// Aiming System - Ball Rising
Servo loadingServo;
const int SERVO_PIN = 12;                                              // < Pin of the servo
const int SERVO_UPPER = 160, SERVO_LOWER = 45, DELAY_BEFORE_DEC = 500; // < Pos of the servo

// Movement System - Wheel Pins & Constant
const int M101 = 10, M102 = 11, M201 = 8, M202 = 9, M301 = 7, M302 = 6, M401 = 5, M402 = 4; // < Movement wheels control pins
const int ALIGN_POWER = 60, NORMAL_POWER = 100, HIGH_POWER = 255;                           // < Default power levels
const float M1_OS = 1, M2_OS = 1, M3_OS = 1, M4_OS = 1;                                     // < Motor power offsets for calibration

// Setup motor
void robotSetup();

// Aiming System - Motor Control
void setMotorSpeed(int, int);

//Aiming System - Stepper Control
int setState(int);
int turnServo(int, int);

// Aiming System - Ball Loading
void loadBall();

// Movement System - Movement Direction
void moveN(int = NORMAL_POWER);
void moveS(int = NORMAL_POWER);
void moveE(int = NORMAL_POWER);
void moveW(int = NORMAL_POWER);
void moveNE(int = NORMAL_POWER);
void moveNW(int = NORMAL_POWER);
void moveSE(int = NORMAL_POWER);
void moveSW(int = NORMAL_POWER);

// Movement System - Rotation
void rotCW(int = NORMAL_POWER);
void rotCCW(int = NORMAL_POWER);
void selfRotCW(int = NORMAL_POWER);
void selfRotCCW(int = NORMAL_POWER);

void moveStop();