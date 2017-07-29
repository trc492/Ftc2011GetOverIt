#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="RobotInfo.h" />
///
/// <summary>
///     This module contains the Robot Info constants.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _ROBOTINFO_H
#define _ROBOTINFO_H

//
// Drive info.
//
#define GEAR_RATIO              (24.0/16.0)     //motor:wheel=24:16
#define WHEEL_CIRCUMFERENCE     13.00           //in inches
#define CLICKS_PER_REVOLUTION   1440.0          //in clicks
#define WHEELBASE_DISTANCE      60.00           //in inches

//
// Assuming the encoder is mounted on the motor shaft.
//
#define DISTANCE_PER_CLICK      (WHEEL_CIRCUMFERENCE*GEAR_RATIO/ \
                                 CLICKS_PER_REVOLUTION)
#define DEGREES_PER_CLICK       (360.0/(PI*WHEELBASE_DISTANCE/ \
                                        DISTANCE_PER_CLICK))

//
// PID Control constants.
//
#define ENCODER_DRIVE_KP        12.0
#define ENCODER_DRIVE_KI        0.0
#define ENCODER_DRIVE_KD        0.0
#define ENCODER_DRIVE_TOLERANCE 1.0

#define ENCODER_TURN_KP         10.0
#define ENCODER_TURN_KI         0.0
#define ENCODER_TURN_KD         0.0
#define ENCODER_TURN_TOLERANCE  2.0

#define GYRO_TURN_KP            12.0
#define GYRO_TURN_KI            0.0
#define GYRO_TURN_KD            0.0
#define GYRO_TURN_TOLERANCE     1.0

#define SONAR_DISTANCE_INCHES  (1/2.54)
#define SONAR_DRIVE_KP          5.0
#define SONAR_DRIVE_KI          0.0
#define SONAR_DRIVE_KD          0.0
#define SONAR_DRIVE_TOLERANCE   5.0

#define IRSEEK_TURN_KP          10.0
#define IRSEEK_TURN_KI          0.0
#define IRSEEK_TURN_KD          0.0
#define IRSEEK_TURN_TOLERANCE   0.5

#define BALANCE_DRIVE_KP        180.0
#define BALANCE_DRIVE_KI        0.0
#define BALANCE_DRIVE_KD        0.0
#define BALANCE_TURN_KP         180.0
#define BALANCE_TURN_KI         0.0
#define BALANCE_TURN_KD         0.0
#define BALANCE_DRIVE_TOLERANCE 0.10
#define BALANCE_TURN_TOLERANCE  0.10

#define SERVO_RANGE             180.0
#define ARM_LOWERED             50.0
#define ARM_LOW_DISPENSE        63.0
#define ARM_MID_DISPENSE        77.0
#define ARM_HI_DISPENSE         96.0
#define ARM_RAISED              180.0
#define SCOOPER_RAISED          10.0
#define SCOOPER_MID             50.0
#define SCOOPER_LOWERED         135.0
#define WRIST_HORIZONTAL        5.0
#define WRIST_VERTICAL          90.0
#define DISPENSE_PUSH           100
#define DISPENSE_PULL           -100

#define SERVO_POS_PER_DEGREE    ((MAX_SERVO_VALUE - MIN_SERVO_VALUE)/ \
                                 SERVO_RANGE)
#define LeftArmAngle(a)         (a)
#define RightArmAngle(a)        (SERVO_RANGE - (a))
#define SetArmAngle(a)          { \
                                    ServoSetAngle(g_ArmLeft, \
                                                  LeftArmAngle(a)); \
                                    ServoSetAngle(g_ArmRight, \
                                                  RightArmAngle(a)); \
                                }

#endif  //ifndef _ROBOTINFO_H
