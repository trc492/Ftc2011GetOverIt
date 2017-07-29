#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="GobbInfo.h" />
///
/// <summary>
///     This module contains the GobbMobile information constants.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _GOBBINFO_H
#define _GOBBINFO_H

//
// Drive info.
//
#define WHEEL_CIRCUMFERENCE     6.92    // in inches
#define CLICKS_PER_REVOLUTION   360.0
#define DISTANCE_PER_CLICK      (WHEEL_CIRCUMFERENCE/CLICKS_PER_REVOLUTION)
#define WHEELBASE_DISTANCE      8.50    // in inches
#define DEGREES_PER_CLICK       (360.0/(PI*WHEELBASE_DISTANCE/ \
                                        DISTANCE_PER_CLICK))
#define SONAR_DISTANCE_INCHES   (1/2.54)

#define ENCODER_DRIVE_KP        50.0
#define ENCODER_DRIVE_KI        0.0
#define ENCODER_DRIVE_KD        0.0
#define ENCODER_DRIVE_TOLERANCE 1.0

#define ENCODER_TURN_KP         4.0
#define ENCODER_TURN_KI         0.0
#define ENCODER_TURN_KD         0.0
#define ENCODER_TURN_TOLERANCE  1.0

#define GYRO_TURN_KP            2.0
#define GYRO_TURN_KI            0.0
#define GYRO_TURN_KD            0.0
#define GYRO_TURN_TOLERANCE     2.0

#define BALANCE_DRIVE_KP        1000.0
#define BALANCE_DRIVE_KI        0.0
#define BALANCE_DRIVE_KD        0.0
#define BALANCE_TURN_KP         1000.0
#define BALANCE_TURN_KI         0.0
#define BALANCE_TURN_KD         0.0
#define BALANCE_DRIVE_TOLERANCE 0.10
#define BALANCE_TURN_TOLERANCE  0.10

#define LNFOLLOW_TURN_KP        5.0
#define LNFOLLOW_TURN_KI        0.0
#define LNFOLLOW_TURN_KD        0.0
#define LNFOLLOW_DRIVE_KP       0.5

#define SONAR_DRIVE_KP          10.0
#define SONAR_DRIVE_KI          0.0
#define SONAR_DRIVE_KD          0.0
#define SONAR_DRIVE_TOLERANCE   0.5
#define IRSEEK_TURN_KP          20.0
#define IRSEEK_TURN_KI          0.0
#define IRSEEK_TURN_KD          0.0
#define IRSEEK_TURN_TOLERANCE   0.1

//
// Grabber info.
//
#define GRABBER_CAL_POWER       20
#define GRABBER_MAX_POWER       30
#define GRABBER_MIN_POS         10
#define GRABBER_MAX_POS         70
#define GRABBER_TIME_STEP       50

//
// Sensor mask.
//
//#define THRESHOLD_LO_LIGHT      341
//#define THRESHOLD_HI_LIGHT      682
#define THRESHOLD_LO_LIGHT      1023
#define THRESHOLD_HI_LIGHT      0
#define THRESHOLD_LO_SONAR      40
#define THRESHOLD_HI_SONAR      50

#endif  //ifndef _GOBBINFO_H
