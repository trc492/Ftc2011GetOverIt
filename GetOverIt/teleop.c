#pragma config(Hubs,  S1, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S2,     accel,               sensorI2CCustom)
#pragma config(Sensor, S3,     gyro,                sensorI2CCustom)
#pragma config(Sensor, S4,     HTSMUX,              sensorI2CCustom)
#pragma config(Motor,  motorB,          motorDispense, tmotorNormal, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     motorLeft,     tmotorNormal, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motorRight,    tmotorNormal, openLoop, encoder)
#pragma config(Servo,  srvo_S1_C2_1,    servoWrist,           tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    servoArmLeft,         tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    servoArmRight,        tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    servoScooper,         tServoStandard)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TeleOp.c" />
///
/// <summary>
///     This module contains the entry point for teleoperator mode.
///     This is template file and should not be modified. The main competition
///     code should live in TeleOpMain.h.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#include "JoystickDriver.c"
#include "..\trclib\trcdefs.h"
#include "..\trclib\dbgtrace.h"
#include "RobotInfo.h"
#include "TeleOpMain.h"

/**
 *  This task is the program entry point.
 */
task main()
{
    long nextTime;
    long currTime;

    TraceInit(TRACE_MODULES, TRACE_LEVEL, MSG_LEVEL);

    //
    // The RobotInit function is provided by student code in main.h.
    //
    RobotInit();

    //
    // Wait for the beginning of teleop mode.
    //
    waitForStart();

    nextTime = nPgmTime;
    while (true)
    {
        currTime = nPgmTime;
        HiFreqTasks();
        if (currTime >= nextTime)
        {
            nextTime = currTime + LOOP_TIME;
            getJoystickSettings(joystick);

            //
            // The following functions are provided by student code in main.h.
            //
            InputTasks();
            MainTasks();
            OutputTasks();
        }

        wait1Msec(1);
    }
}   //main
