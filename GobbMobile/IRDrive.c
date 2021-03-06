#pragma config(Sensor, S1,     irSeeker,            sensorI2CCustom)
#pragma config(Sensor, S4,     sonar,               sensorSONAR)
#pragma config(Motor,  motorA,          motorRight,    tmotorNormal, openLoop, encoder)
#pragma config(Motor,  motorC,          motorLeft,     tmotorNormal, openLoop, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="IRDrive.c" />
///
/// <summary>
///     This module contains the main code.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#include "joystickdriver.c"
#include "..\HTDriversV1.6\drivers\HTIRS2-driver.h"
#include "..\trclib\trcdefs.h"
#include "..\trclib\dbgtrace.h"
#include "..\trclib\batt.h"
#include "..\trclib\pidctrl.h"
#include "..\trclib\drive.h"
DRIVE   g_Drive[1];
PIDCTRL g_PIDCtrl[2];
#include "..\trclib\piddrive.h"
#include "GobbInfo.h"

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_MAIN

//
// Trace info.
//
#define TRACE_MODULES           (MOD_MAIN)
#define TRACE_LEVEL             TASK
#define MSG_LEVEL               INFO

//
// Global data.
//
BATT     g_Batt;
PIDDRIVE g_IRDrive;

/**
 *  This function provides the input value for various PID controllers.
 *
 *  @param pidCtrlID Specifies the PID Controller ID.
 *
 *  @return Returns the input value for the PID controller.
 */
float
PIDCtrlGetInput(
    __in int pidCtrlID
    )
{
    static float prevValue = 0.0;
    float inputValue = 0.0;
    int acS[5];
    int idx;

    TFuncName("PIDCtrlGetInput");
    TLevel(CALLBK);
    TEnterMsg(("ID=%d", pidCtrlID));

    switch (pidCtrlID)
    {
        case 0:
            inputValue = (float)SensorRaw[sonar]*SONAR_DISTANCE_INCHES;
            break;

        case 1:
            idx = HTIRS2readACDir(irSeeker);
            inputValue = (float)idx;
            if (idx == 0)
            {
                inputValue = prevValue;
            }
            else if (HTIRS2readAllACStrength(irSeeker,
                                             acS[0],
                                             acS[1],
                                             acS[2],
                                             acS[3],
                                             acS[4]))
            {
                idx = (idx - 1)/2;
                if ((idx < 4) && (acS[idx] != 0) && (acS[idx + 1] != 0))
                {
                    inputValue += (float)(acS[idx + 1] - acS[idx])/
                                  max(acS[idx], acS[idx + 1]);
                }
                nxtDisplayTextLine(2, "S1=%d,S2=%d", acS[0], acS[1]);
                nxtDisplayTextLine(3, "S3=%d,S4=%d", acS[2], acS[3]);
                nxtDisplayTextLine(4, "S5=%d,idx=%d", acS[4], idx);
            }
            prevValue = inputValue;
            break;
    }
    int nLineNum = (pidCtrlID==0)? 0: 1;
    nxtDisplayTextLine(nLineNum, "%d:In=%f", pidCtrlID, inputValue);

    TExitMsg(("=%5.1f", inputValue));
    return inputValue;
}   //PIDCtrlGetInput

/**
 *  This function handles the drive notification events.
 *
 *  @param drive Points to the DRIVE structure that generated the event.
 */
void
PIDDriveEvent(
    __in PIDDRIVE &pidDrive
    )
{
    TFuncName("PIDDriveEvent");
    TLevel(EVENT);
    TEnter();

    TExit();
    return;
}   //PIDDriveEvent

/**
 *  This function initializes the robot and its subsystems.
 */
void
RobotInit()
{
    TFuncName("RobotInit");
    TLevel(INIT);
    TEnter();

    BattInit(g_Batt, 7, false);
    //
    // Initialize all sensors.
    //
    HTIRS2setDSPMode(irSeeker, DSP_1200);

    //
    // Initialize the PID Drive subsystem.
    //
    DriveInit(g_Drive[0],
              motorLeft,
              motorRight,
              DISTANCE_PER_CLICK,
              DEGREES_PER_CLICK);
    PIDCtrlInit(g_PIDCtrl[0], 0,
                SONAR_DRIVE_KP, SONAR_DRIVE_KI, SONAR_DRIVE_KD,
                MOTOR_MIN_VALUE, MOTOR_MAX_VALUE,
                PIDCTRLF_INVERSE | PIDCTRLF_ABS_SETPOINT);
    PIDCtrlInit(g_PIDCtrl[1], 1,
                IRSEEK_TURN_KP, IRSEEK_TURN_KI, IRSEEK_TURN_KD,
                MOTOR_MIN_VALUE, MOTOR_MAX_VALUE,
                PIDCTRLF_INVERSE | PIDCTRLF_ABS_SETPOINT);
    PIDDriveInit(g_IRDrive,
                 0, //g_Drive,
                 0, //g_SonarPIDCtrl,
                 1, //g_IRPIDCtrl,
                 0);

    TExit();
    return;
}   //RobotInit

/**
 *  This function processes all the high frequency tasks that needs to run
 *  more often than other tasks such as sensor integration tasks.
 */
void
HiFreqTasks()
{
    TFuncName("HiFreqTasks");
    TLevel(TASK);
    TEnter();


    TExit();
    return;
}   //HiFreqTasks

/**
 *  This function processes all the input tasks.
 */
void
InputTasks()
{
    TFuncName("InputTasks");
    TLevel(TASK);
    TEnter();


    TExit();
    return;
}   //InputTasks

/**
 *  This function processes all the main tasks.
 */
void
MainTasks()
{
    TFuncName("MainTasks");
    TLevel(TASK);
    TEnter();



    TExit();
    return;
}   //MainTasks

/**
 *  This function processes all the output tasks. Output tasks are where all
 *  the actions are taking place. All other tasks are just changing states of
 *  various objects. There is no action taken until the output tasks.
 */
void
OutputTasks()
{
    TFuncName("OutputTasks");
    TLevel(TASK);
    TEnter();

    PIDDriveTask(g_IRDrive);
    DriveTask(g_Drive[0]);
    BattTask(g_Batt);

    TExit();
    return;
}   //OutputTasks

/**
 *  This task is the program entry point.
 */
task main()
{
    long nextTime;
    long currTime;

    TraceInit(TRACE_MODULES, TRACE_LEVEL, MSG_LEVEL);

    RobotInit();

    PIDDriveSetTarget(g_IRDrive,
                      6.0, SONAR_DRIVE_TOLERANCE,
                      2.0, IRSEEK_TURN_TOLERANCE,
                      false);

    nextTime = nPgmTime;
    while (true)
    {
        TPeriodStart();
        currTime = nPgmTime;
        HiFreqTasks();
        if (currTime >= nextTime)
        {
            nextTime = currTime + LOOP_TIME;

            InputTasks();
            MainTasks();
            OutputTasks();
        }
        TPeriodEnd();

        wait1Msec(1);
    }
}   //main
