#pragma config(Sensor, S1,     lightRight,          sensorLightActive)
#pragma config(Sensor, S2,     lightCenter,         sensorLightActive)
#pragma config(Sensor, S3,     lightLeft,           sensorLightActive)
#pragma config(Motor,  motorA,          motorRight,    tmotorNormal, openLoop, encoder)
#pragma config(Motor,  motorC,          motorLeft,     tmotorNormal, openLoop, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="LnFollower.c" />
///
/// <summary>
///     This module contains the main code.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#include "JoystickDriver.c"
#include "..\trclib\trcdefs.h"
#include "..\trclib\dbgtrace.h"
#include "..\trclib\batt.h"
#include "..\trclib\joybtn.h"
#include "..\trclib\sensor.h"
#include "..\trclib\sm.h"
#include "..\trclib\pidctrl.h"
#include "..\trclib\drive.h"
DRIVE   g_Drive[1];
PIDCTRL g_PIDCtrl[2];
#include "..\trclib\lnfollow.h"
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

#define NUM_LIGHT_SENSORS       3

//
// Global data.
//
BATT     g_Batt;
JOYBTN   g_JoyBtn;
LNFOLLOW g_LnFollow;
SM       g_LnFollowSM;

/**
 *  This function handles the joystick button notification events.
 *
 *  @param joybtn Points to the JOYBTN structure that generated the event.
 */
void
JoyBtnEvent(
    __in JOYBTN &joybtn
    )
{
    TFuncName("JoyBtnEvent");
    TLevel(EVENT);
    TEnterMsg(("Button=%x,On=%d", joybtn.buttonMask, (byte)joybtn.fPressed));

    if (joybtn.joystickID == 1)
    {
        switch (joybtn.buttonMask)
        {
            case Xbox_A:
	            if (joybtn.fPressed)
	            {
	                if (IsSMDisabled(g_LnFollowSM))
	                {
	                    SMStart(g_LnFollowSM);
	                }
	                else
	                {
	                    LnFollowStart(g_LnFollow, false);
	                    SMStop(g_LnFollowSM);
	                }
	            }
                break;

            case Xbox_B:
	            if (joybtn.fPressed)
                {
                    //
                    // When calibrating the light sensors for the Line
                    // Follower, start calibration mode and drive the robot
                    // around to sample the brightest white and the darkest
                    // black on the floor. Then stop the calibration so that
                    // it can compute the low and high thresholds for each
                    // light sensor.
                    //
                    if (LnFollowCalibrating(g_LnFollow))
                    {
                        //
                        // We were in calibration mode, we are done.
                        //
                        LnFollowCal(g_LnFollow, false);
                        nxtDisplayTextLine(0, "LnFollowMode=Off");
#if 0
                        nxtDisplayTextLine(3, "LLo=%d,LHi=%d",
                                           g_LnFollow.LightSensors[0].lowThreshold,
                                           g_LnFollow.LightSensors[0].highThreshold);
                        nxtDisplayTextLine(4, "CLo=%d,CHi=%d",
                                           g_LnFollow.LightSensors[1].lowThreshold,
                                           g_LnFollow.LightSensors[1].highThreshold);
                        nxtDisplayTextLine(5, "RLo=%d,RHi=%d",
                                           g_LnFollow.LightSensors[2].lowThreshold,
                                           g_LnFollow.LightSensors[2].highThreshold);
#endif
                    }
                    else
                    {
                        //
                        // We weren't in calibration mode, start it.
                        //
                        LnFollowCal(g_LnFollow, true);
                    }
                }
                break;

            default:
	            break;
  	    }
    }

	TExit();
    return;
}   //JoyBtnEvent

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
    float inputValue = 0.0;

    TFuncName("PIDCtrlGetInput");
    TLevel(CALLBK);
    TEnterMsg(("ID=%d", pidCtrlID));

    switch (pidCtrlID)
    {
        case 0:
            //
            // LnFollow Turn.
            //
            inputValue = (float)g_LnFollow.sensorValue;
            break;
    }
    nxtDisplayTextLine(1, "In=%f", inputValue);

    TExitMsg(("=%5.1f", inputValue));
    return inputValue;
}   //PIDCtrlGetInput

/**
 *  This function handles the sensor notification events.
 *
 *  @param sensor Points to the SESNOR structure that generated the event.
 */
void
SensorEvent(
    __in SENSOR &sensor
    )
{
    TFuncName("SensorEvent");
    TLevel(EVENT);
    TEnterMsg(("Sensor=%d,Zone=%d", sensor.sensorID, sensor.sensorZone));

    //
    // Currently unused but we must declare this function since the sensor
    // module will call it.
    //
#if 0
    switch (sensor.sensorID)
    {
        default:
            break;
    }
#endif

    TExit();
    return;
}   //SensorEvent

/**
 *  This function implements the state machine for line follow mode.
 *
 *  @param sm Points to the SM structure.
 */
void
LnFollowSM(
    __inout SM &smLnFollow
    )
{
    TFuncName("LnFollowSM");
    TLevel(TASK);
    TEnter();

    if (SMIsReady(smLnFollow))
    {
        //
        // We only execute the autonomous state machine if it is not in wait
        // mode.
        //
        switch (smLnFollow.currState)
        {
            case SMSTATE_STARTED:
                if (g_LnFollow.sensorValue == 0)
                {
                    //
                    // Don't see the line, drive forward.
                    //
                    DriveArcade(g_Drive[0], 50.0, 0.0);
                }
                else
                {
                    //
                    // Found the line, follow it.
                    //
                    LnFollowStart(g_LnFollow, true);
                    smLnFollow.currState++;
                }
                break;

            case SMSTATE_STARTED + 1:
                LnFollowTask(g_LnFollow);
                break;

            default:
                break;
        }
    }

    TExit();
    return;
}   //LnFollowSM

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
    // Initialize joystick buttons.
    //
    JoyBtnInit(g_JoyBtn, 1, JOYBTNF_ENABLE_EVENTS);

    //
    // Initialize the light sensors.
    //
    SensorInit(g_LnFollow.LightSensors[0],
               lightLeft,
               THRESHOLD_LO_LIGHT,
               THRESHOLD_HI_LIGHT,
               SENSORF_INVERSE);
    SensorInit(g_LnFollow.LightSensors[1],
               lightCenter,
               THRESHOLD_LO_LIGHT,
               THRESHOLD_HI_LIGHT,
               SENSORF_INVERSE);
    SensorInit(g_LnFollow.LightSensors[2],
               lightRight,
               THRESHOLD_LO_LIGHT,
               THRESHOLD_HI_LIGHT,
               SENSORF_INVERSE);
    //
    // Initialize the LnFollow Drive subsystem.
    //
    DriveInit(g_Drive[0],
              motorLeft,
              motorRight,
              DISTANCE_PER_CLICK,
              DEGREES_PER_CLICK);
    PIDCtrlInit(g_PIDCtrl[0], 0,
                LNFOLLOW_TURN_KP, LNFOLLOW_TURN_KI, LNFOLLOW_TURN_KD,
                MOTOR_MIN_VALUE, MOTOR_MAX_VALUE,
                PIDCTRLF_ABS_SETPOINT);
    LnFollowInit(g_LnFollow,
                 0, //g_Drive,
                 0, //LnFollowDrive,
                 NUM_LIGHT_SENSORS,
                 LNFOLLOW_DRIVE_KP);

    //
    // Initialize the LnFollow state machine.
    //
    SMInit(g_LnFollowSM);

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

    getJoystickSettings(joystick);
    JoyBtnTask(g_JoyBtn);
    nxtDisplayTextLine(3, "          ");
    nxtDisplayTextLine(4, "          ");
    nxtDisplayTextLine(5, "          ");
    SensorTask(g_LnFollow.LightSensors[0]);
    SensorTask(g_LnFollow.LightSensors[1]);
    SensorTask(g_LnFollow.LightSensors[2]);

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

    if (IsSMEnabled(g_LnFollowSM))
    {
        //
        // Line following mode.
        //
        nxtDisplayTextLine(0, "LnFollow:State=%d", g_LnFollowSM.currState);
        LnFollowSM(g_LnFollowSM);
    }
    else
    {
        int powerDrive = NORMALIZE_DRIVE(DEADBAND_INPUT(joystick.joy1_y1),
                                         -128, 127);
        int powerTurn = NORMALIZE_DRIVE(DEADBAND_INPUT(joystick.joy1_x1),
                                        -128, 127);
        //
        // TeleOp or Calibrate mode.
        //
        if (LnFollowCalibrating(g_LnFollow))
        {
            nxtDisplayTextLine(0, "LnFollowMode:Cal");
            nxtDisplayTextLine(1, "L=%d,R=%d",
                               SensorRaw[lightLeft],
                               SensorRaw[lightRight]);
            nxtDisplayTextLine(2, "C=%d", SensorRaw[lightCenter]);
        }
        else
        {
            nxtDisplayTextLine(0, "Mode=TeleOp");
            nxtDisplayTextLine(1, "D=%d,T=%d", powerDrive, powerTurn);
        }
        DriveArcade(g_Drive[0], powerDrive, powerTurn);
    }

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
