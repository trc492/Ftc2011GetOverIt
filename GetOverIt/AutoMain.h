#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="AutoMain.h" />
///
/// <summary>
///     This module contains the main autonomous tasks code.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#define _IRDRIVE_ENABLED
//#define _USE_NESTED_SM

#ifdef _IRDRIVE_ENABLED
#include "HTIRS2.h"
#endif
#include "..\trclib\batt.h"
#include "..\trclib\accel.h"
#include "..\trclib\gyro.h"
#include "..\trclib\touch.h"
#include "..\trclib\timer.h"
#include "..\trclib\sm.h"
#include "..\trclib\drive.h"
#include "..\trclib\pidctrl.h"
DRIVE   g_Drive[1];
#ifdef _IRDRIVE_ENABLED
PIDCTRL g_PIDCtrl[6];
#else
PIDCTRL g_PIDCtrl[5];
#endif
#include "..\trclib\piddrive.h"
#include "..\trclib\servo.h"

#pragma systemFile

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
// Game info.
//
#define STARTPOS_LEFT1          0
#define STARTPOS_LEFT2          1
#define STARTPOS_LEFT3          2
#define STARTPOS_RIGHT1         3
#define STARTPOS_RIGHT2         4
#define STARTPOS_RIGHT3         5

//
// State machine event types.
//
#define EVTTYPE_PIDDRIVE        (EVTTYPE_NONE + 1)
#define EVTTYPE_TIMER           (EVTTYPE_NONE + 2)
#define EVTTYPE_TOUCH           (EVTTYPE_NONE + 3)

//
// Global data.
//
int     g_StartPos = STARTPOS_LEFT1;
float   g_WristPos = WRIST_HORIZONTAL;
float   g_ArmPos = ARM_RAISED;
float   g_ScooperPos = SCOOPER_RAISED;
MENU    g_Menu;
BATT    g_Batt;

//
// Input and sensors.
//
ACCEL   g_Accel;
GYRO    g_Gyro;
TOUCH   g_Touch1;
TOUCH   g_Touch2;
TIMER   g_Timer;

//
// Drive subsystems.
//
PIDDRIVE g_EncoderDrive;
PIDDRIVE g_BalanceDrive;
#ifdef _IRDRIVE_ENABLED
PIDDRIVE g_IRDrive;
#endif

//
// Actuators.
//
SERVO   g_Wrist;
SERVO   g_ArmLeft;
SERVO   g_ArmRight;
SERVO   g_Scooper;

//
// State machines.
//
SM      g_AutoSM;
#ifdef _USE_NESTED_SM
SM      g_LowGoalSM;
SM      g_DispenseSM;
SM      g_ClimbBridgeSM;
#endif

#ifdef _USE_NESTED_SM
#include "LowGoal.h"    //Score batons into the low goal
#include "Dispense.h"   //Dispense batons
#include "ClimbBridge.h"//Climb onto the bridge
#endif
#include "Left1.h"      //Left routine with over the cliff (10 pt)
#include "Left2.h"      //Left routine with bridge balancing (15 pt)
#include "Left3.h"      //Left routine with crossing bridge (10 pt)
#include "Right1.h"     //Right routine with bridge balancing (25 pt)
//#include "Right2.h"     //Right routine with crossing mountain (20 pt)
#include "Right2A.h"     //Right routine with bridge balance (15 pt)
#include "Right3.h"     //Right routine with crossing bridge (20 pt)

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
#ifdef _IRDRIVE_ENABLED
    static float prevValue = 0.0;
    int acS[5];
    int idx;
#endif

    TFuncName("PIDCtrlGetInput");
    TLevel(CALLBK);
    TEnterMsg(("ID=%d", pidCtrlID));

    switch (pidCtrlID)
    {
        case 0:
            //
            // Encoder drive.
            //
            inputValue = (float)(nMotorEncoder[motorLeft] +
                                 nMotorEncoder[motorRight])*
                         DISTANCE_PER_CLICK/2.0;
            break;

        case 1:
            //
            // Encoder turn.
            //
            inputValue = (float)(nMotorEncoder[motorLeft] -
                                 nMotorEncoder[motorRight])*
                         DEGREES_PER_CLICK;
            break;

        case 2:
            //
            // Balance drive.
            //
            AccelGetX(g_Accel, inputValue);
            break;

        case 3:
            //
            // Balance turn.
            //
            AccelGetY(g_Accel, inputValue);
            break;

        case 4:
            //
            // Gyro turn.
            //
            inputValue = GyroGetHeading(g_Gyro);
            break;

#ifdef _IRDRIVE_ENABLED
        case 5:
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
#endif
    }

    TExitMsg(("=%5.1f", inputValue));
    return inputValue;
}   //PIDCtrlGetInput

/**
 *  This function handles the PID drive notification events.
 *
 *  @param drive Points to the PIDDRIVE structure that generated the event.
 */
void
PIDDriveEvent(
    __in PIDDRIVE &pidDrive
    )
{
    TFuncName("PIDDriveEvent");
    TLevel(EVENT);
    TEnter();

    if (IsSMEnabled(g_AutoSM))
    {
        SMSetEvent(g_AutoSM, EVTTYPE_PIDDRIVE, 0, 0, 0, 0);
#ifdef _USE_NESTED_SM
        SMSetEvent(g_LowGoalSM, EVTTYPE_PIDDRIVE, 0, 0, 0, 0);
        SMSetEvent(g_DispenseSM, EVTTYPE_PIDDRIVE, 0, 0, 0, 0);
        SMSetEvent(g_ClimbBridgeSM, EVTTYPE_PIDDRIVE, 0, 0, 0, 0);
#endif
    }

    TExit();
    return;
}   //PIDDriveEvent

/**
 *  This function handles the touch notification events.
 *
 *  @param touch Points to the TOUCH structure that generated the event.
 *  @param fActive Specifies the state of the touch sensor.
 */
void
TouchEvent(
    __in TOUCH &touch,
    __in bool fActive
    )
{
    TFuncName("TouchEvent");
    TLevel(EVENT);
    TEnter();

    if (IsSMEnabled(g_AutoSM))
    {
        SMSetEvent(g_AutoSM, EVTTYPE_TOUCH, 0, 0, 0, 0);
#ifdef _USE_NESTED_SM
        SMSetEvent(g_LowGoalSM, EVTTYPE_TOUCH, 0, 0, 0, 0);
        SMSetEvent(g_DispenseSM, EVTTYPE_TOUCH, 0, 0, 0, 0);
        SMSetEvent(g_ClimbBridgeSM, EVTTYPE_TOUCH, 0, 0, 0, 0);
#endif
    }
    PlayTone(fActive? 800: 400, 15);

    TExit();
    return;
}   //TouchEvent

/**
 *  This function handles the timer notification events.
 *
 *  @param timer Points to the TIMER structure that generated the event.
 */
void
TimerEvent(
    __in TIMER &timer
    )
{
    TFuncName("TimerEvent");
    TLevel(EVENT);
    TEnter();

    if (IsSMEnabled(g_AutoSM))
    {
        SMSetEvent(g_AutoSM, EVTTYPE_TIMER, 0, 0, 0, 0);
#ifdef _USE_NESTED_SM
        SMSetEvent(g_LowGoalSM, EVTTYPE_TIMER, 0, 0, 0, 0);
        SMSetEvent(g_DispenseSM, EVTTYPE_TIMER, 0, 0, 0, 0);
        SMSetEvent(g_ClimbBridgeSM, EVTTYPE_TIMER, 0, 0, 0, 0);
#endif
    }

    TExit();
    return;
}   //TimerEvent

#if 0
/**
 *  This function handles the servo notification events.
 *
 *  @param serv Points to the SERVO structure that generated the event.
 */
void
ServoEvent(
    __in SERVO &serv
    )
{
    TFuncName("ServoEvent");
    TLevel(EVENT);
    TEnterMsg(("motor=%d", serv.servoMotor));

    if (IsSMEnabled(g_AutoSM))
    {
        SMSetEvent(g_AutoSM, EVTTYPE_SERVO, serv.servoMotor, 0, 0, 0);
#ifdef _USE_NESTED_SM
        SMSetEvent(g_LowGoalSM, EVTTYPE_SERVO, 0, 0, 0, 0);
        SMSetEvent(g_DispenseSM, EVTTYPE_SERVO, 0, 0, 0, 0);
        SMSetEvent(g_ClimbBridgeSM, EVTTYPE_SERVO, 0, 0, 0, 0);
#endif
    }

    TExit();
    return;
}   //ServoEvent
#endif

/**
 *  This function implements the state machine for autonomous mode.
 *
 *  @param sm Points to the SM structure.
 */
void
AutonomousSM(
    __inout SM &sm
    )
{
    TFuncName("AutonomousSM");
    TLevel(TASK);
    TEnter();

    if (SMIsReady(sm))
    {
        switch (g_StartPos)
        {
            case STARTPOS_LEFT1:
                AutoLeft1(sm);
                break;

            case STARTPOS_LEFT2:
                AutoLeft2(sm);
                break;

            case STARTPOS_LEFT3:
                AutoLeft3(sm);
                break;

            case STARTPOS_RIGHT1:
                AutoRight1(sm);
                break;

            case STARTPOS_RIGHT2:
                AutoRight2(sm);
                break;

            case STARTPOS_RIGHT3:
                AutoRight3(sm);
                break;

            default:
                TErr(("Invalid StartPos"));
                break;
        }
    }

    TExit();
    return;
}   //AutonomousSM

/**
 *  This function initializes the robot and its subsystems.
 */
void
RobotInit()
{
    TFuncName("RobotInit");
    TLevel(INIT);
    TEnter();

#ifdef HTSMUX_STATUS
    //
    // Initialize Sensor MUX.
    //
    HTSMUXinit();
    HTSMUXscanPorts(HTSMUX);
#endif

    BattInit(g_Batt, 5, true);
    //
    // Init choice menu.
    //
    MenuInit(g_Menu, "Start Position:", 0, STARTPOS_RIGHT1);
    MenuAddChoice(g_Menu, "L1:XCliff");
    MenuAddChoice(g_Menu, "L2:BalBridge");
    MenuAddChoice(g_Menu, "L3:XBridge");
    MenuAddChoice(g_Menu, "R1:BalBridge");
    MenuAddChoice(g_Menu, "R2:XMoutain");
    MenuAddChoice(g_Menu, "R3:XBridge");
    g_StartPos = MenuGetChoice(g_Menu);

    eraseDisplay();
    nxtDisplayCenteredBigTextLine(3,(g_StartPos == STARTPOS_LEFT1)?
                                       "LEFT 1":
                                   (g_StartPos == STARTPOS_LEFT2)?
                                       "LEFT 2":
                                   (g_StartPos == STARTPOS_LEFT3)?
                                       "LEFT 3":
                                   (g_StartPos == STARTPOS_RIGHT1)?
                                       "RIGHT 1":
                                   (g_StartPos == STARTPOS_RIGHT2)?
                                       "RIGHT 2": "RIGHT 3");

    //
    // Initialize the input subsystems.
    //
#ifdef _IRDRIVE_ENABLED
//    HTIRS2setDSPMode(irSeeker, DSP_1200);
#endif
    AccelInit(g_Accel, accel, 0);
    GyroInit(g_Gyro, gyro, 0);
    TouchInit(g_Touch1, touch1, TOUCHF_HTSMUX | TOUCHF_ENABLE_EVENTS);
    TouchInit(g_Touch2, touch2, TOUCHF_HTSMUX | TOUCHF_ENABLE_EVENTS);
    TimerInit(g_Timer, TIMERF_ENABLE_EVENTS);

    //
    // Intialize the Drive subsystem of the robot running base.
    //
    DriveInit(g_Drive[0],
              motorLeft,
              motorRight,
              DISTANCE_PER_CLICK,
              DEGREES_PER_CLICK);

    //
    // Encoder Drive.
    //
    PIDCtrlInit(g_PIDCtrl[0], 0,
                ENCODER_DRIVE_KP, ENCODER_DRIVE_KI, ENCODER_DRIVE_KD,
                MOTOR_MIN_VALUE, MOTOR_MAX_VALUE,
                0);
    PIDCtrlInit(g_PIDCtrl[1], 1,
                ENCODER_TURN_KP, ENCODER_TURN_KI, ENCODER_TURN_KD,
                MOTOR_MIN_VALUE, MOTOR_MAX_VALUE,
                0);
    PIDCtrlInit(g_PIDCtrl[4], 1,
                GYRO_TURN_KP, GYRO_TURN_KI, GYRO_TURN_KD,
                MOTOR_MIN_VALUE, MOTOR_MAX_VALUE,
                0);
    PIDDriveInit(g_EncoderDrive,
                 0,     //Drive
                 0,     //EncoderDrive
                 4,     //GyroTurn
                 PIDDRIVEF_ENABLE_EVENTS);

    //
    // Balance Drive.
    //
    PIDCtrlInit(g_PIDCtrl[2], 2,
                BALANCE_DRIVE_KP, BALANCE_DRIVE_KI, BALANCE_DRIVE_KD,
                -50, 50,
                PIDCTRLF_INVERSE | PIDCTRLF_ABS_SETPOINT);
    PIDCtrlInit(g_PIDCtrl[3], 3,
                BALANCE_TURN_KP, BALANCE_TURN_KI, BALANCE_TURN_KD,
                -50, 50,
                PIDCTRLF_ABS_SETPOINT);
    PIDDriveInit(g_BalanceDrive,
                 0, //g_Drive,
                 2, //BalanceDrive,
                 3, //BalanceTurn,
                 0);

#ifdef _IRDRIVE_ENABLED
    //
    // IRSeeker Drive.
    //
    PIDCtrlInit(g_PIDCtrl[5], 5,
                IRSEEK_TURN_KP, IRSEEK_TURN_KI, IRSEEK_TURN_KD,
                MOTOR_MIN_VALUE, MOTOR_MAX_VALUE,
                PIDCTRLF_INVERSE | PIDCTRLF_ABS_SETPOINT);
    PIDDriveInit(g_IRDrive,
                 0, //g_Drive,
                 0, //EncoderDrive,
                 5, //IRTurn,
                 0);
#endif

    //
    // Initialize subsystems.
    //
    ServoInit(g_ArmLeft,
              servoArmLeft,
              SERVO_POS_PER_DEGREE,
              LeftArmAngle(g_ArmPos));
    ServoInit(g_ArmRight,
              servoArmRight,
              SERVO_POS_PER_DEGREE,
              RightArmAngle(g_ArmPos));
    wait1Msec(500);
    ServoInit(g_Wrist,
              servoWrist,
              SERVO_POS_PER_DEGREE,
              g_WristPos);
    ServoInit(g_Scooper,
              servoScooper,
              SERVO_POS_PER_DEGREE,
              g_ScooperPos);

    //
    // Initialize the Autonomous state machine.
    //
    SMInit(g_AutoSM);
#ifdef _USE_NESTED_SM
    SMInit(g_LowGoalSM);
    SMInit(g_DispenseSM);
    SMInit(g_ClimbBridgeSM);
#endif
    SMStart(g_AutoSM);

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

    GyroTask(g_Gyro);

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

    TouchTask(g_Touch1);
    TouchTask(g_Touch2);
    TimerTask(g_Timer);

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

    if (IsSMEnabled(g_AutoSM))
    {
        //
        // Autonomous mode.
        //
        nxtDisplayTextLine(0, "Auto=%d", g_AutoSM.currState);
        AutonomousSM(g_AutoSM);
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

    //
    // The Drive task programs the drive motors and set the robot into
    // action.
    //
    PIDDriveTask(g_EncoderDrive);
    PIDDriveTask(g_BalanceDrive);
#ifdef _IRDRIVE_ENABLED
    PIDDriveTask(g_IRDrive);
#endif
    DriveTask(g_Drive[0]);
#if 0
    ServoTask(g_ArmLeft);
    ServoTask(g_ArmRight);
    ServoTask(g_Wrist);
    ServoTask(g_Scooper);
#endif
    nxtDisplayTextLine(1, "L=%d,R=%d", g_Drive.leftPower, g_Drive.rightPower);
    BattTask(g_Batt);

    TExit();
    return;
}   //OutputTasks
