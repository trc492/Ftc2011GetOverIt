#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TeleOpMain.h" />
///
/// <summary>
///     This module contains the main TeleOp tasks code.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#include "..\trclib\batt.h"
#include "..\trclib\joybtn.h"
#include "..\trclib\drive.h"
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
// Global data.
//
float   g_WristPos = WRIST_HORIZONTAL;
float   g_ArmPos = ARM_RAISED;
float   g_ScooperPos = SCOOPER_RAISED;
int     g_DispensePower = 0;
BATT    g_Batt;

//
// Input and sensors.
//
JOYBTN  g_JoyBtn1;
JOYBTN  g_JoyBtn2;

//
// Actuators.
//
DRIVE   g_Drive[1];
SERVO   g_Wrist;
SERVO   g_ArmLeft;
SERVO   g_ArmRight;
SERVO   g_Scooper;

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
            case Logitech_Btn1:
                break;

            default:
                break;
        }
    }
    else if (joybtn.joystickID == 2)
    {
        //
        // Make sure this is from joystick 2.
        //
        switch (joybtn.buttonMask)
        {
            case Logitech_Btn1:
                if (joybtn.fPressed)
                {
                    g_ArmPos = ARM_LOW_DISPENSE;
                    SetArmAngle(g_ArmPos);
                }
                break;

           case Logitech_Btn2:
                if (joybtn.fPressed)
                {
                    g_ArmPos = ARM_MID_DISPENSE;
                    SetArmAngle(g_ArmPos);
                }
                break;

            case Logitech_Btn3:
                if (joybtn.fPressed)
                {
                    g_ArmPos = ARM_HI_DISPENSE;
                    SetArmAngle(g_ArmPos);
                }
                break;

            case Logitech_Btn4:
                if (joybtn.fPressed)
                {
                    g_WristPos = (g_WristPos == WRIST_HORIZONTAL)?
                                  WRIST_VERTICAL: WRIST_HORIZONTAL;
                    ServoSetAngle(g_Wrist, g_WristPos);
                }
                break;

            case Logitech_LB5:
                if (joybtn.fPressed)
                {
                    g_ArmPos += 5.0;
                    if (g_ArmPos > SERVO_RANGE)
                    {
                       g_ArmPos = SERVO_RANGE;
                    }
                    SetArmAngle(g_ArmPos);
                }
                break;

            case Logitech_LB7:
                if (joybtn.fPressed)
                {
                    g_ArmPos -= 5.0;
                    if (g_ArmPos < 0.0)
                    {
                       g_ArmPos = 0.0;
                    }
                    SetArmAngle(g_ArmPos);
                }
                break;

            case Logitech_RB6:
                if (joybtn.fPressed)
                {
                    g_ScooperPos = SCOOPER_RAISED;
                    ServoSetAngle(g_Scooper, g_ScooperPos);
                }
                break;

            case Logitech_RB8:
                if (joybtn.fPressed)
                {
                    g_ScooperPos = SCOOPER_LOWERED;
                    ServoSetAngle(g_Scooper, g_ScooperPos);
                }
                break;

            case Logitech_Btn9:
                if (joybtn.fPressed)
                {
                    g_DispensePower = (g_DispensePower == 0)? DISPENSE_PULL:
                                                              0;
                    motor[motorDispense] = g_DispensePower;
                }
                break;

            case Logitech_Btn10:
                if (joybtn.fPressed)
                {
                    g_DispensePower = (g_DispensePower == 0)? DISPENSE_PUSH:
                                                              0;
                    motor[motorDispense] = g_DispensePower;
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

    TExit();
    return;
}   //ServoEvent

/**
 *  This function initializes the robot and its subsystems.
 */
void
RobotInit()
{
    TFuncName("RobotInit");
    TLevel(INIT);
    TEnter();

    BattInit(g_Batt, 5, true);
    //
    // Initialize the input subsystems.
    //
    JoyBtnInit(g_JoyBtn1, 1, JOYBTNF_ENABLE_EVENTS);
    JoyBtnInit(g_JoyBtn2, 2, JOYBTNF_ENABLE_EVENTS);

    //
    // Intialize the Drive subsystem of the robot running base.
    //
    DriveInit(g_Drive[0],
              motorLeft,
              motorRight,
              DISTANCE_PER_CLICK,
              DEGREES_PER_CLICK);

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

    JoyBtnTask(g_JoyBtn1);
    JoyBtnTask(g_JoyBtn2);

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

    int powerLeft  = NORMALIZE_DRIVE(DEADBAND_INPUT(joystick.joy1_y1),
                                     -128, 127);
    int powerRight = NORMALIZE_DRIVE(DEADBAND_INPUT(joystick.joy1_y2),
                                     -128, 127);
    //
    // TeleOp mode.
    //
    nxtDisplayTextLine(0, "Mode=TeleOp");
    nxtDisplayTextLine(1, "L=%d,R=%d", powerLeft, powerRight);
    DriveTank(g_Drive[0], powerLeft, powerRight);

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
    DriveTask(g_Drive[0]);
    BattTask(g_Batt);

    TExit();
    return;
}   //OutputTasks
