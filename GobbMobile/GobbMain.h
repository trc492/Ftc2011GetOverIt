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

#include "..\trclib\batt.h"
#include "..\trclib\accel.h"
#include "..\trclib\gyro.h"
#include "..\trclib\timer.h"
#include "..\trclib\sm.h"
#include "..\trclib\drive.h"
#include "..\trclib\pidctrl.h"
DRIVE   g_Drive[1];
PIDCTRL g_PIDCtrl[5];
#include "..\trclib\piddrive.h"

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

#define _USE_NESTED_SM

//
// Game info.
//
#define STARTPOS_LEFT1          0
#define STARTPOS_LEFT2          1
#define STARTPOS_RIGHT1         2
#define STARTPOS_RIGHT2         3
#define STARTPOS_RIGHT3         4

//
// State machine event types.
//
#define EVTTYPE_PIDDRIVE        (EVTTYPE_NONE + 1)
#define EVTTYPE_TIMER           (EVTTYPE_NONE + 2)

//
// Global data.
//
int     g_StartPos = STARTPOS_LEFT1;
BATT    g_Batt;
MENU    g_Menu;

//
// Input and sensors.
//
ACCEL   g_Accel;
GYRO    g_Gyro;
TIMER   g_Timer;

//
// Drive subsystems.
//
PIDDRIVE g_EncoderDrive;
PIDDRIVE g_BalanceDrive;

//
// State machines.
//
SM      g_AutoSM;
#ifdef _USE_NESTED_SM
SM      g_LowGoalSM;
SM      g_DispenseSM;
SM      g_ClimbBridgeSM;
#endif

/**
 *  This function implements the autonomous routine to dump batons to the low
 *  goal.
 *
 *  @param sm Points to the SM structure.
 */
void
LowGoal(
  __inout SM &sm
  )
{
    TFuncName("LowGoal");
    TLevel(TASK);
    TEnter();

    if (SMIsReady(sm))
    {
        switch (sm.currState)
        {
#if 0
            case SMSTATE_STARTED:
                // Lower scooper for 2 seconds.
                ServoSetAngle(g_Scooper, SCOOPER_LOWERED);
                TimerSet(g_Timer, 2000);
                SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 1:
                // Raise scooper.
                ServoSetAngle(g_Scooper, SCOOPER_RAISED);
                SMAddWaitEvent(sm, EVTTYPE_SERVO, servoScooper, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 2:
                // Lower arm.
                SetArmAngle(ARM_LOWERED);
                SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmLeft, -1);
                SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmRight, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 3:
                // Unload the batons for 2 seconds.
                motor[motorDispense] = DISPENSE_PUSH;
                TimerSet(g_Timer, 2000);
                SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 4:
                // Stop dispense motor and raise arm.
                motor[motorDispense] = 0;
                SetArmAngle(ARM_RAISED);
                SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmLeft, -1);
                SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmRight, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;
#endif

            case SMSTATE_STARTED:
                // Drive backward 4 inches.
                PIDDriveSetTarget(g_EncoderDrive,
                                  -4.0, ENCODER_DRIVE_TOLERANCE,
                                  0.0, ENCODER_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 1:
                // Turn left 90 degrees.
                PIDDriveSetTarget(g_EncoderDrive,
                                  0.0, ENCODER_DRIVE_TOLERANCE,
                                  -90.0, ENCODER_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            default:
                SMStop(sm);
                break;
        }
    }

    TExit();
    return;
}   //LowGoal

/**
 *  This function implements the autonomous routine to dispense batons from
 *  the mid dispenser.
 *
 *  @param sm Points to the SM structure.
 */
void
Dispense(
  __inout SM &sm
  )
{
    TFuncName("Dispense");
    TLevel(TASK);
    TEnter();

    if (SMIsReady(sm))
    {
        switch (sm.currState)
        {
            case SMSTATE_STARTED:
                // Drive forward ~4 ft.
                PIDDriveSetTarget(g_EncoderDrive,
                                  49.0, ENCODER_DRIVE_TOLERANCE,
                                  0.0, ENCODER_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 1:
                // Turn right 90 degrees.
                PIDDriveSetTarget(g_EncoderDrive,
                                  0.0, ENCODER_DRIVE_TOLERANCE,
                                  90.0, ENCODER_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 2:
                // Drive forward 6 ft.
                PIDDriveSetTarget(g_EncoderDrive,
                                  72.0, ENCODER_DRIVE_TOLERANCE,
                                  0.0, ENCODER_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

#ifdef _IRDRIVE_ENABLED
            case SMSTATE_STARTED + 3:
                // Engage IR drive for ~4 ft or until touched.
                PIDCtrlSetPowerLimit(g_PIDCtrl[0], -30, 30);
                PIDCtrlSetPowerLimit(g_PIDCtrl[1], -30, 30);
                PIDDriveSetTarget(g_IRDrive,
                                  48.0, ENCODER_DRIVE_TOLERANCE,
                                  8.0, IRSEEK_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMAddWaitEvent(sm, EVTTYPE_TOUCH, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 4:
                // Drive backward 6 inches.
                PIDDriveReset(g_IRDrive);
                PIDCtrlSetPowerLimit(g_PIDCtrl[0], -50, 50);
                PIDCtrlSetPowerLimit(g_PIDCtrl[1], -50, 50);
                PIDDriveSetTarget(g_EncoderDrive,
                                  -7.0, ENCODER_DRIVE_TOLERANCE,
                                  0.0, ENCODER_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 5:
                // Lower arm to mid level.
                SetArmAngle(ARM_MID_DISPENSE);
                SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmLeft, -1);
                SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmRight, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 6:
                // Drive forward 3.5 inches.
                PIDDriveReset(g_IRDrive);
                PIDDriveSetTarget(g_EncoderDrive,
                                  3.5, ENCODER_DRIVE_TOLERANCE,
                                  0.0, ENCODER_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 7:
                // Dispense the batons.
                motor[motorDispense] = DISPENSE_PULL;
                TimerSet(g_Timer, 5000);
                SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 8:
                // Backoff 6 inches.
                motor[motorDispense] = 0;
                PIDDriveSetTarget(g_EncoderDrive,
                                  -6.0, ENCODER_DRIVE_TOLERANCE,
                                  0.0, ENCODER_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;
#endif

            default:
                SMStop(sm);
                break;
        }
    }

    TExit();
    return;
}   //Dispense

/**
 *  This function implements the autonomous routine to climb onto the bridge.
 *
 *  @param sm Points to the SM structure.
 */
void
ClimbBridge(
  __inout SM &sm
  )
{
    TFuncName("ClimbBridge");
    TLevel(TASK);
    TEnter();

    if (SMIsReady(sm))
    {
        switch (sm.currState)
        {
            case SMSTATE_STARTED:
                // Drive forward ~1 ft.
                PIDDriveSetTarget(g_EncoderDrive,
                                  15.0, ENCODER_DRIVE_TOLERANCE,
                                  0.0, ENCODER_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 1:
                // Turn right 90-degree.
                PIDDriveSetTarget(g_EncoderDrive,
                                  0.0, ENCODER_DRIVE_TOLERANCE,
                                  90.0, ENCODER_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 2:
                // Drive forward ~1 ft or until touch active.
                PIDCtrlSetPowerLimit(g_PIDCtrl[0], -30, 30);
                PIDCtrlSetPowerLimit(g_PIDCtrl[1], -30, 30);
                PIDDriveSetTarget(g_EncoderDrive,
                                  16.0, ENCODER_DRIVE_TOLERANCE,
                                  0.0, ENCODER_TURN_TOLERANCE,
                              true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
#if 0
                SMAddWaitEvent(sm, EVTTYPE_TOUCH, -1, -1);
#endif
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 3:
                // Stop previous drive and drive backward 3 inches.
#if 0
                PIDDriveReset(g_EncoderDrive);
#endif
                PIDDriveSetTarget(g_EncoderDrive,
                                  -3.0, ENCODER_DRIVE_TOLERANCE,
                                  0.0, ENCODER_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

#if 0
            case SMSTATE_STARTED + 4:
                // Lower scooper.
                ServoSetAngle(g_Scooper, SCOOPER_LOWERED);
                SMAddWaitEvent(sm, EVTTYPE_SERVO, servoScooper, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;
#endif

            case SMSTATE_STARTED + 5:
                // Drive forward ~3 ft.
                PIDCtrlSetPowerLimit(g_PIDCtrl[0], -50, 50);
                PIDCtrlSetPowerLimit(g_PIDCtrl[1], -50, 50);
                PIDDriveSetTarget(g_EncoderDrive,
                                  38.5, ENCODER_DRIVE_TOLERANCE,
                                  0.0, ENCODER_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            default:
                SMStop(sm);
                break;
        }
    }

    TExit();
    return;
}   //ClimbBridge

/**
 *  This function implements the autonomous mode with a start position on the
 *  left corner.
 *
 *  @param sm Points to the SM structure.
 */
void
AutoLeft1(
  __inout SM &sm
  )
{
    TFuncName("AutoLeft1");
    TLevel(TASK);
    TEnter();

    switch (sm.currState)
    {
        case SMSTATE_STARTED:
            // Drive forward ~9 ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              112.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 1:
            // Turn left 90 degrees.
            PIDDriveSetTarget(g_EncoderDrive,
                              0.0, ENCODER_DRIVE_TOLERANCE,
                              -90.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 2000, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 2:
            // Drive forward ~2 ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              22.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

#ifdef _USE_NESTED_SM
        //
        // Dump batons into low goal.
        //
        case SMSTATE_STARTED + 3:
            SMStart(g_LowGoalSM);
            sm.currState++;
            break;

        case SMSTATE_STARTED + 4:
            if (IsSMEnabled(g_LowGoalSM))
            {
                LowGoal(g_LowGoalSM);
            }
            else
            {
                sm.currState++;
            }
            break;

        //
        // Go dispense batons.
        //
        case SMSTATE_STARTED + 5:
            SMStart(g_DispenseSM);
            sm.currState++;
            break;

        case SMSTATE_STARTED + 6:
            if (IsSMEnabled(g_DispenseSM))
            {
                Dispense(g_DispenseSM);
            }
            else
            {
                sm.currState++;
            }
            break;

#else
        //
        // Dump batons into low goal.
        //
#if 0
        case SMSTATE_STARTED + 3:
            // Lower scooper for 2 seconds.
            ServoSetAngle(g_Scooper, SCOOPER_LOWERED);
            TimerSet(g_Timer, 2000);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 4:
            // Raise scooper.
            ServoSetAngle(g_Scooper, SCOOPER_RAISED);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoScooper, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 5:
            // Lower arm.
            SetArmAngle(ARM_LOWERED);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmLeft, -1);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmRight, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 6:
            // Unload the batons for 2 seconds.
            motor[motorDispense] = DISPENSE_PUSH;
            TimerSet(g_Timer, 2000);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 7:
            // Stop dispense motor and raise arm.
            motor[motorDispense] = 0;
            SetArmAngle(ARM_RAISED);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmLeft, -1);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmRight, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;
#endif

        case SMSTATE_STARTED + 3:
            // Drive backward 4 inches.
            PIDDriveSetTarget(g_EncoderDrive,
                              -4.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 4:
            // Turn left 90 degrees.
            PIDDriveSetTarget(g_EncoderDrive,
                              0.0, ENCODER_DRIVE_TOLERANCE,
                              -90.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        //
        // Go dispense batons.
        //
        case SMSTATE_STARTED + 5:
            // Drive forward ~4 ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              49.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 6:
            // Turn right 90 degrees.
            PIDDriveSetTarget(g_EncoderDrive,
                              0.0, ENCODER_DRIVE_TOLERANCE,
                              90.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 7:
            // Drive forward 6 ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              72.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

#ifdef _IRDRIVE_ENABLED
        case SMSTATE_STARTED + 8:
            // Engage IR drive for ~4 ft or until touched.
            PIDCtrlSetPowerLimit(g_PIDCtrl[0], -30, 30);
            PIDCtrlSetPowerLimit(g_PIDCtrl[1], -30, 30);
            PIDDriveSetTarget(g_IRDrive,
                              48.0, ENCODER_DRIVE_TOLERANCE,
                              8.0, IRSEEK_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMAddWaitEvent(sm, EVTTYPE_TOUCH, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 9:
            // Drive backward 6 inches.
            PIDDriveReset(g_IRDrive);
            PIDCtrlSetPowerLimit(g_PIDCtrl[0], -50, 50);
            PIDCtrlSetPowerLimit(g_PIDCtrl[1], -50, 50);
            PIDDriveSetTarget(g_EncoderDrive,
                              -7.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 10:
            // Lower arm to mid level.
            SetArmAngle(ARM_MID_DISPENSE);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmLeft, -1);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmRight, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 11:
            // Drive forward 3.5 inches.
            PIDDriveReset(g_IRDrive);
            PIDDriveSetTarget(g_EncoderDrive,
                              3.5, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 12:
            // Dispense the batons.
            motor[motorDispense] = DISPENSE_PULL;
            TimerSet(g_Timer, 5000);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 13:
            // Backoff 6 inches.
            motor[motorDispense] = 0;
            PIDDriveSetTarget(g_EncoderDrive,
                              -6.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;
#endif
#endif

        default:
            SMStop(sm);
            break;
    }

    TExit();
    return;
}   //AutoLeft1

/**
 *  This function implements the autonomous mode with a start position on the
 *  left corner.
 *
 *  @param sm Points to the SM structure.
 */
void
AutoLeft2(
  __inout SM &sm
  )
{
    TFuncName("AutoLeft2");
    TLevel(TASK);
    TEnter();

    switch (sm.currState)
    {
        case SMSTATE_STARTED:
            // Drive forward ~6.5 ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              80.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        default:
            SMStop(sm);
            break;
    }

    TExit();
    return;
}   //AutoLeft2

/**
 *  This function implements the autonomous mode with a start position on the
 *  right corner.
 *
 *  @param sm Points to the SM structure.
 */
void
AutoRight1(
  __inout SM &sm
  )
{
    float accelValue;

    TFuncName("AutoRight1");
    TLevel(TASK);
    TEnter();

    switch (sm.currState)
    {
        case SMSTATE_STARTED:
            // Drive forward 2- ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              20.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

#ifdef _USE_NESTED_SM
        //
        // Dump batons into low goal.
        //
        case SMSTATE_STARTED + 1:
            SMStart(g_LowGoalSM);
            sm.currState++;
            break;

        case SMSTATE_STARTED + 2:
            if (IsSMEnabled(g_LowGoalSM))
            {
                LowGoal(g_LowGoalSM);
            }
            else
            {
                sm.currState++;
            }
            break;

        //
        // Climb onto the bridge.
        //
        case SMSTATE_STARTED + 3:
            SMStart(g_ClimbBridgeSM);
            sm.currState++;
            break;

        case SMSTATE_STARTED + 4:
            if (IsSMEnabled(g_ClimbBridgeSM))
            {
                ClimbBridge(g_ClimbBridgeSM);
            }
            else
            {
                sm.currState++;
            }
            break;

#else
        //
        // Dump batons into low goal.
        //
#if 0
        case SMSTATE_STARTED + 1:
            // Lower scooper for 2 seconds.
            ServoSetAngle(g_Scooper, SCOOPER_LOWERED);
            TimerSet(g_Timer, 2000);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 2:
            // Raise scooper.
            ServoSetAngle(g_Scooper, SCOOPER_RAISED);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoScooper, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 3:
            // Lower arm.
            SetArmAngle(ARM_LOWERED);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmLeft, -1);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmRight, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 4:
            // Unload the batons for 2 seconds.
            motor[motorDispense] = DISPENSE_PUSH;
            TimerSet(g_Timer, 2000);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 5:
            // Stop dispense motor and raise arm.
            motor[motorDispense] = 0;
            SetArmAngle(ARM_RAISED);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmLeft, -1);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmRight, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;
#endif

        case SMSTATE_STARTED + 1:
            // Drive backward 4 inches.
            PIDDriveSetTarget(g_EncoderDrive,
                              -4.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 2:
            // Turn left 90 degrees.
            PIDDriveSetTarget(g_EncoderDrive,
                              0.0, ENCODER_DRIVE_TOLERANCE,
                              -90.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        //
        // Climb onto the bridge.
        //
        case SMSTATE_STARTED + 3:
            // Drive forward ~1 ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              15.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 4:
            // Turn right 90-degree.
            PIDDriveSetTarget(g_EncoderDrive,
                              0.0, ENCODER_DRIVE_TOLERANCE,
                              90.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 5:
            // Drive forward ~1 ft or until touch active.
            PIDCtrlSetPowerLimit(g_PIDCtrl[0], -30, 30);
            PIDCtrlSetPowerLimit(g_PIDCtrl[1], -30, 30);
            PIDDriveSetTarget(g_EncoderDrive,
                              16.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
#if 0
            SMAddWaitEvent(sm, EVTTYPE_TOUCH, -1, -1);
#endif
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 6:
            // Stop previous drive and drive backward 3 inches.
#if 0
            PIDDriveReset(g_EncoderDrive);
#endif
            PIDDriveSetTarget(g_EncoderDrive,
                              -3.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

#if 0
        case SMSTATE_STARTED + 12:
            // Lower scooper.
            ServoSetAngle(g_Scooper, SCOOPER_LOWERED);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoScooper, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;
#endif

        case SMSTATE_STARTED + 7:
            // Drive forward ~3 ft.
            PIDCtrlSetPowerLimit(g_PIDCtrl[0], -50, 50);
            PIDCtrlSetPowerLimit(g_PIDCtrl[1], -50, 50);
            PIDDriveSetTarget(g_EncoderDrive,
                              38.5, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;
#endif

        //
        // Balance on the bridge.
        //
#ifdef _USE_NESTED_SM
        case SMSTATE_STARTED + 5:
#else
        case SMSTATE_STARTED + 8:
#endif
            TimerSet(g_Timer, 1000);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

#ifdef _USE_NESTED_SM
        case SMSTATE_STARTED + 6:
#else
        case SMSTATE_STARTED + 9:
#endif
            AccelGetY(g_Accel, accelValue);
            nxtDisplayTextLine(2, "AccelY=%f", accelValue);
            if (abs(accelValue) < 0.001)
            {
                //
                // We are balanced.
                //
                sm.currState--;
            }
            else if (accelValue > 0.0)
            {
                //
                // We need to go forward 1 inch.
                //
                PIDDriveSetTarget(g_EncoderDrive,
                                  1.0, ENCODER_DRIVE_TOLERANCE,
                                  0.0, ENCODER_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState - 1, 0, SMF_CLEAR_EVENTS);
            }
            else
            {
                //
                // We need to go backward 1 inch.
                //
                PIDDriveSetTarget(g_EncoderDrive,
                                  -1.0, ENCODER_DRIVE_TOLERANCE,
                                  0.0, ENCODER_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState - 1, 0, SMF_CLEAR_EVENTS);
            }
            break;

        default:
            SMStop(sm);
            break;
    }

    TExit();
    return;
}   //AutoRight1

/**
 *  This function implements the autonomous mode with a start position on the
 *  right corner.
 *
 *  @param sm Points to the SM structure.
 */
void
AutoRight2(
  __inout SM &sm
  )
{
    TFuncName("AutoRight2");
    TLevel(TASK);
    TEnter();

    switch (sm.currState)
    {
        case SMSTATE_STARTED:
            // Drive forward 2- ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              20.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

#ifdef _USE_NESTED_SM
        //
        // Dump batons into low goal.
        //
        case SMSTATE_STARTED + 1:
            SMStart(g_LowGoalSM);
            sm.currState++;
            break;

        case SMSTATE_STARTED + 2:
            if (IsSMEnabled(g_LowGoalSM))
            {
                LowGoal(g_LowGoalSM);
            }
            else
            {
                sm.currState++;
            }
            break;

        //
        // Go dispense batons.
        //
        case SMSTATE_STARTED + 3:
            SMStart(g_DispenseSM);
            sm.currState++;
            break;

        case SMSTATE_STARTED + 4:
            if (IsSMEnabled(g_DispenseSM))
            {
                Dispense(g_DispenseSM);
            }
            else
            {
                sm.currState++;
            }
            break;

#else
        //
        // Dump batons into low goal.
        //
#if 0
        case SMSTATE_STARTED + 1:
            // Lower scooper for 2 seconds.
            ServoSetAngle(g_Scooper, SCOOPER_LOWERED);
            TimerSet(g_Timer, 2000);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 2:
            // Raise scooper.
            ServoSetAngle(g_Scooper, SCOOPER_RAISED);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoScooper, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 3:
            // Lower arm.
            SetArmAngle(ARM_LOWERED);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmLeft, -1);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmRight, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 4:
            // Unload the batons for 2 seconds.
            motor[motorDispense] = DISPENSE_PUSH;
            TimerSet(g_Timer, 2000);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 5:
            // Stop dispense motor and raise arm.
            motor[motorDispense] = 0;
            SetArmAngle(ARM_RAISED);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmLeft, -1);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmRight, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;
#endif

        case SMSTATE_STARTED + 1:
            // Drive backward 4 inches.
            PIDDriveSetTarget(g_EncoderDrive,
                              -4.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 2:
            // Turn left 90 degrees.
            PIDDriveSetTarget(g_EncoderDrive,
                              0.0, ENCODER_DRIVE_TOLERANCE,
                              -90.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        //
        // Go dispense batons.
        //
        case SMSTATE_STARTED + 3:
            // Drive forward ~4 ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              49.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 4:
            // Turn right 90 degrees.
            PIDDriveSetTarget(g_EncoderDrive,
                              0.0, ENCODER_DRIVE_TOLERANCE,
                              90.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 5:
            // Drive forward 6 ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              72.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

#ifdef _IRDRIVE_ENABLED
        case SMSTATE_STARTED + 6:
            // Engage IR drive for ~4 ft or until touched.
            PIDCtrlSetPowerLimit(g_PIDCtrl[0], -30, 30);
            PIDCtrlSetPowerLimit(g_PIDCtrl[1], -30, 30);
            PIDDriveSetTarget(g_IRDrive,
                              48.0, ENCODER_DRIVE_TOLERANCE,
                              8.0, IRSEEK_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMAddWaitEvent(sm, EVTTYPE_TOUCH, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 7:
            // Drive backward 6 inches.
            PIDDriveReset(g_IRDrive);
            PIDCtrlSetPowerLimit(g_PIDCtrl[0], -50, 50);
            PIDCtrlSetPowerLimit(g_PIDCtrl[1], -50, 50);
            PIDDriveSetTarget(g_EncoderDrive,
                              -7.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 8:
            // Lower arm to mid level.
            SetArmAngle(ARM_MID_DISPENSE);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmLeft, -1);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmRight, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 9:
            // Drive forward 3.5 inches.
            PIDDriveReset(g_IRDrive);
            PIDDriveSetTarget(g_EncoderDrive,
                              3.5, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 10:
            // Dispense the batons.
            motor[motorDispense] = DISPENSE_PULL;
            TimerSet(g_Timer, 5000);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 16:
            // Backoff 6 inches.
            motor[motorDispense] = 0;
            PIDDriveSetTarget(g_EncoderDrive,
                              -6.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;
#endif
#endif

        default:
            SMStop(sm);
            break;
    }

    TExit();
    return;
}   //AutoRight2

/**
 *  This function implements the autonomous mode with a start position on the
 *  right corner.
 *
 *  @param sm Points to the SM structure.
 */
void
AutoRight3(
  __inout SM &sm
  )
{
    TFuncName("AutoRight3");
    TLevel(TASK);
    TEnter();

    switch (sm.currState)
    {
        case SMSTATE_STARTED:
            // Drive forward 2- ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              20.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

#ifdef _USE_NESTED_SM
        //
        // Dump batons into low goal.
        //
        case SMSTATE_STARTED + 1:
            SMStart(g_LowGoalSM);
            sm.currState++;
            break;

        case SMSTATE_STARTED + 2:
            if (IsSMEnabled(g_LowGoalSM))
            {
                LowGoal(g_LowGoalSM);
            }
            else
            {
                sm.currState++;
            }
            break;

        //
        // Climb onto the bridge.
        //
        case SMSTATE_STARTED + 3:
            SMStart(g_ClimbBridgeSM);
            sm.currState++;
            break;

        case SMSTATE_STARTED + 4:
            if (IsSMEnabled(g_ClimbBridgeSM))
            {
                ClimbBridge(g_ClimbBridgeSM);
            }
            else
            {
                sm.currState++;
            }
            break;

#else
        //
        // Dump batons into low goal.
        //
#if 0
        case SMSTATE_STARTED + 1:
            // Lower scooper for 2 seconds.
            ServoSetAngle(g_Scooper, SCOOPER_LOWERED);
            TimerSet(g_Timer, 2000);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 2:
            // Raise scooper.
            ServoSetAngle(g_Scooper, SCOOPER_RAISED);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoScooper, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 3:
            // Lower arm.
            SetArmAngle(ARM_LOWERED);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmLeft, -1);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmRight, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 4:
            // Unload the batons for 2 seconds.
            motor[motorDispense] = DISPENSE_PUSH;
            TimerSet(g_Timer, 2000);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 5:
            // Stop dispense motor and raise arm.
            motor[motorDispense] = 0;
            SetArmAngle(ARM_RAISED);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmLeft, -1);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoArmRight, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;
#endif

        case SMSTATE_STARTED + 1:
            // Drive backward 4 inches.
            PIDDriveSetTarget(g_EncoderDrive,
                              -4.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 2:
            // Turn left 90 degrees.
            PIDDriveSetTarget(g_EncoderDrive,
                              0.0, ENCODER_DRIVE_TOLERANCE,
                              -90.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        //
        // Climb onto the bridge.
        //
        case SMSTATE_STARTED + 3:
            // Drive forward ~1 ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              15.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 4:
            // Turn right 90-degree.
            PIDDriveSetTarget(g_EncoderDrive,
                              0.0, ENCODER_DRIVE_TOLERANCE,
                              90.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 5:
            // Drive forward ~1 ft or until touch active.
            PIDCtrlSetPowerLimit(g_PIDCtrl[0], -30, 30);
            PIDCtrlSetPowerLimit(g_PIDCtrl[1], -30, 30);
            PIDDriveSetTarget(g_EncoderDrive,
                              16.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
#if 0
            SMAddWaitEvent(sm, EVTTYPE_TOUCH, -1, -1);
#endif
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 6:
            // Stop previous drive and drive backward 3 inches.
#if 0
            PIDDriveReset(g_EncoderDrive);
#endif
            PIDDriveSetTarget(g_EncoderDrive,
                              -3.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

#if 0
        case SMSTATE_STARTED + 12:
            // Lower scooper.
            ServoSetAngle(g_Scooper, SCOOPER_LOWERED);
            SMAddWaitEvent(sm, EVTTYPE_SERVO, servoScooper, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;
#endif

        case SMSTATE_STARTED + 7:
            // Drive forward ~3 ft.
            PIDCtrlSetPowerLimit(g_PIDCtrl[0], -50, 50);
            PIDCtrlSetPowerLimit(g_PIDCtrl[1], -50, 50);
            PIDDriveSetTarget(g_EncoderDrive,
                              38.5, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;
#endif

#ifdef _USE_NESTED_SM
        case SMSTATE_STARTED + 5:
#else
        case SMSTATE_STARTED + 8:
#endif
            // Drive forward 3 ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              36.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        default:
            SMStop(sm);
            break;
    }

    TExit();
    return;
}   //AutoRight3

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

    BattInit(g_Batt, 1, true);
    //
    // Init choice menu.
    //
    MenuInit(g_Menu, "Start Position:", 0, STARTPOS_RIGHT1);
    MenuAddChoice(g_Menu, "Left  1");
    MenuAddChoice(g_Menu, "Left  2");
    MenuAddChoice(g_Menu, "Right 1");
    MenuAddChoice(g_Menu, "Right 2");
    MenuAddChoice(g_Menu, "Right 3");
    g_StartPos = MenuGetChoice(g_Menu);

    eraseDisplay();
    nxtDisplayCenteredBigTextLine(5, (g_StartPos == STARTPOS_LEFT1)?
                                        "LEFT 1":
                                     (g_StartPos == STARTPOS_LEFT2)?
                                        "LEFT 2":
                                     (g_StartPos == STARTPOS_RIGHT1)?
                                        "RIGHT 1":
                                     (g_StartPos == STARTPOS_RIGHT2)?
                                        "RIGHT 2": "RIGHT 3");

    //
    // Initialize the input subsystems.
    //
    AccelInit(g_Accel, accel, 0);
    GyroInit(g_Gyro, gyro, 0);
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
    PIDCtrlInit(g_PIDCtrl[4], 4,
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
                 2, //BalanceDrive_PIDCtrl,
                 3, //BalanceTurn_PIDCtrl,
                 0);

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
    DriveTask(g_Drive[0]);
    BattTask(g_Batt);

    TExit();
    return;
}   //OutputTasks
