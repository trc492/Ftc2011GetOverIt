#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Right1.h" />
///
/// <summary>
///     This module contains the autonomous routine for starting at the
///     right corner.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_MAIN

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
        case SMSTATE_STARTED + 1:
            // Lower scooper for 2 seconds.
            ServoSetAngle(g_Scooper, SCOOPER_LOWERED);
            TimerSet(g_Timer, 2000);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 2:
            // Raise scooper and wait half second.
            ServoSetAngle(g_Scooper, SCOOPER_RAISED);
            TimerSet(g_Timer, 500);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 3:
            // Lower arm and wait 100 msec.
            SetArmAngle(ARM_LOWERED);
            TimerSet(g_Timer, 100);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
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
            // Stop dispense motor, raise arm and wait 100 msec.
            motor[motorDispense] = 0;
            SetArmAngle(ARM_RAISED);
            TimerSet(g_Timer, 100);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 6:
            // Drive backward 6 inches.
            PIDDriveSetTarget(g_EncoderDrive,
                              -6.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 7:
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
        case SMSTATE_STARTED + 8:
            // Drive forward ~1 ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              14.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 9:
            // Turn right 90-degree.
            PIDDriveSetTarget(g_EncoderDrive,
                              0.0, ENCODER_DRIVE_TOLERANCE,
                              90.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 10:
            // Drive forward ~1 ft or until touch active.
            PIDCtrlSetPowerLimit(g_PIDCtrl[0], -30, 30);
            PIDCtrlSetPowerLimit(g_PIDCtrl[1], -30, 30);
            PIDDriveSetTarget(g_EncoderDrive,
                              18.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMAddWaitEvent(sm, EVTTYPE_TOUCH, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 11:
            // Stop previous drive and drive backward 3 inches.
            PIDDriveReset(g_EncoderDrive);
            PIDDriveSetTarget(g_EncoderDrive,
                              -3.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 12:
            // Lower scooper and wait half second.
            ServoSetAngle(g_Scooper, SCOOPER_LOWERED);
            TimerSet(g_Timer, 500);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 13:
            // Drive forward ~3 ft.
            PIDCtrlSetPowerLimit(g_PIDCtrl[0], -50, 50);
            PIDCtrlSetPowerLimit(g_PIDCtrl[1], -50, 50);
            PIDDriveSetTarget(g_EncoderDrive,
                              37.5, ENCODER_DRIVE_TOLERANCE,
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
        case SMSTATE_STARTED + 14:
#endif
            PIDDriveReset(g_EncoderDrive);
            TimerSet(g_Timer, 2000);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

#ifdef _USE_NESTED_SM
        case SMSTATE_STARTED + 6:
#else
        case SMSTATE_STARTED + 15:
#endif
            AccelGetX(g_Accel, accelValue);
            nxtDisplayTextLine(2, "AccelX=%f", accelValue);
            if (abs(accelValue) < 0.07)
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
                DriveArcade(g_Drive[0], 30, 0);
                TimerSet(g_Timer, 100);
                SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
                SMWaitEvents(sm, sm.currState - 1, 0, SMF_CLEAR_EVENTS);
            }
            else
            {
                //
                // We need to go backward 1 inch.
                //
                DriveArcade(g_Drive[0], -30, 0);
                TimerSet(g_Timer, 100);
                SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
                SMWaitEvents(sm, sm.currState - 1, 0, SMF_CLEAR_EVENTS);
            }
            break;

#if 0
        case SMSTATE_STARTED + 16:
            PIDCtrlSetPowerLimit(g_PIDCtrl[0], -5, 10);
            PIDCtrlSetPowerLimit(g_PIDCtrl[1], -5, 10);
            PIDDriveSetTarget(g_BalanceDrive,
                              0.0, BALANCE_DRIVE_TOLERANCE,
                              0.0, BALANCE_TURN_TOLERANCE,
                              false);
            sm.currState++;
            break;

        case SMSTATE_STARTED + 17:
            //
            // Loop in this state until autonomous period ends.
            //
            break;
#endif

        default:
            SMStop(sm);
            break;
    }

    TExit();
    return;
}   //AutoRight1
