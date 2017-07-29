#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Left2.h" />
///
/// <summary>
///     This module contains the autonomous routine for starting at the
///     left corner.
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
 *  left corner. It balances itself on the bridge.
 *
 *  @param sm Points to the SM structure.
 */
void
AutoLeft2(
  __inout SM &sm
  )
{
    float accelValue;

    TFuncName("AutoLeft2");
    TLevel(TASK);
    TEnter();

    switch (sm.currState)
    {
        case SMSTATE_STARTED:
            // Drive forward ~1 ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              13.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 1:
            // Turn left 90-degree.
            PIDDriveSetTarget(g_EncoderDrive,
                              0.0, ENCODER_DRIVE_TOLERANCE,
                              -90.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 2:
            // Drive forward 2 ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              24.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        //
        // Climb onto the bridge.
        //
        case SMSTATE_STARTED + 3:
            // Drive forward ~1 ft or until touch active.
            PIDCtrlSetPowerLimit(g_PIDCtrl[0], -30, 30);
            PIDCtrlSetPowerLimit(g_PIDCtrl[1], -30, 30);
            PIDDriveSetTarget(g_EncoderDrive,
                              16.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMAddWaitEvent(sm, EVTTYPE_TOUCH, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 4:
            // Stop previous drive and drive backward 3 inches.
            PIDDriveReset(g_EncoderDrive);
            PIDDriveSetTarget(g_EncoderDrive,
                              -3.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 5:
            // Lower scooper and wait half second.
            ServoSetAngle(g_Scooper, SCOOPER_LOWERED);
            TimerSet(g_Timer, 500);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, 0, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 6:
            // Drive forward ~3 ft.
            PIDCtrlSetPowerLimit(g_PIDCtrl[0], -50, 50);
            PIDCtrlSetPowerLimit(g_PIDCtrl[1], -50, 50);
            PIDDriveSetTarget(g_EncoderDrive,
                              36.5, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        //
        // Balance on the bridge.
        //
        case SMSTATE_STARTED + 7:
            PIDDriveReset(g_EncoderDrive);
            TimerSet(g_Timer, 2000);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 8:
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

        default:
            SMStop(sm);
            break;
    }

    TExit();
    return;
}   //AutoLeft2
