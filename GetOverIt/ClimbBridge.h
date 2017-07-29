#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="ClimbBridge.h" />
///
/// <summary>
///     This module contains the autonomous routine for climbing onto the
///     bridge.
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

            case SMSTATE_STARTED + 1:
                // Stop previous drive and drive backward 3 inches.
                PIDDriveReset(g_EncoderDrive);
                PIDDriveSetTarget(g_EncoderDrive,
                                  -3.0, ENCODER_DRIVE_TOLERANCE,
                                  0.0, ENCODER_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 2:
                // Lower scooper and wait half second.
                ServoSetAngle(g_Scooper, SCOOPER_LOWERED);
                TimerSet(g_Timer, 500);
                SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 3:
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
