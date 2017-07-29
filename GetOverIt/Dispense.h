#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Dispense.h" />
///
/// <summary>
///     This module contains the autonomous routine for dispensing batons
///     from the mid dispenser.
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
                // Drive backward 7 inches.
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
                // Lower arm to mid level and wait 300 msec.
                SetArmAngle(ARM_MID_DISPENSE);
                TimerSet(g_Timer, 300);
                SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
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
                // Dispense the batons for 5 seconds.
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
