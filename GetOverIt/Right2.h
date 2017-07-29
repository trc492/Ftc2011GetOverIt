#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Right2.h" />
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
        // Go dispense batons.
        //
        case SMSTATE_STARTED + 8:
            // Drive forward ~4 ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              49.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 9:
            // Turn right 90 degrees.
            PIDDriveSetTarget(g_EncoderDrive,
                              0.0, ENCODER_DRIVE_TOLERANCE,
                              90.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 10:
            // Drive forward 6 ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              74.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

#ifdef _IRDRIVE_ENABLED
        case SMSTATE_STARTED + 11:
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

        case SMSTATE_STARTED + 12:
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

        case SMSTATE_STARTED + 13:
            // Lower arm to mid level and wait 300 msec.
            SetArmAngle(ARM_MID_DISPENSE);
            TimerSet(g_Timer, 300);
            SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 14:
            // Drive forward 3.5 inches.
            PIDDriveReset(g_IRDrive);
            PIDDriveSetTarget(g_EncoderDrive,
                              3.5, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
            SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
            break;

        case SMSTATE_STARTED + 15:
            // Dispense the batons for 5 seconds.
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
