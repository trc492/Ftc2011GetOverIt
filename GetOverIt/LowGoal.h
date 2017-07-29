#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="LowGoal.h" />
///
/// <summary>
///     This module contains the autonomous routine for dumping batons to the
///     low goal.
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
            case SMSTATE_STARTED:
                // Lower scooper for 2 seconds.
                ServoSetAngle(g_Scooper, SCOOPER_LOWERED);
                TimerSet(g_Timer, 2000);
                SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 1:
                // Raise scooper and wait half second.
                ServoSetAngle(g_Scooper, SCOOPER_RAISED);
                TimerSet(g_Timer, 500);
                SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 2:
                // Lower arm and wait 100 msec.
                SetArmAngle(ARM_LOWERED);
                TimerSet(g_Timer, 100);
                SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
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
                // Stop dispense motor, raise arm and wait 100 msec.
                motor[motorDispense] = 0;
                SetArmAngle(ARM_RAISED);
                TimerSet(g_Timer, 100);
                SMAddWaitEvent(sm, EVTTYPE_TIMER, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 5:
                // Drive backward 4 inches.
                PIDDriveSetTarget(g_EncoderDrive,
                                  -4.0, ENCODER_DRIVE_TOLERANCE,
                                  0.0, ENCODER_TURN_TOLERANCE,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1, 0, SMF_CLEAR_EVENTS);
                break;

            case SMSTATE_STARTED + 6:
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
