#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="Left1.h" />
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
 *  left corner. It crosses the cliff to the other side and sits there.
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
            // Drive forward ~6.5 ft.
            PIDDriveSetTarget(g_EncoderDrive,
                              98.0, ENCODER_DRIVE_TOLERANCE,
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
}   //AutoLeft1
