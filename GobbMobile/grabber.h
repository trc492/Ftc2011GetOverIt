#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="grabber.h" />
///
/// <summary>
///     This module contains the functions to handle the grabber.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_GRABBER                     TGenModId(1)
#define MOD_ID                          MOD_GRABBER

//
// Constants.
//
#define GRABBERMODE_STOPPED             0
#define GRABBERMODE_OPENING             1
#define GRABBERMODE_CLOSING             2

#define GRABBER_MOVE_THRESHOLD          2

//
// Type definitions.
//
typedef struct
{
    int  grabberMotor;
    int  calPower;
    int  maxPower;
    long minPos;
    long maxPos;
    long timeStep;
    int  grabberMode;
    bool fCalibrating;
    int  grabberPower;
    long grabberPos;
    long prevTime;
} GRABBER;

/**
 *  This function resets the grabber system.
 *
 *  @param grabber Points to the GRABBER structure to be reset.
 */
void
GrabberReset(
    __out GRABBER &grabber
    )
{
    TFuncName("GrabberReset");
    TLevel(API);
    TEnter();

    grabber.grabberMode = GRABBERMODE_STOPPED;
    grabber.fCalibrating = false;
    grabber.grabberPower = 0;
    grabber.grabberPos = 0;
    grabber.prevTime = time1[T1];
    motor[grabber.grabberMotor] = 0;
    nMotorEncoder[grabber.grabberMotor] = 0;

    TExit();
    return;
}   //GrabberReset

/**
 *  This function starts the grabber calibration.
 *
 *  @param grabber Points to the GRABBER structure.
*/
void
GrabberStartCal(
    __inout GRABBER &grabber
    )
{
    TFuncName("GrabberStartCal");
    TLevel(API);
    TEnter();

    if (!grabber.fCalibrating)
    {
        grabber.fCalibrating = true;
        grabber.grabberPos = nMotorEncoder[grabber.grabberMotor];
        grabber.grabberPower = -grabber.calPower;
        grabber.grabberMode = GRABBERMODE_CLOSING;
    }

    TExit();
    return;
}   //GrabberStartCal

/**
 *  This function initializes the grabber system.
 *
 *  @param grabber Points to the GRABBER structure to be initialized.
 *  @param grabberMotor Specifies the grabber motor.
 *  @param calPower Specifies the power used for calibration.
 *  @param maxPower Specifies the maximum power for the grabber.
 *  @param minPos Specifies the minimum position of the grabber when fully closed.
 *  @param maxPos Specifies the maximum position of the grabber when fully opened.
 *  @param timeStep Specifies the time step for the grabber task loop.
 */
void
GrabberInit(
    __out GRABBER &grabber,
    __in  int grabberMotor,
    __in  int calPower,
    __in  int maxPower,
    __in  long minPos,
    __in  long maxPos,
    __in  long timeStep
    )
{
    TFuncName("GrabberInit");
    TLevel(INIT);
    TEnter();

    grabber.grabberMotor = grabberMotor;
    grabber.calPower = calPower;
    grabber.maxPower = maxPower;
    grabber.minPos = minPos;
    grabber.maxPos = maxPos;
    grabber.timeStep = timeStep;
    GrabberReset(grabber);
//    GrabberStartCal(grabber);

    TExit();
    return;
}   //GrabberInit

/**
 *  This function sets the grabber power to start the action.
 *
 *  @param grabber Points to the GRABBER structure.
 *  @param power Specifies the grabber motor power within the specified
 *         range. The power range is used to normalized a power value from
 *         the joystick range to the specified maxPower range.
 */
void
GrabberSetPower(
    __out GRABBER &grabber,
    __in  int power
    )
{
    TFuncName("GrabberSetPower");
    TLevel(API);
    TEnterMsg(("Power=%d", power));

    if (!grabber.fCalibrating)
    {
        //
        // Don't do anything if we are still calibrating.
        //
        grabber.grabberPower = NORMALIZE(power,
                                         -100, 100,
                                         -grabber.maxPower,
                                         grabber.maxPower);
        grabber.grabberMode = (power == 0)? GRABBERMODE_STOPPED:
                              (power > 0)? GRABBERMODE_OPENING:
                                           GRABBERMODE_CLOSING;
    }

    TExit();
    return;
}   //GrabberSetPower

/**
 *  This function performs the grabber task according to the grabber mode.
 *
 *  @param grabber Points to the GRABBER structure.
 */
void
GrabberTask(
    __inout GRABBER &grabber
    )
{
    TFuncName("GrabberTask");
    TLevel(TASK);
    TEnter();

    static bool fMoving = false;
    long currTime = time1[T1];

    if (currTime >= grabber.prevTime + grabber.timeStep)
    {
        int power = grabber.grabberPower;
        long currPos = nMotorEncoder[grabber.grabberMotor];
        bool fStuck = fMoving &&
                      (abs(currPos - grabber.grabberPos) <
                       GRABBER_MOVE_THRESHOLD);

        TVerbose(("Mode=%d", grabber.grabberMode));
        switch (grabber.grabberMode)
        {
            case GRABBERMODE_OPENING:
                if (fStuck ||
                    (!grabber.fCalibrating && (currPos >= grabber.maxPos)))
                {
                    fMoving = false;
                    grabber.grabberPower = 0;
                    motor[grabber.grabberPower] = 0;
                    grabber.grabberPower = GRABBERMODE_STOPPED;
                }
                else
                {
                    motor[grabber.grabberMotor] = power;
                    fMoving = grabber.grabberPower != 0;
                }
                break;

            case GRABBERMODE_CLOSING:
                if (fStuck ||
                    (!grabber.fCalibrating && (currPos <= grabber.minPos)))
                {
                    fMoving = false;
                    grabber.grabberPower = 0;
                    motor[grabber.grabberMotor] = 0;
                    grabber.grabberMode = GRABBERMODE_STOPPED;
                    if (grabber.fCalibrating)
                    {
                        nMotorEncoder[grabber.grabberMotor] = 0;
                        grabber.fCalibrating = false;
                    }
                }
                else
                {
                    motor[grabber.grabberMotor] = power;
                    fMoving = grabber.grabberPower != 0;
                }
                break;
        }

        grabber.grabberPos = currPos;
        grabber.prevTime = currTime;
    }

    TExit();
    return;
}   //GrabberTask
