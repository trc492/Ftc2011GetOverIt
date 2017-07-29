#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="lnfollow.h" />
///
/// <summary>
///     This module contains the library functions to follow the line.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _LNFOLLOW_H
#define _LNFOLLOW_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_LNFOLLOW

//
// Constants.
//
#ifndef MAX_LIGHT_SENSORS
    #define MAX_LIGHT_SENSORS   3
#endif

#define LNF_STARTED             0x0100
#define LNF_CALIBRATING         0x0200
#define LnFollowStarted(l)      (l.lnfollowFlags & LNF_STARTED)
#define LnFollowCalibrating(l)  (l.lnfollowFlags & LNF_CALIBRATING)

//
// Type definitions.
//
typedef struct
{
//    DRIVE   &drive;
//    PIDCTRL &pidCtrl;
    int      drive;
    int      pidCtrl;
    int      numLightSensors;
    int      driveKp;
    SENSOR   LightSensors[MAX_LIGHT_SENSORS];
    int      valueMap[1 << MAX_LIGHT_SENSORS];
    int      lnfollowFlags;
    int      maxSensorValue;
    int      mapIndex;
    int      sensorValue;
} LNFOLLOW;

/**
 *  This function initializes the line follower object.
 *
 *  @param lnfollow Points to the LNFOLLOW structure to be initialized.
 *  @param drive Points to the DRIVE structure.
 *  @param pidCtrl Points to the PIDCTRL structure.
 *  @param numLightSensors Specifies the number of light sensors used.
 *  @param driveKp Specifies the drive power scale factor.
 */
void
LnFollowInit(
    __out LNFOLLOW &lnfollow,
//    __in  DRIVE &drive,
//    __in  PIDCTRL &pidCtrl,
    __in  int drive,
    __in  int pidCtrl,
    __in  int numLightSensors,
    __in  int driveKp
    )
{
    int i;

    TFuncName("LnFollowInit");
    TLevel(INIT);
    TEnter();

    lnfollow.drive = drive;
    lnfollow.pidCtrl = pidCtrl;
    lnfollow.numLightSensors = (numLightSensors <= MAX_LIGHT_SENSORS)?
                               numLightSensors: MAX_LIGHT_SENSORS;
    lnfollow.driveKp = driveKp;
    lnfollow.lnfollowFlags = 0;
    lnfollow.maxSensorValue = lnfollow.numLightSensors*2 - 1;
    lnfollow.mapIndex = 0;
    lnfollow.sensorValue = 0;

    //
    // Initialize the "no light" value.
    //
    lnfollow.valueMap[0] = 0;

    //
    // Initialize the rest of the array with an invalid value.
    //
    for (i = 1; i < (1 << lnfollow.numLightSensors); i++)
    {
        lnfollow.valueMap[i] = -1;
    }

    for (i = 0; i < lnfollow.numLightSensors; i++)
    {
        //
        // Initialize the "right on" light values.
        //
        lnfollow.valueMap[1 << i] = i*2 + 1;
        //
        // Initialize the "in between" light values.
        //
        if (i < lnfollow.numLightSensors - 1)
        {
            lnfollow.valueMap[(i << i) + (i << (i + 1))] = (i + 1)*2;
        }
    }

    TExit();
    return;
}   //LnFollowInit

/**
 *  This function calibrates the light sensors of the line follower.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 *  @param fStart Specifies TRUE to start calibration, FALSE to stop.
 */
void
LnFollowCal(
    __inout LNFOLLOW &lnfollow,
    __in    bool fStart
    )
{
    TFuncName("LnFollowCal");
    TLevel(API);
    TEnterMsg(("fStart=%d", (byte)fStart));

    if (fStart)
    {
        lnfollow.lnfollowFlags |= LNF_CALIBRATING;
    }
    else
    {
        lnfollow.lnfollowFlags &= ~LNF_CALIBRATING;
    }

    for (int i = 0; i < lnfollow.numLightSensors; i++)
    {
        SensorCal(lnfollow.LightSensors[i], fStart);
    }

    TExit();
    return;
}   //LnFollowCal

/**
 *  This function starts or stops the line follower.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 *  @param fStart Specifies TRUE to start the line follower, FALSE to stop.
 */
void
LnFollowStart(
    __inout LNFOLLOW &lnfollow,
    __in    bool fStart
    )
{
    TFuncName("LnFollowStart");
    TLevel(API);
    TEnterMsg(("fStart=%d", (byte)fStart));

    if (fStart)
    {
        if (!LnFollowStarted(lnfollow))
        {
            lnfollow.lnfollowFlags |= LNF_STARTED;
            PIDCtrlSetTarget(g_PIDCtrl[lnfollow.pidCtrl],
                             (lnfollow.maxSensorValue + 1)/2.0, -1.0);
        }
    }
    else if (LnFollowStarted(lnfollow))
    {
        lnfollow.lnfollowFlags &= ~LNF_STARTED;
        DriveReset(g_Drive[lnfollow.drive]);
        PIDCtrlReset(g_PIDCtrl[lnfollow.pidCtrl]);
    }

    TExit();
    return;
}   //LnFollowStart

/**
 *  This function processes the sensor data for line following.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 */
void
LnFollowTask(
    __inout LNFOLLOW &lnfollow
    )
{
    TFuncName("LnFollowTask");
    TLevel(TASK);
    TEnter();

    if (LnFollowStarted(lnfollow))
    {
        lnfollow.mapIndex = 0;
        for (int i = 0; i < lnfollow.numLightSensors; i++)
        {
            lnfollow.mapIndex <<= 1;
            if (lnfollow.LightSensors[i].sensorZone == SENSORZONE_HI)
            {
                lnfollow.mapIndex++;
            }
        }
        lnfollow.sensorValue = lnfollow.valueMap[lnfollow.mapIndex];

        //
        // Performing PID control on turn.
        //
        nxtDisplayTextLine(2, "idx=%d,value=%d",
                           lnfollow.mapIndex, lnfollow.sensorValue);
//        lnfollow.turnPower = PIDCtrlOutput(lnfollow.pidCtrl,
//                                           lnfollow.weightedSensorValue);
        int turnPower = PIDCtrlOutput(g_PIDCtrl[lnfollow.pidCtrl]);
        //
        // Calculate the drive power: Drive power should be inversely
        // proportaional to the turn power. So the faster the robot turns,
        // the slower we will drive the robot.
        //
        int drivePower = (MOTOR_MAX_VALUE - abs(turnPower))*lnfollow.driveKp;
//        DriveArcade(lnfollow.drive,
        DriveArcade(g_Drive[lnfollow.drive], drivePower, turnPower);
    }

    TExit();
    return;
}   //LnFollowTask

#endif  //ifndef _LNFOLLOW_H
