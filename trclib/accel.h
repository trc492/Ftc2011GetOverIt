#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="accel.h" />
///
/// <summary>
///     This module contains the library functions for the accelerometer
///     sensor.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _ACCEL_H
#define _ACCEL_H

#include "..\HTDriversV1.6\drivers\HTAC-driver.h"

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_ACCEL

//
// Constants.
//
#define ACCELF_USER_MASK        0x00ff
#ifdef HTSMUX_STATUS
  #define ACCELF_HTSMUX         0x0080
#endif
#define ACCEL_COUNT_PER_G       200
#define ACCEL_NUM_CAL_SAMPLES   50
#define ACCEL_CAL_INTERVAL      10

//
// Type definitions.
//
typedef struct
{
    int sensorID;
    int accelFlags;
    int xZeroOffset;
    int yZeroOffset;
    int zZeroOffset;
    int xDeadBand;
    int yDeadBand;
    int zDeadBand;
} ACCEL;

/**
 *  This function calibrates the accelerometer for zero offsets and deadband
 *  on all axes.
 *
 *  @param accel Points to the ACCEL structure to be initialized.
 *  @param numSamples Specifies the number of calibration samples.
 *  @param calInterval Specifies the calibration interval in msec.
 */
void
AccelCal(
    __out ACCEL &accel,
    __in  int numSamples,
    __in  int calInterval
    )
{
    int i;
    int xRaw, yRaw, zRaw;
    int xMin, yMin, zMin;
    int xMax, yMax, zMax;
    bool fSMux;

    TFuncName("AccelCal");
    TLevel(API);

    accel.xZeroOffset = 0;
    accel.yZeroOffset = 0;
    accel.zZeroOffset = 0;
    accel.xDeadBand = 0;
    accel.yDeadBand = 0;
    accel.zDeadBand = 0;
    xMin = yMin = zMin = 1023;
    xMax = yMax = zMax = 0;
    fSMux = (accel.accelFlags & ACCELF_HTSMUX) != 0;
    for (i = 0; i < numSamples; i++)
    {
#ifdef HTSMUX_STATUS
        if (fSMux &&
            HTACreadAllAxes((tMUXSensor)accel.sensorID, xRaw, yRaw, zRaw) ||
            !fSMux &&
            HTACreadAllAxes((tSensors)accel.sensorID, xRaw, yRaw, zRaw))
#else
        if (HTACreadAllAxes((tSensors)accel.sensorID, xRaw, yRaw, zRaw))
#endif
        {
            accel.xZeroOffset += xRaw;
            accel.yZeroOffset += yRaw;
            accel.zZeroOffset += zRaw;

            if (xRaw < xMin)
            {
                xMin = xRaw;
            }
            else if (xRaw > xMax)
            {
                xMax = xRaw;
            }

            if (yRaw < yMin)
            {
                yMin = yRaw;
            }
            else if (yRaw > yMax)
            {
                yMax = yRaw;
            }

            if (zRaw < zMin)
            {
                zMin = zRaw;
            }
            else if (zRaw > zMax)
            {
                zMax = zRaw;
            }
        }
        wait1Msec(calInterval);
    }

    accel.xZeroOffset /= numSamples;
    accel.yZeroOffset /= numSamples;
    accel.zZeroOffset /= numSamples;

    accel.xDeadBand = xMax - xMin;
    accel.yDeadBand = yMax - yMin;
    accel.zDeadBand = zMax - zMin;

    TExit();
    return;
}   //AccelCal

/**
 *  This function initializes the accelerometer sensor.
 *
 *  @param accel Points to the ACCEL structure to be initialized.
 *  @param sensorID Specifies the ID of the accelerometer sensor.
 *  @param accelFlags Specifies the accelerometer flags.
 */
void
AccelInit(
    __out ACCEL &accel,
    __in  int sensorID,
    __in  int accelFlags
    )
{
    TFuncName("AccelInit");
    TLevel(INIT);
    TEnter();

    accel.sensorID = sensorID;
    accel.accelFlags = accelFlags & ACCELF_USER_MASK;
    AccelCal(accel, ACCEL_NUM_CAL_SAMPLES, ACCEL_CAL_INTERVAL);

    TExit();
    return;
}   //AccelInit

/**
 *  This function gets the current value of the accelerometer X-axis in unit G.
 *
 *  @param accel Points to the ACCEL structure.
 *  @param value Returns the value in G.
 *
 *  @return Returns true if no error, false otherwise.
 */
bool
AccelGetX(
    __in  ACCEL &accel,
    __out float &value
    )
{
    bool rc;
    int raw;
    int dummy;

    TFuncName("AccelGetX");
    TLevel(API);
    TEnter();

#ifdef HTSMUX_STATUS
    rc = (accel.accelFlags & ACCELF_HTSMUX)?
            HTACreadAllAxes((tMUXSensor)accel.sensorID, raw, dummy, dummy):
            HTACreadAllAxes((tSensors)accel.sensorID, raw, dummy, dummy);
#else
    rc = HTACreadAllAxes((tSensors)accel.sensorID, raw, dummy, dummy));
#endif
    if (rc == true)
    {
        value = (float)(DEADBAND(raw - accel.xZeroOffset, accel.xDeadBand))/
                ACCEL_COUNT_PER_G;
    }

    TExitMsg(("x=%5.1f", value));
    return rc;
}   //AccelGetX

/**
 *  This function gets the current value of the accelerometer Y-axis in unit G.
 *
 *  @param accel Points to the ACCEL structure.
 *  @param value Returns the value in G.
 *
 *  @return Returns true if no error, false otherwise.
 */
bool
AccelGetY(
    __in  ACCEL &accel,
    __out float &value
    )
{
    bool rc;
    int raw;
    int dummy;

    TFuncName("AccelGetY");
    TLevel(API);
    TEnter();

#ifdef HTSMUX_STATUS
    rc = (accel.accelFlags & ACCELF_HTSMUX)?
            HTACreadAllAxes((tMUXSensor)accel.sensorID, dummy, raw, dummy):
            HTACreadAllAxes((tSensors)accel.sensorID, dummy, raw, dummy);
#else
    rc = HTACreadAllAxes((tSensors)accel.sensorID, dummy, raw, dummy));
#endif
    if (rc == true)
    {
        value = (float)(DEADBAND(raw - accel.yZeroOffset, accel.yDeadBand))/
                ACCEL_COUNT_PER_G;
    }

    TExitMsg(("x=%5.1f", value));
    return rc;
}   //AccelGetY

/**
 *  This function gets the current value of the accelerometer Z-axis in unit G.
 *
 *  @param accel Points to the ACCEL structure.
 *  @param value Returns the value in G.
 *
 *  @return Returns true if no error, false otherwise.
 */
bool
AccelGetZ(
    __in  ACCEL &accel,
    __out float &value
    )
{
    bool rc;
    int raw;
    int dummy;

    TFuncName("AccelGetZ");
    TLevel(API);
    TEnter();

#ifdef HTSMUX_STATUS
    rc = (accel.accelFlags & ACCELF_HTSMUX)?
            HTACreadAllAxes((tMUXSensor)accel.sensorID, dummy, dummy, raw):
            HTACreadAllAxes((tSensors)accel.sensorID, dummy, dummy, raw);
#else
    rc = HTACreadAllAxes((tSensors)accel.sensorID, dummy, dummy, raw));
#endif
    if (rc == true)
    {
        value = (float)(DEADBAND(raw - accel.zZeroOffset, accel.zDeadBand))/
                ACCEL_COUNT_PER_G;
    }

    TExitMsg(("x=%5.1f", value));
    return rc;
}   //AccelGetZ

/**
 *  This function gets the current values of the accelerometer on all axes in
 *  unit G.
 *
 *  @param accel Points to the ACCEL structure.
 *  @param xValue Returns the X value in G.
 *  @param yValue Returns the Y value in G.
 *  @param zValue Returns the Z value in G.
 *
 *  @return Returns true if no error, false otherwise.
 */
bool
AccelGetAllAxes(
    __in  ACCEL &accel,
    __out float &xValue,
    __out float &yValue,
    __out float &zValue
    )
{
    bool rc;
    int xRaw, yRaw, zRaw;

    TFuncName("AccelGetAllAxes");
    TLevel(API);
    TEnter();

#ifdef HTSMUX_STATUS
    rc = (accel.accelFlags & ACCELF_HTSMUX)?
            HTACreadAllAxes((tMUXSensor)accel.sensorID, xRaw, yRaw, zRaw):
            HTACreadAllAxes((tSensors)accel.sensorID, xRaw, yRaw, zRaw);
#else
    rc = HTACreadAllAxes((tSensors)accel.sensorID, xRaw, yRaw, zRaw));
#endif
    if (rc == true)
    {
        xValue = (float)(DEADBAND(xRaw - accel.xZeroOffset, accel.xDeadBand))/
                 ACCEL_COUNT_PER_G;
        yValue = (float)(DEADBAND(yRaw - accel.yZeroOffset, accel.yDeadBand))/
                 ACCEL_COUNT_PER_G;
        zValue = (float)(DEADBAND(zRaw - accel.zZeroOffset, accel.zDeadBand))/
                 ACCEL_COUNT_PER_G;
    }

    TExit();
    return rc;
}   //AccelGetAllAxes

#endif  //ifndef _ACCEL_H
