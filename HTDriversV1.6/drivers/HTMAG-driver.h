/*!@addtogroup HiTechnic
 * @{
 * @defgroup HTMAG Magnetic Field Sensor
 * HiTechnic Magnetic Field Sensor
 * @{
 */

/*
 * $Id: HTMAG-driver.h 33 2010-09-24 10:11:54Z soldaatx $
 */

#ifndef __HTMAG_H__
#define __HTMAG_H__
/** \file HTMAG-driver.h
 * \brief HiTechnic Magnetic Field Sensor driver
 *
 * HTMAG-driver.h provides an API for the HiTechnic Magnetic Field Sensor.
 *
 * Changelog:
 * - 0.1: Initial release
 *
 * Credits:
 * - Big thanks to HiTechnic for providing me with the hardware necessary to write and test this.
 *
 * License: You may use this code as you wish, provided you give credit where its due.
 *
 * THIS CODE WILL ONLY WORK WITH ROBOTC VERSION 2.00 AND HIGHER.
 * \author Xander Soldaat (mightor_at_gmail.com)
 * \date 27 July 2010
 * \version 0.1
 * \example HTMAG-test1.c
 * \example HTMAG-SMUX-test1.c
 */

#pragma systemFile

#ifndef __COMMON_H__
#include "common.h"
#endif

int HTMAGreadVal(tSensors link);
int HTMAGreadVal(tMUXSensor muxsensor);
int HTMAGreadRaw(tSensors link);
int HTMAGreadRaw(tMUXSensor muxsensor);
int HTMAGstartCal(tSensors link);
int HTMAGstartCal(tMUXSensor muxsensor);
int HTMAGreadCal(tSensors link);
int HTMAGreadCal(tMUXSensor muxsensor);
void HTMAGsetCal(tSensors link, int bias);
void HTMAGsetCal(tMUXSensor muxsensor, int bias);

int HTMAG_bias[][] = {{512, 512, 512, 512}, /*!< Array for bias values.  Default is 512 */
                          {512, 512, 512, 512},
                          {512, 512, 512, 512},
                          {512, 512, 512, 512}};

/**
 * Read the value of the Magnetic Field Sensor
 * @param link the HTMAG port number
 * @return the value of the Magnetic Field Sensor (-200 to +200)
 */
int HTMAGreadVal(tSensors link) {
  // Make sure the sensor is configured as type sensorRawValue
  if (SensorType[link] != sensorRawValue) {
    SetSensorType(link, sensorRawValue);
    wait1Msec(100);
  }

  return (SensorValue[link] - HTMAG_bias[link][0]);
}


/**
 * Read the value of the Magnetic Field Sensor
 * @param muxsensor the SMUX sensor port number
 * @return the value of the Magnetic Field Sensor (-200 to +200)
 */
int HTMAGreadVal(tMUXSensor muxsensor) {
  return HTSMUXreadAnalogue(muxsensor) - HTMAG_bias[SPORT(muxsensor)][MPORT(muxsensor)];
}


/**
 * Read the raw value of the Magnetic Field Sensor
 * @param link the HTMAG port number
 * @return the value of the Magnetic Field Sensor (approx 300 to 700)
 */
int HTMAGreadRaw(tSensors link) {
  // Make sure the sensor is configured as type sensorRawValue
  if (SensorType[link] != sensorRawValue) {
    SetSensorType(link, sensorRawValue);
    wait1Msec(100);
  }

  return SensorValue[link];
}


/**
 * Read the raw value of the Magnetic Field Sensor
 * @param muxsensor the SMUX sensor port number
 * @return the value of the Magnetic Field Sensor (approx 300 to 700)
 */
int HTMAGreadRaw(tMUXSensor muxsensor) {
  return HTSMUXreadAnalogue(muxsensor);
}


/**
 * Calibrate the sensor by calculating the average bias of 5 raw readings.
 * @param link the HTMAG port number
 * @return the new bias value for the sensor
 */
int HTMAGstartCal(tSensors link) {
  int _avgdata = 0;

  // Make sure the sensor is configured as type sensorRawValue
  if (SensorType[link] != sensorRawValue) {
    SetSensorType(link, sensorRawValue);
    wait1Msec(100);
  }

  // Take 5 readings and average them out
  for (int i = 0; i < 5; i++) {
    _avgdata += SensorValue[link];
    wait1Msec(50);
  }

  // Store new bias
  HTMAG_bias[link][0] = (_avgdata / 5);

  // Return new bias value
  return HTMAG_bias[link][0];
}


/**
 * Calibrate the Magnetic Field Sensor by calculating the average bias of 5 raw readings.
 * @param muxsensor the SMUX sensor port number
 * @return the new bias value for the Magnetic Field Sensor
 */
int HTMAGstartCal(tMUXSensor muxsensor) {
  int _avgdata = 0;

  // Take 5 readings and average them out
  for (int i = 0; i < 5; i++) {
    _avgdata += HTSMUXreadAnalogue(muxsensor);
    wait1Msec(50);
  }

  // Store new bias
  HTMAG_bias[SPORT(muxsensor)][MPORT(muxsensor)] = (_avgdata / 5);

  // Return new bias value
  return HTMAG_bias[SPORT(muxsensor)][MPORT(muxsensor)];
}


/**
 * Override the current bias for the sensor manually
 * @param link the HTMAG port number
 * @param bias the new bias to be used
 */
void HTMAGsetCal(tSensors link, int bias) {
  HTMAG_bias[link][0] = bias;
}


/**
 * Override the current bias for the sensor manually
 * @param muxsensor the SMUX sensor port number
 * @param bias the new bias to be used
 */
void HTMAGsetCal(tMUXSensor muxsensor, int bias) {
  HTMAG_bias[SPORT(muxsensor)][MPORT(muxsensor)] = bias;
}


/**
 * Retrieve the current bias for the sensor
 * @param link the HTMAG port number
 * @return the bias value for the sensor
 */
int HTMAGreadCal(tSensors link) {
  return HTMAG_bias[link][0];
}


/**
 * Retrieve the current bias for the sensor
 * @param muxsensor the SMUX sensor port number
 * @return the bias value for the sensor
 */
int HTMAGreadCal(tMUXSensor muxsensor) {
  return HTMAG_bias[SPORT(muxsensor)][MPORT(muxsensor)];
}


#endif // __HTMAG_H__

/*
 * $Id: HTMAG-driver.h 33 2010-09-24 10:11:54Z soldaatx $
 */
/* @} */
/* @} */
