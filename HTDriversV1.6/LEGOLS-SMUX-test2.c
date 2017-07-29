#pragma config(Sensor, S1,     HTSMUX,              sensorLowSpeed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
 * $Id: LEGOLS-SMUX-test2.c 20 2009-12-08 22:59:13Z xander $
 */

/**
 * LEGOLS-driver.h provides an API for the Lego Light Sensor.  This program
 * demonstrates how to use that API to calibrate the sensor connected to a SMUX.
 *
 * Changelog:
 * - 0.1: Initial release
 * - 0.2: More comments
 *
 * License: You may use this code as you wish, provided you give credit where it's due.
 *
 * THIS CODE WILL ONLY WORK WITH ROBOTC VERSION 2.00 AND HIGHER.
 * Xander Soldaat (mightor_at_gmail.com)
 * 25 November 2009
 * version 0.2
 */

#include "drivers/LEGOLS-driver.h"

task main () {
  int raw = 0;
  int nrm = 0;
  // Get control over the buttons
  nNxtButtonTask  = -2;

  // Before using the SMUX, you need to initialise the driver
  HTSMUXinit();

  // Tell the SMUX to scan its ports for connected sensors
  HTSMUXscanPorts(HTSMUX);

  // The sensor is connected to the first port
  // of the SMUX which is connected to the NXT port S1.
  // To access that sensor, we must use msensor_S1_1.  If the sensor
  // were connected to 3rd port of the SMUX connected to the NXT port S4,
  // we would use msensor_S4_3
  LSsetActive(msensor_S1_1);
  eraseDisplay();
  nxtDisplayTextLine(0, "Light Sensor Cal.");
  nxtDisplayTextLine(2, "Left:  set black");
  nxtDisplayTextLine(3, "Right: set white");
  nxtDisplayTextLine(7, "Grey:  exit");

  while (true) {
    switch(nNxtButtonPressed) {
      // if the left button is pressed calibrate the black value for the sensor
      case kLeftButton:
                        LScalLow(msensor_S1_1);
                        PlaySound(soundBeepBeep);
                        while(bSoundActive);
                        break;
      // if the left button is pressed calibrate the white value for the sensor
      case kRightButton:
                        LScalHigh(msensor_S1_1);
                        PlaySound(soundBeepBeep);
                        while(bSoundActive);
                        break;
    }
    nxtDisplayClearTextLine(5);
    nxtDisplayClearTextLine(6);

    // Read the raw value of the sensor
    raw = LSvalRaw(msensor_S1_1);

    // Read the normalised value of the sensor
    nrm = LSvalNorm(msensor_S1_1);

    // Display the raw and normalised values
    nxtDisplayTextLine(5, "R: %4d N: %4d", raw, nrm);

    // Display the values for black and white
    nxtDisplayTextLine(6, "B: %4d W: %4d", lslow, lshigh);
    wait1Msec(50);
  }
}

/*
 * $Id: LEGOLS-SMUX-test2.c 20 2009-12-08 22:59:13Z xander $
 */