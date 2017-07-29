#pragma config(Sensor, S1,     HTCS,                sensorLowSpeed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
 * $Id: HTCS-test1.c 20 2009-12-08 22:59:13Z xander $
 */

/**
 * HTCS-driver.h provides an API for the HiTechnic Color Sensor.  This program
 * demonstrates how to use that API.
 *
 * Changelog:
 * - 0.1: Initial release
 * - 0.2: Changed the intro and info displayed
 * - 0.3: More comments
 *
 * Credits:
 * - Big thanks to HiTechnic for providing me with the hardware necessary to write and test this.
 *
 * License: You may use this code as you wish, provided you give credit where it's due.
 *
 * THIS CODE WILL ONLY WORK WITH ROBOTC VERSION 2.00 AND HIGHER.
 * Xander Soldaat (mightor_at_gmail.com)
 * 25 November 2009
 * version 0.3
 */

#include "drivers/common.h"
#include "drivers/HTCS-driver.h"

task main () {
  int red = 0;
  int green = 0;
  int blue = 0;
  int _color = 0;
  int _colorindex = 0;
  string _tmp;
  int val = 255;

  nxtDisplayCenteredTextLine(0, "HiTechnic");
  nxtDisplayCenteredBigTextLine(1, "COLOUR");
  nxtDisplayCenteredTextLine(3, "Test 1");
  nxtDisplayCenteredTextLine(5, "Connect sensor");
  nxtDisplayCenteredTextLine(6, "to S1");
  wait1Msec(2000);

  eraseDisplay();
  while (true) {
    // Read the currently detected colour from the sensor
    _color = HTCSreadColor(HTCS);

    // If colour == -1, it implies an error has occurred
    if (_color < 0) {
      nxtDisplayTextLine(4, "ERROR!!");
      wait1Msec(2000);
      StopAllTasks();
    }

    // Read the RGB values of the currently colour from the sensor
    // A return value of false implies an error has occurred
    if (!HTCSreadRGB(HTCS, red, green, blue)) {
      nxtDisplayTextLine(4, "ERROR!!");
      wait1Msec(2000);
      StopAllTasks();
    }

    nxtDisplayCenteredTextLine(0, "Color: %d", _color);
    nxtDisplayCenteredBigTextLine(1, "R  G  B");

    nxtEraseRect(0,10, 99, 41);
    nxtFillRect( 0, 10, 30, 10 + (red+1)/8);
    nxtFillRect(35, 10, 65, 10 + (green+1)/8);
    nxtFillRect(70, 10, 99, 10 + (blue+1)/8);
    StringFormat(_tmp, " %3d   %3d", red, green);
    nxtDisplayTextLine(7, "%s   %3d", _tmp, blue);

    wait1Msec(100);
  }
}

/*
 * $Id: HTCS-test1.c 20 2009-12-08 22:59:13Z xander $
 */