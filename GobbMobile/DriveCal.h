#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="DriveCal.h" />
///
/// <summary>
///     This module contains the drive calibration functions.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#include "JoystickDriver.c"
#include "..\trclib\trcdefs.h"
#include "..\trclib\dbgtrace.h"
#include "..\trclib\batt.h"
#include "..\trclib\menu.h"
#include "..\trclib\nxtbtn.h"
#include "..\trclib\gyro.h"
#include "..\trclib\pidctrl.h"
#include "..\trclib\drive.h"
DRIVE   g_Drive[1];
PIDCTRL g_PIDCtrl[3];
#include "..\trclib\piddrive.h"

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_MAIN

//
// Trace info.
//
#define TRACE_MODULES           (MOD_MAIN)
#define TRACE_LEVEL             TASK
#define MSG_LEVEL               INFO

//
// Global data.
//
bool     g_fCalDrive = false;
bool     g_fGyroDrive = false;
BATT     g_Batt;
MENU     g_Menu;
NXTBTN   g_NxtBtn;
GYRO     g_Gyro;
PIDDRIVE g_PIDDrive;
int      g_CalCtrl = 0;
float    g_StartPos = 0.0;

/**
 *  This function displays the menu and performs the action chosen.
 */
void
RunMenu(
    __in MENU &menu
    )
{
    int action;
    TFuncName("MenuRun");
    TLevel(FUNC);
    TEnter();

    action = MenuGetChoice(menu);
    eraseDisplay();

    switch (action)
    {
        case 0:
            g_fGyroDrive = !g_fGyroDrive;
            nxtDisplayTextLine(2, g_fGyroDrive? "Gyro Drive": "Encoder Drive");
            g_PIDDrive.turnPIDCtrl = g_fGyroDrive? 2: 1;
            break;

        case 1:
            PIDDriveSetTarget(g_PIDDrive,
                              96.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            g_fCalDrive = true;
            g_CalCtrl = 0;  //Calibrating PIDCtrlDrive
            g_StartPos = PIDCtrlGetInput(g_CalCtrl);
            ClearTimer(T1);
            break;

        case 2:
            PIDDriveSetTarget(g_PIDDrive,
                              -96.0, ENCODER_DRIVE_TOLERANCE,
                              0.0, ENCODER_TURN_TOLERANCE,
                              true);
            g_fCalDrive = true;
            g_CalCtrl = 0;  //Calibrating PIDCtrlDrive
            g_StartPos = PIDCtrlGetInput(g_CalCtrl);
            ClearTimer(T1);
            break;

        case 3:
            PIDDriveSetTarget(g_PIDDrive,
                              0.0, ENCODER_DRIVE_TOLERANCE,
                              360.0, ENCODER_TURN_TOLERANCE,
                              true);
            g_fCalDrive = true;
            g_CalCtrl = g_fGyroDrive? 2: 1; //Calibrating PIDCtrlTurn
            g_StartPos = PIDCtrlGetInput(g_CalCtrl);
            ClearTimer(T1);
            break;
    }

    TExit();
    return;
}   //RunMenu

/**
 *  This function handles the NXT button events.
 *
 *  @param nxtbtn Points to the NXTBTN structure.
 *  @param nxtButton Specifies the button that generated the event.
 *  @param fPressed If true, the button was pressed, otherwise it was
 *         released.
 */
void
NxtBtnEvent(
    __in NXTBTN &nxtbtn,
    __in int nxtButton,
    __in bool fPressed
    )
{
    TFuncName("NxtBtnEvent");
    TLevel(EVENT);
    TEnterMsg(("Button=%d,On=%d", nxtButton, (byte)fPressed));

    switch (nxtButton)
    {
        case kEnterButton:
            if (fPressed)
            {
                RunMenu(g_Menu);
            }
            break;

        case kExitButton:
            break;

        case kLeftButton:
            break;

        case kRightButton:
            break;
    }

    TExit();
    return;
}   //NxtBtnEvent

/**
 *  This function provides the input value for various PID controllers.
 *
 *  @param pidCtrlID Specifies the PID Controller ID.
 *
 *  @return Returns the input value for the PID controller.
 */
float
PIDCtrlGetInput(
    __in int pidCtrlID
    )
{
    float inputValue = 0.0;

    TFuncName("PIDCtrlGetInput");
    TLevel(CALLBK);
    TEnterMsg(("ID=%d", pidCtrlID));

    switch (pidCtrlID)
    {
        case 0:
            //
            // Encoder drive.
            //
            inputValue = (float)(nMotorEncoder[motorLeft] +
                                 nMotorEncoder[motorRight])*
                         DISTANCE_PER_CLICK/2.0;
            break;

        case 1:
            //
            // Encoder turn.
            //
            inputValue = (float)(nMotorEncoder[motorLeft] -
                                 nMotorEncoder[motorRight])*
                         DEGREES_PER_CLICK;
            break;

        case 2:
            //
            // Gyro turn.
            //
            inputValue = GyroGetHeading(g_Gyro);
            break;
    }

    TExitMsg(("=%5.1f", inputValue));
    return inputValue;
}   //PIDCtrlGetInput

/**
 *  This function handles the drive notification events.
 *
 *  @param drive Points to the DRIVE structure that generated the event.
 */
void
PIDDriveEvent(
    __in PIDDRIVE &pidDrive
    )
{
    TFuncName("PIDDriveEvent");
    TLevel(EVENT);
    TEnter();

    if (g_fCalDrive)
    {
        float pos;
        float elapsedTime;

        g_fCalDrive = false;
        nxtDisplayTextLine(0, "CalMode=Off");
        pos = PIDCtrlGetInput(g_CalCtrl) - g_StartPos;
        elapsedTime = (float)time1[T1]/1000.0;
        nxtDisplayTextLine(2, "Pos =%6.3f", pos);
        nxtDisplayTextLine(3, "Time=%6.3f", elapsedTime);
        nxtDisplayTextLine(4, "Vel =%6.3f", pos/elapsedTime);
    }

    TExit();
    return;
}   //PIDDriveEvent

/**
 *  This function initializes the robot and its subsystems.
 */
void
RobotInit()
{
    TFuncName("RobotInit");
    TLevel(INIT);
    TEnter();

    BattInit(g_Batt, 5, true);
    //
    // Init choice menu.
    //
    MenuInit(g_Menu, "Routines:", 0, 0);
    MenuAddChoice(g_Menu, "Gyro/Enc Drive");
    MenuAddChoice(g_Menu, "Forward 8 ft");
    MenuAddChoice(g_Menu, "Backward 8 ft");
    MenuAddChoice(g_Menu, "Turn 360");

    NxtBtnInit(g_NxtBtn, NXTBTNF_ENABLE_EVENTS);

    GyroInit(g_Gyro, gyro, 0);

    //
    // Intialize the Drive subsystem of the robot base.
    //
    DriveInit(g_Drive[0],
              motorLeft,
              motorRight,
              DISTANCE_PER_CLICK,
              DEGREES_PER_CLICK);
    //
    // Initialize the Encoder Drive.
    //
    PIDCtrlInit(g_PIDCtrl[0], 0,
                ENCODER_DRIVE_KP, ENCODER_DRIVE_KI, ENCODER_DRIVE_KD,
                MOTOR_MIN_VALUE, MOTOR_MAX_VALUE,
                0);
    PIDCtrlInit(g_PIDCtrl[1], 1,
                ENCODER_TURN_KP, ENCODER_TURN_KI, ENCODER_TURN_KD,
                MOTOR_MIN_VALUE, MOTOR_MAX_VALUE,
                0);
    PIDCtrlInit(g_PIDCtrl[2], 2,
                GYRO_TURN_KP, GYRO_TURN_KI, GYRO_TURN_KD,
                MOTOR_MIN_VALUE, MOTOR_MAX_VALUE,
                0);
    PIDDriveInit(g_PIDDrive,
                 0,     //Drive
                 0,     //EncoderDrive
                 1,     //EncoderTurn
                 PIDDRIVEF_ENABLE_EVENTS);

    TExit();
    return;
}   //RobotInit

/**
 *  This function processes all the high frequency tasks that needs to run
 *  more often than other tasks such as sensor integration tasks.
 */
void
HiFreqTasks()
{
    TFuncName("HiFreqTasks");
    TLevel(TASK);
    TEnter();

    GyroTask(g_Gyro);

    TExit();
    return;
}   //HiFreqTasks

/**
 *  This function processes all the input tasks.
 */
void
InputTasks()
{
    TFuncName("InputTasks");
    TLevel(TASK);
    TEnter();

    getJoystickSettings(joystick);
    NxtBtnTask(g_NxtBtn);

    TExit();
    return;
}   //InputTasks

/**
 *  This function processes all the main tasks.
 */
void
MainTasks()
{
#if 0
    static float prevInput = 0.0;
    static long prevTime = 0;
    static float peakInput = 0.0;
    static long prevPeakTime = 0;
#endif

    TFuncName("MainTasks");
    TLevel(TASK);
    TEnter();

    nxtDisplayTextLine(1, "L=%d,R=%d",
                       nMotorEncoder[motorLeft], nMotorEncoder[motorRight]);
    if (!g_fCalDrive)
    {
        int powerLeft  = NORMALIZE_DRIVE(DEADBAND_INPUT(joystick.joy1_y1),
                                         -128, 127);
        int powerRight = NORMALIZE_DRIVE(DEADBAND_INPUT(joystick.joy1_y2),
                                         -128, 127);
        //
        // TeleOp mode.
        //
        DriveTank(g_Drive[0], powerLeft, powerRight);
    }
#if 0
    else if (g_PIDCtrl[g_CalCtrl].input < prevInput)
    {
        if (prevInput > peakInput)
        {
            peakInput = prevInput;
        }
        nxtDisplayTextLine(3, "%d:Peak=%5.1f", g_CalCtrl, peakInput);
        nxtDisplayTextLine(4, "%d:Period=%d", g_CalCtrl, prevTime - prevPeakTime);
        prevPeakTime = prevTime;
    }
    else
    {
        prevInput = g_PIDCtrl[g_CalCtrl].input;
        prevTime = g_PIDCtrl[g_CalCtrl].time;
    }
#endif

    TExit();
    return;
}   //MainTasks

/**
 *  This function processes all the output tasks. Output tasks are where all
 *  the actions are taking place. All other tasks are just changing states of
 *  various objects. There is no action taken until the output tasks.
 */
void
OutputTasks()
{
    TFuncName("OutputTasks");
    TLevel(TASK);
    TEnter();

    PIDDriveTask(g_PIDDrive);
    DriveTask(g_Drive[0]);
    BattTask(g_Batt);

    TExit();
    return;
}   //OutputTasks

/**
 *  This task is the program entry point.
 */
task main()
{
    long nextTime;
    long currTime;

    TraceInit(TRACE_MODULES, TRACE_LEVEL, MSG_LEVEL);

    RobotInit();

    nextTime = nPgmTime;
    while (true)
    {
        TPeriodStart();
        currTime = nPgmTime;
        HiFreqTasks();
        if (currTime >= nextTime)
        {
            nextTime = currTime + LOOP_TIME;

            InputTasks();
            MainTasks();
            OutputTasks();
        }
        TPeriodEnd();

        wait1Msec(1);
    }
}   //main
