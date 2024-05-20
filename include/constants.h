/*---------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       constants.h                                               */
/*    Author:       C:\Users\nerij                                            */
/*    Created:      Wed Apr 19 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <math.h>

//----------------- Robot constants -----------------//
const double WHEEL_DIAMETER = 82;    //mm
const double WHEEL_TRAVEL   = M_PI * WHEEL_DIAMETER;
// Distance between the wheels width
const double TRACK_WIDTH    = 270;      //mm
// Distance between the wheels length
const double TRACK_BASE     = 188;      //mm
const double EXT_GEAR_RATIO = 1;    

//----------------- Autonomous time -----------------//
const double DIST_TO_BALL_1       = 30;
const double TURN_TO_BALL_1       = 30;


//------------------ User control -------------------//
const double JOYSTICK_DEADBAND    = 10;