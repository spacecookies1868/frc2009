/**
 * File Name: RAWCLib.cpp
 * Function: Header file: Team 968 libraries for custom functions
 * 
 * Author: Kiet Chau
 * Editor: Jakub Fiedorowicz
 * 
 * SVN Repository: https://svn2.dev.vivid-hosting.net/svn/968_Software
 * 
 * This code is confidential and cannot be released or published
 * without explicit permission in writing from the Author.
 * 
 */

#include <iostream.h>
#include "WPILib.h"


#ifndef RAWCLIB_H_
#define RAWCLIB_H_

/**
 * These values are used by AutoAddSeq when choosing
 * different modes of autonomous.
 */
#define Code1868 1
#define DRIVE_USING_VALUES 1
#define CAMERA_FIND_GREEN 2
#define CAMERA_FIND_PINK 3
#define DRIVE_BY_DISTANCE 4
#define TURN_BY_ANGLE 5

//#define !ds->GetDigitalIn(1)
#define SHOOT_BUTTON !ds->GetDigitalIn(2)
#define PICK_UP_BUTTON !ds->GetDigitalIn(3)
#define SHOOTER_SWITCH !ds->GetDigitalIn(4)
#define UN_JAM_BUTTON !ds->GetDigitalIn(5)
#define EXHAUST_BUTTON !ds->GetDigitalIn(6)

#ifdef Code1868 
	#define TCS_TOGGLE_SWITCH 1
	#define AUTON_VALID ds->GetDigitalIn(3)
	#define CORNER_SIDE ds->GetDigitalIn(7)
	#define LEFT_RIGHT ds->GetDigitalIn(8)
	#define AUTON_SWITCH 0 
#else
	#define TCS_TOGGLE_SWITCH ds->GetDigitalIn(7)
	#define AUTON_SWITCH !ds->GetDigitalIn(8)
#endif

/**
 * This class will be used by the 968 robotics team
 */
class RAWCLib
{
public:

	/**
	 * This struct will hold the different values specified
	 * by the user in AutoAddSeq
	 */
	typedef struct CMSEQ
	{
		//This member holds which mode of autonomous is to be run
		unsigned int Mode;
		//This member holds the value of the Y
		double Y;
		//This member holds the value of the X
		double X;
		//This member holds the amount of time to run the sequence for
		float Time;
	} CMSEQ;
	
	/**
	 * Resets the autonomous values
	 */
	static void AutoReset(void);
	
	/**
	 * Initializes autonomous mode
	 */
	static void AutoInit(void);
	
	/**
	 * Adds new autonomous nodes
	 */
	static void AutoAddSeq(int mde, double leftMotor, double rightMotor, float timeUser);
	
	/**
	 * Executes autonomous cycles with nodes in memory
	 */
	static void AutoExecCycle(int greenX, int greenY, int pinkX, int pinkY, bool foundColors);
	
	/**
	 * Limits output to either -1.0 or 1.0
	 */
	static float LimitOutput(float f);
	
	/**
	 * Squares the values and ensures output is positive
	 */
	static float SignSquare(float f);
	
	/**
	 * Converts normalized values into PWM(0-256) integer numbers
	 */
	static float ConvertScaledtoPWM(float f);
	static float RAWCLib::AnalogInScale(float oldx);
	
};

#endif
