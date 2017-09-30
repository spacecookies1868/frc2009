/**
 * File Name: RAWCLib.cpp
 * Function: Holds Team 968 libraries for custom functions
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
#include "TCS.h"
#include "RAWCLib.h"
#include "Timer.h"
#include "AxisCamera.h" 
#include "TrackAPI.h" 

const double MIN_PARTICLE_TO_IMAGE_PERCENT = 0.25;		// target is too small
const double MAX_PARTICLE_TO_IMAGE_PERCENT = 10.0;	// target is too close

const int MAXSEQ = 20;

Victor *driveLeftFront;
Victor *driveRightFront;
Victor *upperConA;
Victor *upperConB;
Victor *lowerConA;
Victor *lowerConB;
Victor *shooterA;
Victor *shooterB;
Gyro *ourGyro;
Encoder* l_gearbox_encoder;
Encoder* l_follower;
Encoder* r_gearbox_encoder;
Encoder* r_follower;
DigitalInput* teamIdentifier;
extern float l_PID_y, r_PID_y;
int cmdIter;
RAWCLib::CMSEQ cmd[MAXSEQ];
int cmdNum = 0;
float currentTime = 0;
float prevAngleError;

float pid_Error;
float yVal;
int lastTurn; //0 is right(postive values), 1 is left(negative values)
int lastValue;
int lastValueY;
int lastY;
bool gotFirstVal = false;
bool wasDriving = false;

bool camActive = false;

Timer autonTimer;

TrackingThreshold tdata1; 	// image data for tracking
TrackingThreshold tdata2; 	// image data for tracking

ParticleAnalysisReport par; // particle analysis reports

extern Timer yaccelTimer;

void RAWCLib::AutoReset(void)
{
	//cmdIter = 0;
	//Reset_Gyro_Angle();
	//Reset_Encoder_1_Count();
	
	autonTimer.Start();
	yaccelTimer.Start();

	l_gearbox_encoder->Reset();
	l_follower->Reset();
	r_gearbox_encoder->Reset();
	r_follower->Reset();
	ourGyro->Reset();
}

void RAWCLib::AutoInit(void)
{
	cmdNum = 0;
	
	driveLeftFront->Set(0.0);
	driveRightFront->Set(0.0);
	
	RAWCLib::AutoReset();
	
	// Start the camera.
	if (StartCameraTask(10, 0, k160x120, ROT_0) == -1)
	{
		printf("FAIL CAMERA!!!!!");
	}
	else
	{
		camActive = true;
	}
	
	tdata1 = GetTrackingData(GREEN, PASSIVE_LIGHT);
	tdata2 = GetTrackingData(PINK, PASSIVE_LIGHT);
}

void RAWCLib::AutoAddSeq(int mde, double y, double x, float timeUser)
{
	if (cmdNum < MAXSEQ)
	{
		printf("AutoSeq add: %d\r\n", cmdNum);

		if (!timeUser) timeUser = 1;

		// Create command sequence
		cmd[cmdNum].Mode = mde;
		cmd[cmdNum].Y = y;
		cmd[cmdNum].X = x;
		cmd[cmdNum].Time = timeUser;

		cmdNum ++;
	}
}

void RAWCLib::AutoExecCycle(int greenX, int greenY, int pinkX, int pinkY, bool foundColors)
{
	if(Code1868){
		upperConA->Set(0);
		lowerConA->Set(0);				
	}
	else{
		upperConA->Set(0);
		upperConB->Set(0);
		lowerConA->Set(0);
		lowerConB->Set(0);
		shooterA->Set(0);
		shooterB->Set(0); 
	}
	
	
	currentTime = autonTimer.Get();
	
	if(cmdIter < cmdNum)
	{
		//printf("Current Time: %f, Iter: %d, NUM: %d, L: %f, R: %f\n",  currentTime, cmdIter, cmdNum, cmd[cmdIter].LeftMotor, cmd[cmdIter].RightMotor);
		if(cmd[cmdIter].Mode == DRIVE_USING_VALUES)
		{
			
			TCS::TCSUpdate(cmd[cmdIter].Y,cmd[cmdIter].X);
			
			if(cmd[cmdIter].Y == 0)
			{
				yaccelTimer.Reset();
			}
			float ytemp = LimitOutput((cmd[cmdIter].Y*LimitOutput(yaccelTimer.Get() *1.8)));
			float left = LimitOutput(cmd[cmdIter].X - (ytemp - (l_PID_y*1.2)));
			float right = LimitOutput(cmd[cmdIter].X + (ytemp - (r_PID_y*1.2)));
			driveLeftFront->Set(left);
			driveRightFront->Set(right);
			if(cmd[cmdIter].Time <= currentTime)
			{	
				printf("Current iteration: %d\n", cmdIter);
				printf("CTime: %f, WTime: %f, NUM: %d, Y: %f, X: %f\n",
						currentTime,
						cmd[cmdIter].Time,
						cmdNum,
						cmd[cmdIter].Y,
						cmd[cmdIter].X);
				
				cmdIter ++;
				autonTimer.Reset();
				driveLeftFront->Set(0.0);
				driveRightFront->Set(0.0);
				ourGyro->Reset();
				l_gearbox_encoder->Reset();
				l_follower->Reset();
				r_gearbox_encoder->Reset();
				r_follower->Reset();
			}
		}
		else if(cmd[cmdIter].Mode == TURN_BY_ANGLE)
		{
						
			if(cmd[cmdIter].Y == 0)
			{
				yaccelTimer.Reset();
			}
			float currentAngle = ourGyro->GetAngle();
			printf("Current angle is: %f\n", currentAngle);
			float ytemp = LimitOutput((cmd[cmdIter].Y*LimitOutput(yaccelTimer.Get() *1.1)));
			
			float angleError = (cmd[cmdIter].Time - currentAngle);
			printf("Current angle error is: %f\n", angleError);
			float xtemp = (angleError / 25.0 + (angleError-prevAngleError)/200.0);
			
			if (cmd[cmdIter].Time >= 0)
				TCS::TCSUpdate(xtemp,cmd[cmdIter].Y);
			else 
				TCS::TCSUpdate((-1*xtemp),cmd[cmdIter].Y);
			
			/*if (cmd[cmdIter].Time < 0)
				xtemp = xtemp * -1;*/
			prevAngleError = angleError;
			printf("Current xtemp is: %f\n", xtemp);
						
				
			float left = LimitOutput(xtemp - (ytemp + (l_PID_y*1.2)));
			float right = LimitOutput(xtemp + (ytemp + (r_PID_y*1.2)));
			
			driveLeftFront->Set(left);
			driveRightFront->Set(right);
			
			if((currentAngle < (cmd[cmdIter].Time + 1.5)) && (currentAngle > (cmd[cmdIter].Time - 1.5)))
			{	
				printf("Current iteration: %d\n", cmdIter);
				printf("CTime: %f, WTime: %f, NUM: %d, Y: %f, X: %f\n",
						currentTime,
						cmd[cmdIter].Time,
						cmdNum,
						cmd[cmdIter].Y,
						cmd[cmdIter].X);
				
				cmdIter ++;
				autonTimer.Reset();
				driveLeftFront->Set(0.0);
				driveRightFront->Set(0.0);
				ourGyro->Reset();
				l_gearbox_encoder->Reset();
				l_follower->Reset();
				r_gearbox_encoder->Reset();
				r_follower->Reset();
			}
		}
		else if(cmd[cmdIter].Mode == DRIVE_BY_DISTANCE)
		{
			TCS::TCSUpdate(cmd[cmdIter].X,cmd[cmdIter].Y);
			if(cmd[cmdIter].Y == 0)
			{
				yaccelTimer.Reset();
			}
			float distance = (l_follower->Get() + r_follower->Get())/2.0;
			distance = (distance / 14.5513) / 12.0; //distance in feet traveled
			printf("Current distance is: %f\n", distance);
						
			float ytemp = LimitOutput((cmd[cmdIter].Y*LimitOutput(yaccelTimer.Get() *1.8)));
			float xtemp = cmd[cmdIter].X;
			printf("ytemp:%f xtemp:%f lpid:%f rpid:%f\n", ytemp, xtemp, l_PID_y, r_PID_y);
				
			float left = LimitOutput(xtemp - (ytemp + (l_PID_y*1.2)));
			float right = LimitOutput(xtemp + (ytemp + (r_PID_y*1.2)));
			
			driveLeftFront->Set(left);
			driveRightFront->Set(right);
			
			if(distance >= cmd[cmdIter].Time)
			{	
				printf("Current iteration: %d\n", cmdIter);
				printf("CTime: %f, WTime: %f, NUM: %d, Y: %f, X: %f\n",
						currentTime,
						cmd[cmdIter].Time,
						cmdNum,
						cmd[cmdIter].Y,
						cmd[cmdIter].X);
				
				cmdIter ++;
				autonTimer.Reset();
				driveLeftFront->Set(0.0);
				driveRightFront->Set(0.0);
				ourGyro->Reset();
				l_gearbox_encoder->Reset();
				l_follower->Reset();
				r_gearbox_encoder->Reset();
				r_follower->Reset();
			}
		}
		else if(cmd[cmdIter].Mode == CAMERA_FIND_GREEN)
		{	
				if(foundColors && (greenY < pinkY))
				{
					Timer().Start();
					gotFirstVal = true;
					wasDriving = true;
					
					pid_Error = (0 - greenX);
					pid_Error = pid_Error;
					
					
					//900(right), current (800)
						if(greenX < lastValue)
						{
							//This is when the target moves right of the camera view
							lastTurn = 1;
						}
						else if(greenX > lastValue)
						{
							//This is when the target moves left of the camera view
							lastTurn = 0;
						}
						if(greenY > lastValueY)
						{
							lastTurn = 2;
						}
					
					
					if((greenX < 40) && (greenX > -40))
					{
						pid_Error = 0;
						//yVal = -1;
					}
					else
					{
						pid_Error = pid_Error;
						//yVal = 0;
					}
					

					if((greenY > -900) && (greenY < -990))
					{
						yVal = 0.5;
					}
					else
					{
						yVal = 0.5;
					}
					
					
					float left = RAWCLib::LimitOutput(yVal - (pid_Error/2000));
					float right = -1 * RAWCLib::LimitOutput(yVal + (pid_Error/2000));
					
					//float left = RAWCLib::LimitOutput(1 - 0);
					//float right = -1 * RAWCLib::LimitOutput(1 + 0);
									
					
					driveLeftFront->Set(left);
					driveRightFront->Set(right);
					
					lastValue = greenX;
					lastValueY = greenY;
					printf("Light found: x: %i y: %i\n", greenX, greenY);
				} 
				else 
				{
						if(!foundColors)
						{
							printf("I don't see teh lightzeh! \n");
							driveLeftFront->Set(0.0);
							driveRightFront->Set(0.0);
						}
						else
						{
							Timer().Stop();
							currentTime = Timer().Get();
							if(lastTurn == 0)
							{
								printf("Light moved RIGHT, after going out of view\n");
								if(wasDriving && (currentTime > 0.4))
								{
									driveLeftFront->Set(-1);
									driveRightFront->Set(1);
									Wait(0.2);
									wasDriving = false;
								}
								else
								{
									driveLeftFront->Set(0.3);
									driveRightFront->Set(0.3);
								}
							}
							else if(lastTurn == 1)
							{
								printf("Light moved LEFT, after going out of view\n");
								if(wasDriving && (currentTime > 0.4))
								{
									driveLeftFront->Set(-1);
									driveRightFront->Set(1);
									Wait(0.2);
									wasDriving = false;
								}
								else
								{
									driveLeftFront->Set(-0.3);
									driveRightFront->Set(-0.3);
								}
							}
							else if(lastTurn == 2)
							{
								printf("Oh Shit! Robot is about to splouugeeeee\n");
								driveLeftFront->Set(0.8);
								driveRightFront->Set(-0.8);
							}
							else
							{
								printf("Did not get init val\n");
								driveLeftFront->Set(0.0);
								driveRightFront->Set(0.0);
							}
						}
				}
				if(cmd[cmdIter].Time <= currentTime)
				{	
					printf("Current iteration: %d\n", cmdIter);
					printf("CTime: %f, WTime: %f, NUM: %d, Y: %f, X: %f\n",
							currentTime,
							cmd[cmdIter].Time,
							cmdNum,
							cmd[cmdIter].Y,
							cmd[cmdIter].X);
					
					cmdIter ++;
					autonTimer.Reset();
					driveLeftFront->Set(0.0);
					driveRightFront->Set(0.0);
					ourGyro->Reset();
					l_gearbox_encoder->Reset();
					l_follower->Reset();
					r_gearbox_encoder->Reset();
					r_follower->Reset();
				}
		}
		else if(cmd[cmdIter].Mode == CAMERA_FIND_PINK)
		{	
				if(foundColors && (greenY > pinkY))
				{
					Timer().Start();
					gotFirstVal = true;
					wasDriving = true;
					
					pid_Error = (0 - greenX);
					pid_Error = pid_Error;
					
					
					//900(right), current (800)
						if(greenX < lastValue)
						{
							//This is when the target moves right of the camera view
							lastTurn = 1;
						}
						else if(greenX > lastValue)
						{
							//This is when the target moves left of the camera view
							lastTurn = 0;
						}
						if(greenY > lastValueY)
						{
							lastTurn = 2;
						}
					
					
					
					if((greenX < 40) && (greenX > -40))
					{
						pid_Error = 0;
						//yVal = -1;
					}
					else
					{
						pid_Error = pid_Error;
						//yVal = 0;
					}
					
					if((greenY > -900) && (greenY < -990))
					{
						yVal = 0.5;
					}
					else
					{
						yVal = 0.5;
					}
					
					
					
					float left = RAWCLib::LimitOutput(yVal - (pid_Error/2000));
					float right = -1 * RAWCLib::LimitOutput(yVal + (pid_Error/2000));
					
					//float left = RAWCLib::LimitOutput(1 - 0);
					//float right = -1 * RAWCLib::LimitOutput(1 + 0);
									
					
					driveLeftFront->Set(left);
					driveRightFront->Set(right);
					
					lastValue = greenX;
					lastValueY = greenY;
					printf("Light found: x: %i y: %i\n", greenX, greenY);
				} 
				else 
				{
						if(!foundColors)
						{
							printf("I don't see teh lightzeh! \n");
							driveLeftFront->Set(0.0);
							driveRightFront->Set(0.0);
						}
						else
						{
							Timer().Stop();
							currentTime = Timer().Get();
							if(lastTurn == 0)
							{
								printf("Light moved RIGHT, after going out of view\n");
								if(wasDriving && (currentTime > 0.4))
								{
									driveLeftFront->Set(-1);
									driveRightFront->Set(1);
									Wait(0.2);
									wasDriving = false;
								}
								else
								{
									driveLeftFront->Set(0.3);
									driveRightFront->Set(0.3);
								}
							}
							else if(lastTurn == 1)
							{
								printf("Light moved LEFT, after going out of view\n");
								if(wasDriving && (currentTime > 0.4))
								{
									driveLeftFront->Set(-1);
									driveRightFront->Set(1);
									Wait(0.2);
									wasDriving = false;
								}
								else
								{
									driveLeftFront->Set(-0.3);
									driveRightFront->Set(-0.3);
								}
							}
							else if(lastTurn == 2)
							{
								printf("Oh Shit! Robot is about to splouugeeeee\n");
								driveLeftFront->Set(0.8);
								driveRightFront->Set(-0.8);
							}
							else
							{
								printf("Did not get init val\n");
								driveLeftFront->Set(0.0);
								driveRightFront->Set(0.0);
							}
						}
				}
				if(cmd[cmdIter].Time <= currentTime)
				{	
					printf("Current iteration: %d\n", cmdIter);
					printf("CTime: %f, WTime: %f, NUM: %d, Y: %f, X: %f\n",
							currentTime,
							cmd[cmdIter].Time,
							cmdNum,
							cmd[cmdIter].Y,
							cmd[cmdIter].X);
					
					cmdIter ++;
					autonTimer.Reset();
					driveLeftFront->Set(0.0);
					driveRightFront->Set(0.0);
					ourGyro->Reset();
					l_gearbox_encoder->Reset();
					l_follower->Reset();
					r_gearbox_encoder->Reset();
					r_follower->Reset();
				}
		}		
	}
	else
	{
		driveLeftFront->Set(0.0);
		driveRightFront->Set(0.0);
	}
	//lastTime = GetTime();
		
}

float RAWCLib::LimitOutput(float f)
{
	if(f > 1.0)
	{
		return 1.0;
	}
	else if( f < -1.0)
	{
		return -1.0;
	}
	return f;
}

float RAWCLib::SignSquare(float f)
{
	if(f < 0)
	{
		return -1.0 * f * f;
	}
	else
	{
		return f * f;
	}
}

float RAWCLib::ConvertScaledtoPWM(float f)
{
	float tempPWM = 127.0;
	if(f > 0)
	{
		tempPWM = f * 255.0;
	}
	else if(f < 0)
	{
		tempPWM = (255.0 + (f * 255.0));
	}
	else
	{
		tempPWM = 127.0;
	}
	return tempPWM;
}
float RAWCLib::AnalogInScale(float oldx){
	double center = 500.0;
	double min = -500.0;
	double max = 500.0;
	double deadband = .05;
	float x = oldx - center;
	
	if ((x <= (center*deadband)) && (x >= (-1*center*deadband))){
		x = 0;
	}else if (x > (max - center*deadband)){
		x = 1;
	}else if (x < (min + center*deadband)){
		x = -1;
	}else if (x > 0){
		x = ((x - (center*deadband))/((max - center*deadband) - (center*deadband)));
	}else if (x < 0){
		x = ((x + (center*deadband))/(-1*((min + center*deadband) + (center*deadband))));
	}
	return x;
}


