#include "WPILib.h"
#include "Library2009.h"
#include "TCS.h"

extern Victor* lowerConA;
extern Victor* upperConA;
extern Victor* leftDrivemotor;
extern Victor* rightDrivemotor;
extern DriverStation* ds; 

extern Encoder* l_gearbox_encoder;
extern Encoder* l_follower_encoder;
extern Encoder* r_gearbox_encoder;
extern Encoder* r_follower_encoder;

class Workshop2009 : public IterativeRobot
{
private: 
	
	float joyX, joyY;
	float leftD, rightD;
	
	Gyro* myGyro;
	Encoder* l_gearbox_encoder;
	Encoder* l_follower_encoder;
	Encoder* r_gearbox_encoder;
	Encoder* r_follower_encoder;
	
public:
	Workshop2009(void) {
		GetWatchdog().SetEnabled(false);
		
		leftDrivemotor =  new Victor(3);
		rightDrivemotor = new Victor(2);
		ds = DriverStation::GetInstance(); 
		
		upperConA = new Victor(1);
		lowerConA = new Victor(4);
		
		myGyro = new Gyro(1);
		myGyro->SetSensitivity(.005);
		
		l_gearbox_encoder = new Encoder(1,2,false);
		l_follower_encoder = new Encoder(5,6,true);
		
		r_gearbox_encoder = new Encoder(3,4,true);
		r_follower_encoder = new Encoder(7,8,false);
		
		myGyro->Reset();
		l_gearbox_encoder->Start();
		l_follower_encoder->Start();
		r_gearbox_encoder->Start();
		r_follower_encoder->Start();
		
		leftDrivemotor->Set(0);
		rightDrivemotor->Set(0);
		upperConA->Set(0);
		lowerConA->Set(0);
		
		TCS::TCSInit();
	}
	void RobotInit(void) {
		
	}
	void DisabledInit(void) {
		
	}
	void AutonomousInit(void) {
		
	}
	void TeleopInit(void) {
		
	}
	void DisabledPeriodic(void)  {
	
	}
	void AutonomousPeriodic(void) {
		
	}
	void TeleopPeriodic(void) {
		
	} 
	void DisabledContinuous(void) {
	
	}
	void AutonomousContinuous(void)	{
	
	}
	void TeleopContinuous(void) {
		
		l_gearbox_encoder->Get();
		myGyro->GetAngle();		//degrees
	
		joyY = Library2009::analogJoystickInScale(ds->GetAnalogIn(2));
		joyX = Library2009::analogJoystickInScale(ds->GetAnalogIn(1));
	
	leftD = (joyX - joyY);
	rightD = (joyX + joyY);
	
	leftDrivemotor->Set(leftD);
	rightDrivemotor->Set(rightD);
	
	Library2009::conveyorControl();
	}
	
};

START_ROBOT_CLASS(Workshop2009);
