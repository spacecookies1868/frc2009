#include "RAWCLib.h"
#include "TCS.h"


Timer accelTimer;
Timer yaccelTimer;
Timer l_PID_I_TIMER;
Timer r_PID_I_TIMER;

extern Gyro *ourGyro;
extern Encoder* l_gearbox_encoder;
extern Encoder* l_follower;
extern Encoder* r_gearbox_encoder;
extern Encoder* r_follower;
extern DigitalInput* teamIdentifier;
float l_PID_y, r_PID_y;

double lastTimer;
double currentTimer;
double printerTimer;
double lastPrinterTimer;

double l_deltaEncoder, l_deltaFollower;
double l_currentEncoder, l_followerEncoder;
double l_lastEncoder, l_lastFollower;
double r_deltaEncoder, r_deltaFollower;
double r_currentEncoder, r_followerEncoder;
double r_lastEncoder, r_lastFollower;

float l_currentFPS, l_realFPS;
float r_currentFPS, r_realFPS;
		
double l_PID_P_SPEED, l_PID_I_SPEED, l_PID_D_SPEED;
double l_PID_D_SPEED_LASTVAL;
double r_PID_P_SPEED, r_PID_I_SPEED, r_PID_D_SPEED;
double r_PID_D_SPEED_LASTVAL;

static const double PID_P_GAIN = 9.4;
static const double PID_I_GAIN = 0;
static const double PID_D_GAIN = 0.4;

bool l_TCS_ACTIVE;
bool r_TCS_ACTIVE;
bool printer;

void TCS::TCSInit(void){
	accelTimer.Start();
	yaccelTimer.Start();
	l_PID_I_TIMER.Start();
	r_PID_I_TIMER.Start();
	lastTimer = 0.0;
	currentTimer = 0.0;
	printerTimer = 0.0;
	lastPrinterTimer = 0.0;
	
	l_deltaEncoder = l_deltaFollower = 0;
	l_currentEncoder = l_followerEncoder = 0;
	l_lastEncoder = l_lastFollower = 0;
	r_deltaEncoder = r_deltaFollower = 0;
	r_currentEncoder = r_followerEncoder = 0;
	r_lastEncoder = r_lastFollower = 0;
	
	l_currentFPS = l_realFPS = 0;
	r_currentFPS = r_realFPS = 0;
			
	l_PID_P_SPEED = l_PID_I_SPEED = l_PID_D_SPEED = l_PID_y = 0;
	l_PID_D_SPEED_LASTVAL = 0;
	r_PID_P_SPEED = r_PID_I_SPEED = r_PID_D_SPEED = r_PID_y = 0;
	r_PID_D_SPEED_LASTVAL = 0;
	
	l_TCS_ACTIVE = false;
	r_TCS_ACTIVE = false;
	printer = false;
}
void TCS::TCSReset(void){
	lastTimer = 0.0;
	currentTimer = 0.0;
	printerTimer = 0.0;
	lastPrinterTimer = 0.0;
	
	l_gearbox_encoder->Reset();
	l_follower->Reset();
	r_gearbox_encoder->Reset();
	r_follower->Reset();
	ourGyro->Reset();
			
	//Start the timer 
	accelTimer.Reset();
	yaccelTimer.Reset();
	l_PID_I_TIMER.Reset();
	r_PID_I_TIMER.Reset();	
}
void TCS::TCSUpdate(float y, float x){
	currentTimer = accelTimer.Get();
			
	if(currentTimer > lastTimer + 0.01)
	{
		//13.58 ticks per inch using 256 count encoders w/o oversampling
		//Primary encoder for right gearbox
		l_currentEncoder = static_cast<double>(l_gearbox_encoder->Get());
		if(!teamIdentifier->Get())
		{
			l_currentEncoder = l_currentEncoder / 2;
		}
		l_deltaEncoder = ((l_currentEncoder-l_lastEncoder) / 6.75683);//			
		l_currentFPS = (l_deltaEncoder / 12);
		r_currentEncoder = static_cast<double>(r_gearbox_encoder->Get());
		if(!teamIdentifier->Get())
		{
			r_currentEncoder = r_currentEncoder / 2;
		}
		r_deltaEncoder = ((r_currentEncoder-r_lastEncoder) / 6.75683);//			
		r_currentFPS = (r_deltaEncoder / 12);
		
		
		//27.17 ticks per inch using 256 count encoders w/o oversampling
		//Follower wheel for right gearbox
		l_followerEncoder = static_cast<double>(l_follower->Get());
		if(!teamIdentifier->Get())
		{
			l_followerEncoder = l_followerEncoder / 2;
		}
		l_deltaFollower = ((l_followerEncoder - l_lastFollower) / 14.5513);//
		l_realFPS = (l_deltaFollower / 12);
		r_followerEncoder = static_cast<double>(r_follower->Get());
		if(!teamIdentifier->Get())
		{
			r_followerEncoder = r_followerEncoder / 2;
		}
		r_deltaFollower = ((r_followerEncoder - r_lastFollower) / 14.5513);//
		r_realFPS = (r_deltaFollower / 12);
		
		//unnecessary since Ki == 0
		if((l_PID_P_SPEED < 0.05) && (l_PID_P_SPEED > -0.05))
		{
			l_PID_I_TIMER.Reset();
		}
		if((r_PID_P_SPEED < 0.05) && (r_PID_P_SPEED > -0.05))
		{
			r_PID_I_TIMER.Reset();
		}
		if(y == 0 && x == 0){
			if(l_currentFPS >= l_realFPS)
			{
				l_TCS_ACTIVE = true;
				l_PID_P_SPEED = l_currentFPS - 0;
				l_PID_I_SPEED = l_PID_I_TIMER.Get()/10.00;
				l_PID_D_SPEED = l_PID_D_SPEED_LASTVAL - l_PID_P_SPEED;
				
				l_PID_y = (l_PID_P_SPEED * PID_P_GAIN * 2) + (l_PID_D_SPEED * PID_D_GAIN * 3) + (l_PID_I_SPEED * PID_I_GAIN);
			}
			else if(l_currentFPS <= l_realFPS)
			{
				l_TCS_ACTIVE = true;
				l_PID_P_SPEED = l_currentFPS - 0;
				l_PID_I_SPEED = l_PID_I_TIMER.Get()/10.00;
				l_PID_D_SPEED = l_PID_D_SPEED_LASTVAL - l_PID_P_SPEED;
				
				l_PID_y = (l_PID_P_SPEED * PID_P_GAIN * 2) + (l_PID_D_SPEED * PID_D_GAIN * 3) + (l_PID_I_SPEED * PID_I_GAIN);
			}
		}
		else if(l_currentFPS >= l_realFPS && (y <= 0))
		{
			l_TCS_ACTIVE = true;
			l_PID_P_SPEED = l_currentFPS - l_realFPS;
			l_PID_I_SPEED = l_PID_I_TIMER.Get()/10.00;
			l_PID_D_SPEED = l_PID_D_SPEED_LASTVAL - l_PID_P_SPEED;
			
			l_PID_y = (l_PID_P_SPEED * PID_P_GAIN) + (l_PID_D_SPEED * PID_D_GAIN) + (l_PID_I_SPEED * PID_I_GAIN);
		}
		else if(l_currentFPS <= l_realFPS && (y >= 0))
		{
			l_TCS_ACTIVE = true;
			l_PID_P_SPEED = l_currentFPS - l_realFPS;
			l_PID_I_SPEED = l_PID_I_TIMER.Get()/10.00;
			l_PID_D_SPEED = l_PID_D_SPEED_LASTVAL - l_PID_P_SPEED;
			
			l_PID_y = (l_PID_P_SPEED * PID_P_GAIN) + (l_PID_D_SPEED * PID_D_GAIN) + (l_PID_I_SPEED * PID_I_GAIN);
		}
		else
		{
			l_TCS_ACTIVE = false;
			l_PID_I_TIMER.Reset();
			l_PID_y = 0.0;
			l_PID_P_SPEED = 0.0;
			l_PID_D_SPEED = 0.0;
			l_PID_D_SPEED_LASTVAL = 0.0;
		}
		if(y == 0 && x == 0){
			if(r_currentFPS >= l_realFPS)
			{
				r_TCS_ACTIVE = true;
				r_PID_P_SPEED = r_currentFPS - 0;
				r_PID_I_SPEED = r_PID_I_TIMER.Get()/10.00;
				r_PID_D_SPEED = r_PID_D_SPEED_LASTVAL - r_PID_P_SPEED;
				
				r_PID_y = (r_PID_P_SPEED * PID_P_GAIN * 2) + (r_PID_D_SPEED * PID_D_GAIN * 3) + (r_PID_I_SPEED * PID_I_GAIN);
			}
			else if(r_currentFPS <= r_realFPS)
			{
				r_TCS_ACTIVE = true;
				r_PID_P_SPEED = r_currentFPS - 0;
				r_PID_I_SPEED = r_PID_I_TIMER.Get()/10.00;
				r_PID_D_SPEED = r_PID_D_SPEED_LASTVAL - r_PID_P_SPEED;
				
				r_PID_y = (r_PID_P_SPEED * PID_P_GAIN * 2) + (r_PID_D_SPEED * PID_D_GAIN * 3) + (r_PID_I_SPEED * PID_I_GAIN);
			}
		}
		else if(r_currentFPS >= r_realFPS && (y <= 0))
		{
			r_TCS_ACTIVE = true;
			r_PID_P_SPEED = r_currentFPS - r_realFPS;
			r_PID_I_SPEED = r_PID_I_TIMER.Get()/10.00;
			r_PID_D_SPEED = r_PID_D_SPEED_LASTVAL - r_PID_P_SPEED;
			
			r_PID_y = (r_PID_P_SPEED * PID_P_GAIN) + (r_PID_D_SPEED * PID_D_GAIN) + (r_PID_I_SPEED * PID_I_GAIN);
		}
		else if(r_currentFPS <= r_realFPS && (y >= 0))
		{
			r_TCS_ACTIVE = true;
			r_PID_P_SPEED = r_currentFPS - r_realFPS;
			r_PID_I_SPEED = r_PID_I_TIMER.Get()/10.00;
			r_PID_D_SPEED = r_PID_D_SPEED_LASTVAL - r_PID_P_SPEED;
			
			r_PID_y = (r_PID_P_SPEED * PID_P_GAIN) + (r_PID_D_SPEED * PID_D_GAIN) + (r_PID_I_SPEED * PID_I_GAIN);
		}
		else
		{
			r_TCS_ACTIVE = false;
			r_PID_I_TIMER.Reset();
			r_PID_y = 0.0;
			r_PID_P_SPEED = 0.0;
			r_PID_D_SPEED = 0.0;
			r_PID_D_SPEED_LASTVAL = 0.0;
		}
		
		/* *
		 * Set the last encoder values before being compared
		 * to current encoder values
		 */
		l_lastEncoder = l_currentEncoder;
		l_lastFollower = l_followerEncoder;
		l_PID_D_SPEED_LASTVAL = l_PID_P_SPEED;
		r_lastEncoder = r_currentEncoder;
		r_lastFollower = r_followerEncoder;
		r_PID_D_SPEED_LASTVAL = r_PID_P_SPEED;
						
		//Set the current time to compare to elapsed time
		lastTimer = currentTimer;
		//printf("Follower: %f\n", realFPS);
	
	}
}
