#include "Library2009.h"
#include "TCS.h"
#include "WPILib.h"
extern Encoder* l_gearbox_encoder;
extern Encoder* l_follower_encoder;
extern Encoder* r_gearbox_encoder;
extern Encoder* r_follower_encoder;

float l_gearbox_FPS, l_follower_FPS, r_gearbox_FPS, r_follower_FPS;
float l_gearbox_PrevEncoder, l_follower_PrevEncoder, r_gearbox_PrevEncoder, r_follower_PrevEncoder;

Timer traction_Control_Timer;
float currentTimer, prevTimer;
void TCS::TCSInit(void){
	traction_Control_Timer.Start();
	currentTimer = 0;
	prevTimer = traction_Control_Timer.Get();
	l_gearbox_FPS = 0; 
	l_follower_FPS = 0;
	r_gearbox_FPS = 0;
	r_follower_FPS = 0;
	l_gearbox_PrevEncoder = 0; 
	l_follower_PrevEncoder = 0;
	r_gearbox_PrevEncoder = 0;
	r_follower_PrevEncoder = 0;
}

void TCS::TCSReset (void){
}

void TCS::TCSUpdate(float y, float x){
	currentTimer = traction_Control_Timer.Get();
	if(currentTimer > prevTimer + .01){
	/*	float current_l_gearbox_encoder = l_gearbox_encoder->Get();
		l_gearbox_FPS = current_l_gearbox_encoder - l_gearbox_PrevEncoder;
		l_gearbox_PrevEncoder = current_l_gearbox_encoder;
		l_gearbox_FPS = ((l_gearbox_FPS / 6.75683) / 12.0);
		
		float current_r_gearbox_encoder = r_gearbox_encoder->Get();
		r_gearbox_FPS = current_r_gearbox_encoder - r_gearbox_PrevEncoder;
		r_gearbox_PrevEncoder = current_r_gearbox_encoder;
		r_gearbox_FPS = ((r_gearbox_FPS / 6.75683) / 12.0);
		
		float current_l_follower_encoder = l_follower_encoder->Get();
		l_follower_FPS = current_l_follower_encoder - l_follower_PrevEncoder;
		l_follower_PrevEncoder = current_l_follower_encoder;
		l_follower_FPS = ((l_follower_FPS / 6.75683) / 12.0);
		
		float current_r_follower_encoder = r_follower_encoder->Get();
		r_follower_FPS = current_r_follower_encoder - r_follower_PrevEncoder;
		r_follower_PrevEncoder = current_r_follower_encoder;
		r_follower_FPS = ((r_follower_FPS / 6.75683) / 12.0);
*/
		
		l_gearbox_FPS = getFPS(l_gearbox_encoder, &l_gearbox_PrevEncoder, 6.75683);
		l_follower_FPS = getFPS(l_gearbox_encoder, &l_follower_PrevEncoder, 6.75683);
		r_gearbox_FPS = getFPS(l_gearbox_encoder, &r_gearbox_PrevEncoder, 14.5513);
		r_follower_FPS = getFPS(l_gearbox_encoder, &r_follower_PrevEncoder, 14.5513);
	prevTimer = currentTimer;
	}
}
float getFPS(Encoder* tempEncoder,float* tempEncoder_Prev, float ticksPerInch){
	float temp_FPS;
	float currentEncoder = tempEncoder->Get();
	temp_FPS = currentEncoder - *tempEncoder_Prev;
	*tempEncoder_Prev = currentEncoder;
	temp_FPS = ((temp_FPS / ticksPerInch) / 12.0);
	return temp_FPS;
	
}


