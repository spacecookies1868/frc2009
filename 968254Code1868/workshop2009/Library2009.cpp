#include "Library2009.h"
#include "WPILib.h"

#define JOYCENTER 500.0
#define JOYMIN 0.0
#define JOYMAX 1000.0
#define WHEELCENTER 500.0
#define WHEELMIN 0.0
#define WHEELMAX 1000.0

Victor* lowerConA;
Victor* upperConA;
Victor* leftDrivemotor;
Victor* rightDrivemotor;

DriverStation* ds;

Encoder* l_gearbox_encoder;
Encoder* l_follower_encoder;
Encoder* r_gearbox_encoder;
Encoder* r_follower_encoder;

float Library2009::analogWheelInScale(float oldx){
	float center = WHEELCENTER;
	float min = WHEELMIN;
	float max = WHEELMAX;
	float deadband = .10;
	float upperDead = (max - center)*deadband;
	float lowerDead = (center - min)*deadband;
	float lowerRange = (max-center)-2*upperDead;
	float upperRange = (center-min)-2*lowerDead;
	float x = oldx - center;
	
	if ((x <= upperDead) && (x >= (-1*lowerDead))){
		x = 0;
	}else if (x > (max - center - upperDead)) {
		x = 1;
	}else if (x < (min - center + lowerDead)) {
		x = -1;
	}else if (x > 0) {
		x = (x- upperDead)/upperRange;
	}else if (x < 0) {
		x = (x + lowerDead)/lowerRange;
	}
	return x;
}
	float Library2009::analogJoystickInScale(float oldx){
		float center = JOYCENTER;
		float min = JOYMIN;
		float max = JOYMAX;
		float deadband = .10;
		float upperDead = (max - center)*deadband;
		float lowerDead = (center - min)*deadband;
		float lowerRange = (max-center)-2*upperDead;
		float upperRange = (center-min)-2*lowerDead;
		float x = oldx - center;
		
		if ((x <= upperDead) && (x >= (-1*lowerDead))){
			x = 0;
		}else if (x > (max - center - upperDead)) {
			x = 1;
		}else if (x < (min - center + lowerDead)) {
			x = -1;
		}else if (x > 0) {
			x = (x- upperDead)/upperRange;
		}else if (x < 0) {
			x = (x + lowerDead)/lowerRange;
		}
		return x;
	}

void Library2009::conveyorControl(void){
	if(SHOOT_BUTTON){
			upperConA->Set(-1);
			lowerConA->Set(1);
		}else if(PICK_UP_BUTTON){
			upperConA->Set(0);
			lowerConA->Set(1);
		}else if(EXAUHST_BUTTON){
			upperConA->Set(0);
			lowerConA->Set(-1);
		}else if(UNJAM_BUTTON){
			upperConA->Set(1);
			lowerConA->Set(-1);
		}else{
			upperConA->Set(0);
			lowerConA->Set(0);
		}


}
	
