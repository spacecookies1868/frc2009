#ifndef TCS_H_
#define TCS_H_

#include "WPILib.h"

class TCS{
public:
	static void TCSInit(void);
	static void TCSReset(void);
	static void TCSUpdate(float y, float x);
	static float getFPS(Encoder* tempEncoder,float* tempEncoder_Prev, float ticksPerInch);
		
};
#endif
