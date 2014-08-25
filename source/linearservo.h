//-------------------------------------------------------------------
#ifndef __linearservo_H__
#define __linearservo_H__
//-------------------------------------------------------------------

#include <arduino.h>

//-------------------------------------------------------------------

//-------------------------------------------------------------------

// Put yout declarations here

class LinearServo {

public:
	LinearServo(float);
	void SetTarget(float);
	void SetActual(float);
	float GetOutput();
	float Compute();

private:
	float actual;
	float target;
	float step;
	unsigned long lastTime;

};

//-------------------------------------------------------------------

//===================================================================
// -> DO NOT WRITE ANYTHING BETWEEN HERE...
// 		This section is reserved for automated code generation
// 		This process tries to detect all user-created
// 		functions in main_sketch.cpp, and inject their  
// 		declarations into this file.
// 		If you do not want to use this automated process,  
//		simply delete the lines below, with "&MM_DECLA" text 
//===================================================================
//---- DO NOT DELETE THIS LINE -- @MM_DECLA_BEG@---------------------
//---- DO NOT DELETE THIS LINE -- @MM_DECLA_END@---------------------
// -> ...AND HERE. This space is reserved for automated code generation!
//===================================================================


//-------------------------------------------------------------------
#endif
//-------------------------------------------------------------------
