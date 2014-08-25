/*************************************************************
project: <type project name here>
author: <type your name here>
description: <type what this file does>
*************************************************************/

#include "linearservo.h"

LinearServo::LinearServo(float s) {
	actual = 0.0f;
	target = 0.0f;
	step = s;
	//lastTime = millis(); 
}

void LinearServo::SetTarget(float t) {
	target = t;
}

void LinearServo::SetActual(float a) {
	actual = a;
}

float LinearServo::GetOutput() {
	return actual;
}

float LinearServo::Compute() {

	float diff = target - actual;

	if(abs(diff) < step) { // Last step, just set to target and the job is done
		actual = target;
		return target; 
	}

	if(diff > 0.0f) {	// We're going positive :
		actual += step; // 	Increment actual by step
	} else {			// Negative :
		actual -= step; // 	Decrement actual by step
	}
	
	return actual;
	
}