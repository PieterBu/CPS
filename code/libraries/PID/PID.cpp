#include "PID.h"

PID::PID(){
	integral = 0;
	PreOut = 0;
}



float PID::ComputePID(float Desired, float Meassured, float derivative, float Kp, float Ki, float Kd, float Dt){
	float error = Desired - Meassured;
	integral = integral + (error * Dt);
	
	float Output = (Kp * error) + (Ki * integral) + (Kd * derivative);
	
	if(Output <= 0 || Output > 0){
		PreOut = Output;
	}else{
		Output = PreOut;
	}
	
	return Output;
	
	
}

PID::~PID(){
	
}
