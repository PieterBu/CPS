#include "PID.h"


//Start PID class
PID::PID(){
	integral = 0;
	PreOut = 0;
}



float PID::ComputePID(float Desired, float Meassured, float derivative, float Kp, float Ki, float Kd, float Dt){
	float error = Desired - Meassured;
	integral = integral + (error * Dt);

	float Output = (Kp * error) + (Ki * integral) + (Kd * derivative);

	if(Output != Output){
		Output = PreOut;
	}else{
		PreOut = Output;
	}

	return Output;


}

PID::~PID(){

}

//End PID class

//Start AvoidNaN class
AvoidNaN::AvoidNaN(){
    PreOut = 0;
}
AvoidNaN::ReplaceNaN(float Input){
    float Output = 0;
    if(Input != Input){    //Input != Input is always true for NaN
        Output = PreOut;
    }else{
        PreOut = Input;
    }
    return Output;
}
AvoidNaN::~AvoidNaN(){
}
//End AvoidNaN class
