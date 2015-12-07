#include "PID.h"


//Start PID class
PID::PID(){
	integral = 0;
}



float PID::ComputePID(float Desired, float Meassured, float derivative, float Kp, float Ki, float Kd, float Dt){
	float error = Desired - Meassured;
	integral = integral + (error * Dt);

	float Output = (Kp * error) + (Ki * integral) + (Kd * derivative);

	return Output;


}

PID::~PID(){

}

//End PID class

//Start AvoidNaN class
AvoidNaN::AvoidNaN(){
    PreOut = 0;
}
float AvoidNaN::ReplaceNaN(float Input){
    if(Input <= 0 || Input > 0){    //Input != Input is always true for NaN
        PreOut = Input;
    }
    return PreOut;
}
AvoidNaN::~AvoidNaN(){
}
//End AvoidNaN class
