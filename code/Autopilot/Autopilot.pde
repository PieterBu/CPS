	/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


#define THISFIRMWARE "CPS-Autopilot-Project"

#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <Wire.h>
#include <RC_Channel.h>
#include <AP_Motors.h>
#include <AP_Notify.h>
#include <AP_Curve.h>

#include <PID.h>
#include <SteadyStateControl.h>




//create object for AvoidNaN
AvoidNaN InTHETA, InPHI, InPSI, InVX, InVY, InVZ , InP, InR, InQ, InAX, InAY, InAZ, InALT;
float AX, AY, AZ, P, Q, R, THETA, PHI, PSI, ALT, VX, VY, VZ, pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot, PHI_dot, THETA_dot, PSI_dot;
enum FlightMode{
	stable,unstable,TakeOff, OnlyRollHeading, OnlyPitchRollHeading, OnlySpeedRollHeading
};
//Dt
float dt=0.000000125;

bool TakeOffBool = false;
bool fromEquilibriumToBorder = true;
bool stayStablePID=false;
uint16_t equiCounter=0;
	
//desired
float des_head = 90*0.0174533;
float des_speed = 45/2.23694;
float des_alt = -100;

SteadyStateControl ourSystem;
bool steadyFirstTime = true;

//TEST: Serial Communication
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(10, 11);
//int incomingByte = 0;   // for incoming serial data






// Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Declare PIDs
// NOTE: You need to implement your own PID class at ../libraries/PID/


// Number of data points sent by the simulator
#define DATAPOINTS 15

// Indices into array of data. Example: sample.data.f[I_PSI] gives you
// the heading.
#define I_AX 0
#define I_AY 1
#define I_AZ 2
#define I_P 3
#define I_Q 4
#define I_R 5
#define I_PHI 6
#define I_THETA 7
#define I_PSI 8
#define I_LAT 9
#define I_LON 10
#define I_ALT 11
#define I_VX 12
#define I_VY 13
#define I_VZ 14

// Data structure for each data sample. The data union is an array
// that can be accessed by bytes or by floats. Float access is for
// actually using data points. Byte access is for use by I2C code.
struct sample {
    union {
        float f[DATAPOINTS];
        uint8_t raw[DATAPOINTS * sizeof(float)];
    } data;
};

// Data storage
struct sample dataSample;

// Control targets
float targetHeading, targetAltitude, targetSpeed;

// Hard bounds on the pitch and roll of the plane
float hardMaxPitch, hardMaxRoll;

// Loop period in microseconds
#define PERIOD 20000

// Time of next PWM output
uint32_t nextWrite;

// Time of next serial output
uint32_t nextPrint;


// mod ensures that val is between min and max. This is mostly used
// for ensuring that angles are within the range -Pi to Pi.


// constrain bounds val to at least min and at most max.
float constrain(float val, float min, float max)
{
    if(val < min) {
        return min;
    } else if(val > max) {
        return max;
    } else {
        return val;
    }
}

// setup: called once at boot
void setup()
{
    hal.console->printf("\r\n\r\nStarting up\r\n\r\n");
    Wire.begin(); // Begin I2C communcation
    int i;
    // Initialize dataSample to all 0
    for(i = 0; i < DATAPOINTS; i++) {
        dataSample.data.f[i] = 0.0;
    }

    // Initial flight control targets, note altitude is down.
    targetHeading = M_PI/2;
    targetSpeed = 60;
    targetAltitude = -200;

    // Set maximum pitch and roll to 30 degrees.
    hardMaxPitch = 30.0*(M_PI/180.0);
    hardMaxRoll = 30.0*(M_PI/180.0);

    // Construct PIDs with gains


    // Enable PWM output on channels 0 to 3
    hal.rcout->enable_ch(0);
    hal.rcout->enable_ch(1);
    hal.rcout->enable_ch(2);
    hal.rcout->enable_ch(3);

    // Set next times for PWM output, serial output, and target change
    nextWrite = hal.scheduler->micros() + PERIOD;
    nextPrint = hal.scheduler->micros() + 1000000;
}

// loop: called repeatedly in a loop
void loop()
{
    uint32_t time = hal.scheduler->micros();
    if(time >= nextWrite) {
        nextWrite += PERIOD;

        // Read sensor data from emulation adapter
        uint8_t size = Wire.requestFrom(2, sizeof(dataSample.data));
        int i = 0;
        while(Wire.available()) {
            dataSample.data.raw[i] = Wire.read();
            i++;
        }

        ///hal.console->printf("IN: %f \n ",dataSample.data.f[I_PHI]);

		//Reading inputs creating objects to avoid NaN
		AX	= InAX.ReplaceNaN( dataSample.data.f[I_AX] );
        AY	= InAY.ReplaceNaN( dataSample.data.f[I_AY] );
        AZ	= InAZ.ReplaceNaN( dataSample.data.f[I_AZ] );

        P		= InP.ReplaceNaN( dataSample.data.f[I_P] );
        Q		= InQ.ReplaceNaN( dataSample.data.f[I_Q] );
        R		= InR.ReplaceNaN( dataSample.data.f[I_R] );

		THETA = InTHETA.ReplaceNaN( dataSample.data.f[I_THETA] );
        PHI 	= InPHI.ReplaceNaN( dataSample.data.f[I_PHI] );
        PSI 	= InPSI.ReplaceNaN( dataSample.data.f[I_PSI] );

        ALT 	= InALT.ReplaceNaN( dataSample.data.f[I_ALT] );

        VX 	= InVX.ReplaceNaN( dataSample.data.f[I_VX] );
        VY	= InVY.ReplaceNaN( dataSample.data.f[I_VY] );
        VZ	= InVZ.ReplaceNaN( dataSample.data.f[I_VZ] );

        //Start Calculation
        pn_dot = (cos(THETA)*cos(PSI))*VX +(sin(PHI)*sin(THETA)*cos(PSI)-cos(PHI)*sin(PSI))*VY+(cos(PHI)*sin(THETA)*cos(PSI)+sin(PHI)*sin(PSI))*VZ;
        pe_dot = (cos(THETA)*sin(PSI))*VX +(sin(PHI)*sin(THETA)*sin(PSI)+cos(PHI)*cos(PSI))*VY+(cos(PHI)*sin(THETA)*sin(PSI)-sin(PHI)*cos(PSI))*VZ;
        pd_dot = -sin(THETA)*VX+sin(PHI)*cos(THETA)*VY+cos(PHI)*cos(THETA)*VZ;

        u_dot = (R*VY-Q*VZ)+AX;
        v_dot = (P*VZ-R*VY)+AY;
        w_dot = (Q*VX-P*VY)+AZ;

        //CALCULATION OF PHI_dot, THETA_dot, PSI_dot
        PHI_dot = P+sin(PHI)*tan(THETA)*Q+cos(PHI)*tan(THETA)*R;
        THETA_dot = cos(PHI)*Q-sin(PHI)*R;
        PSI_dot = sin(PHI)/cos(THETA)*Q+cos(PHI)/cos(THETA)*R;
		
		
		//reset values if simulater resets
		if(ALT > -1){
			TakeOffBool = false;
		}
		
        if(FlightControllPID(des_head, des_alt, des_speed, TakeOff)){
			
			// Check where in the region we are
			
			// xTPx <= 1
			float XT[5] = {VX,VY,Q,THETA,ALT};
			float X[5][1] = {{VX},{VY},{Q},{THETA},{ALT}};
			float P[5][5] = {{0.0191,   -0.0008,   -0.0173,   -0.2298,    0.0000},
							 {-0.0008,    0.0010,    0.0008,    0.0101,   0.0000},
							 {-0.0173,    0.0008,    0.0175,    0.2205,   0.0000},
							 {-0.2298,    0.0101,    0.2205,    2.9396,   0.0000},
							 {0.0000,   0.0000,   0.0000,   0.0000,    0.0000}};
    
			float x_N[5][1] = {{20.0},
					   {0.0},
					   {0.0},
					   {0.0},
					   {-100.0}};
			
			// f1 = x_T * P
			float f1_steady[5] = {0.0,0.0,0.0,0.0,0.0};
			for(int col=0; col<5; col++){
				for(int inner=0; inner<5; inner++){
					f1_steady[col] += (x_N[inner][0]-XT[inner]) * P[inner][col];
				}
			}
			
			// f2 = f1 * x
			float f2_steady = 0.0;
			for(int inner=0; inner<5; inner++){
				f2_steady += f1_steady[inner] * (x_N[inner][0]-X[inner][0]);
			}

			hal.console->printf("x_T*P*x = %f  ", f2_steady);
			
			//if(f2_steady > 0.7 && f2_steady<1.0){
				//hal.console->printf("U= %f ",(X[0][0]-x_N[0][0]));
				//hal.console->printf(" W= %f ",(X[1][0]-x_N[1][0]));
				//hal.console->printf(" Q= %f ",(X[2][0]-x_N[2][0]));
				//hal.console->printf(" THETA= %f ",(X[3][0]-x_N[3][0]));
				//hal.console->printf(" ALT= %f ",(X[4][0]-x_N[4][0]));
			//}
			
			
			if(fromEquilibriumToBorder && f2_steady >0 && f2_steady < 0.5){
			// After take off we use unstable PID to brings the system to the border of the stability region
				float blah_ALT = ALT - 10;
				FlightControllPID(des_head, blah_ALT, des_speed, unstable);
		
			}
					
			// Then the steady state controller brings the system back to the equilibrium point
			else {//if(!stayStablePID && (fromEquilibriumToBorder || (!fromEquilibriumToBorder && f2_steady >0 /*&& f2_steady <1*/))){
				
				fromEquilibriumToBorder = false;
				
				if(f2_steady<0.01){
					
					equiCounter += 1;
					
					if(equiCounter > 1000){
						equiCounter = 0;
						fromEquilibriumToBorder=true;
						stayStablePID=false;
					}
					else{
						stayStablePID = true;
						//FlightControllPID(des_head, des_alt, des_speed, stable);
					}
				}
				
				if (stayStablePID){
					hal.console->printf(" unstable Counter = %d  ", equiCounter);
					FlightControllPID(des_head, des_alt, des_speed, stable);
				}
				else{
					
					hal.console->printf(" steady state controller ");
					// 3.28 converts m/s to ft/s
					float y[5][1] = {{0.0},{0.0},{0.0},{0.0},{0.0}};
					//float X[5][1] = {{VX},{VY},{Q},{THETA},{ALT}};
					float delta_x[5][1]={{0.0},{0.0},{0.0},{0.0},{0.0}};
					float dt=0.000000125;
					
					if(steadyFirstTime){
						ourSystem.initX(X);
						steadyFirstTime = false;
					}
					
					//ourSystem.ApplySteadyStateControl(dt,X,y,delta_x);
					
					float k[2][5] = {{-0.0204,   -0.0062,    0.0203,    0.0816,   0.0000},
									 {-0.0837,    0.0043,    0.0799,    1.0812,    0.0000}};
									 
					float superOut[2] = {0.0,0.0};
					for(int row = 0; row < 2; row++){				
						for(int inner=0; inner<5; inner++){
							superOut[row] += k[row][inner] * (x_N[inner][0]-X[inner][0]);
						}
					
					}
					
					float pitchPIDOut = /*-0.0168 + 0.01**/superOut[0];//0;//y[3][0];
					float speedPIDOut = 0.5 + superOut[1];//0;//y[0][0];///3.28;//sqrt(y[0][0]*y[0][0] + y[1][0]*y[1][0]);
					
					//hal.console->printf("pitchPIDOut = %f ", pitchPIDOut);
					//hal.console->printf("speedPIDOut = %f ", speedPIDOut);
					
					FlightControllPID(des_head, des_alt, des_speed, OnlyRollHeading);
					//FlightControllPID(des_head, des_alt, des_speed, OnlyPitchRollHeading);
					
					
					// Constrain all control surface outputs to the range -1 to 1
					float elevatorL = constrain(pitchPIDOut, -1, 1);//-0.0168;//constrain(pitchPIDOut, -1, 1);
					float elevatorR = constrain(pitchPIDOut, -1, 1);//-0.0168;//constrain(pitchPIDOut, -1, 1);
					float throttle = constrain(speedPIDOut, -1, 1);

					#define SERVO_MIN 1000 // Minimum duty cycle
					#define SERVO_MID 1500 // Mid duty cycle
					#define SERVO_MAX 2000 // Maximum duty cycle

					// Compute duty cycle for PWM output from generic control
					int16_t elevatorLOut = ((elevatorL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
					int16_t elevatorROut = ((elevatorR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
					int16_t throttleOut = ((throttle+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;

					// Output PWM
					hal.rcout->write(0, throttleOut);
					hal.rcout->write(1, elevatorLOut);
				}
			}
		}
	}
}

bool FlightControllPID(float des_head, float des_alt, float des_Speed, int ControllMode){
	// Gains for pidHeading
	float Kp_head = 5;
	float Ki_head = 0.1;
	float Kd_head = 0.005;

	// Gains for pidRoll
	float Kp_roll = 0.25;
	float Ki_roll = 0.5;
	float Kd_roll = 0.000001;

	// Gains for pidAltitude
	float Kp_alt = 0.85;
	float Ki_alt = 3.5;
	float Kd_alt = 0; 

	// Gains for pidClimbRate
	float Kp_climb = 0.0021;
	float Ki_climb = 0.06; 
	float Kd_climb = 0; //no derivative term

	// Gains for pidPitch
	float Kp_pitch = 0.31;
	float Ki_pitch = 0.73;
	float Kd_pitch = 0;

	// Gains for pidSpeed
	float Kp_speed = 0.5;
	float Ki_speed = 1;
	float Kd_speed = 0;
	
	PID Heading, Roll, Altitude, ClimbRate, Pitch, Speed;
	float headingPIDOut, rollPIDOut, altitudePIDOut, climbratePIDOut, pitchPIDOut, speedPIDOut;
	
	if(ControllMode == TakeOff){	
		if(!TakeOffBool){
			hal.console->printf("TakeOff\n ");
			headingPIDOut = Heading.ComputePID(des_head,PSI,PSI_dot,Kp_head,Ki_head,Kd_head,dt);
			// Compute roll PID
			rollPIDOut = Roll.ComputePID(headingPIDOut, PHI, PHI_dot, Kp_roll,Ki_roll,Kd_roll,dt);

			// Compute altitude PID
			altitudePIDOut = Altitude.ComputePID(des_alt,ALT,pd_dot,Kp_alt,Ki_alt,Kd_alt,dt);
			altitudePIDOut = -1*altitudePIDOut; //change sign

			// Compute climb rate PID
			climbratePIDOut = ClimbRate.ComputePID(altitudePIDOut,pd_dot,0,Kp_climb,Ki_climb,Kd_climb,dt);

			// Compute pitch PID
			pitchPIDOut = Pitch.ComputePID(climbratePIDOut, THETA, THETA_dot, Kp_pitch,Ki_pitch,Kd_pitch,dt);

			// Compute speed PID
			PID Speed;
			float VXYZ = sqrt(VX*VX+VY*VY+VZ*VZ);
			speedPIDOut = Pitch.ComputePID(des_speed, VXYZ, u_dot, Kp_speed,Ki_speed,Kd_speed,dt);
		}
		//# preexist code
		// Constrain all control surface outputs to the range -1 to 1
		float aileronL = -1*constrain(rollPIDOut, -1, 1);
		float aileronR = constrain(rollPIDOut, -1, 1);
		float elevatorL = constrain(pitchPIDOut, -1, 1);
		float elevatorR = constrain(pitchPIDOut, -1, 1);
		float throttle = constrain(speedPIDOut, -1, 1);
		#define SERVO_MIN 1000 // Minimum duty cycle
		#define SERVO_MID 1500 // Mid duty cycle
		#define SERVO_MAX 2000 // Maximum duty cycle

		// Compute duty cycle for PWM output from generic control
		int16_t aileronLOut = ((aileronL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t aileronROut = ((aileronR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t elevatorLOut = ((elevatorL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t elevatorROut = ((elevatorR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t throttleOut = ((throttle+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		// Output PWM
		hal.rcout->write(0, throttleOut);
		hal.rcout->write(1, elevatorLOut);
		hal.rcout->write(2, aileronROut);
		hal.rcout->write(3, aileronLOut);
	}
	
	
	if(ControllMode == stable){
		hal.console->printf("Stable flight \n ");
		headingPIDOut = Heading.ComputePID(des_head,PSI,PSI_dot,Kp_head,Ki_head,Kd_head,dt);
		// Compute roll PID
		rollPIDOut = Roll.ComputePID(headingPIDOut, PHI, PHI_dot, Kp_roll,Ki_roll,Kd_roll,dt);

		// Compute altitude PID
		altitudePIDOut = Altitude.ComputePID(des_alt,ALT,pd_dot,Kp_alt,Ki_alt,Kd_alt,dt);
		altitudePIDOut = -1*altitudePIDOut; //change sign

		// Compute climb rate PID
		climbratePIDOut = ClimbRate.ComputePID(altitudePIDOut,pd_dot,0,Kp_climb,Ki_climb,Kd_climb,dt);

		// Compute pitch PID
		pitchPIDOut = Pitch.ComputePID(climbratePIDOut, THETA, THETA_dot, Kp_pitch,Ki_pitch,Kd_pitch,dt);

		// Compute speed PID
		PID Speed;
		float VXYZ = sqrt(VX*VX+VY*VY+VZ*VZ);
		speedPIDOut = Pitch.ComputePID(des_speed, VXYZ, u_dot, Kp_speed,Ki_speed,Kd_speed,dt);
		//# preexist code
		// Constrain all control surface outputs to the range -1 to 1
		float aileronL = -1*constrain(rollPIDOut, -1, 1);
		float aileronR = constrain(rollPIDOut, -1, 1);
		float elevatorL = constrain(pitchPIDOut, -1, 1);
		float elevatorR = constrain(pitchPIDOut, -1, 1);
		float throttle = constrain(speedPIDOut, -1, 1);
		#define SERVO_MIN 1000 // Minimum duty cycle
		#define SERVO_MID 1500 // Mid duty cycle
		#define SERVO_MAX 2000 // Maximum duty cycle

		// Compute duty cycle for PWM output from generic control
		int16_t aileronLOut = ((aileronL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t aileronROut = ((aileronR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t elevatorLOut = ((elevatorL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t elevatorROut = ((elevatorR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t throttleOut = ((throttle+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		// Output PWM
		hal.rcout->write(0, throttleOut);
		hal.rcout->write(1, elevatorLOut);
		hal.rcout->write(2, aileronROut);
		hal.rcout->write(3, aileronLOut);
	}
	if(ControllMode == unstable){
		hal.console->printf("Unstable flight \n ");

		float Kp_alt_unstable = 20;
		float Ki_alt_unstable = 100;
		float Kd_alt_unstable = 5;
		float Ki_climb_unstable = 100;
		float Kd_pitch_unstable = 0.4;
		float Ki_pitch_unstable = 100;

		headingPIDOut = Heading.ComputePID(des_head,PSI,PSI_dot,Kp_head,Ki_head,Kd_head,dt);
		// Compute roll PID
		rollPIDOut = Roll.ComputePID(headingPIDOut, PHI, PHI_dot, Kp_roll,Ki_roll,Kd_roll,dt);

		// Compute altitude PID
		altitudePIDOut = Altitude.ComputePID(des_alt,ALT,pd_dot,Kp_alt_unstable,Ki_alt_unstable,Kd_alt_unstable,dt);
		altitudePIDOut = -1*altitudePIDOut; //change sign

		// Compute climb rate PID
		climbratePIDOut = ClimbRate.ComputePID(altitudePIDOut,pd_dot,0,Kp_climb,Ki_climb_unstable,Kd_climb,dt);

		// Compute pitch PID
		pitchPIDOut = Pitch.ComputePID(climbratePIDOut, THETA, THETA_dot, Kp_pitch ,Ki_pitch_unstable,Kd_pitch_unstable,dt);

		// Compute speed PID
		PID Speed;
		float VXYZ = sqrt(VX*VX+VY*VY+VZ*VZ);
		speedPIDOut = Pitch.ComputePID(des_speed, VXYZ, u_dot, Kp_speed,Ki_speed,Kd_speed,dt);
		//# preexist code
		// Constrain all control surface outputs to the range -1 to 1
		float aileronL = -1*constrain(rollPIDOut, -1, 1);
		float aileronR = constrain(rollPIDOut, -1, 1);
		float elevatorL = constrain(pitchPIDOut, -1, 1);
		float elevatorR = constrain(pitchPIDOut, -1, 1);
		float throttle = constrain(speedPIDOut, -1, 1);
		#define SERVO_MIN 1000 // Minimum duty cycle
		#define SERVO_MID 1500 // Mid duty cycle
		#define SERVO_MAX 2000 // Maximum duty cycle

		// Compute duty cycle for PWM output from generic control
		int16_t aileronLOut = ((aileronL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t aileronROut = ((aileronR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t elevatorLOut = ((elevatorL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t elevatorROut = ((elevatorR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t throttleOut = ((throttle+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		// Output PWM
		hal.rcout->write(0, throttleOut);
		hal.rcout->write(1, elevatorLOut);
		hal.rcout->write(2, aileronROut);
		hal.rcout->write(3, aileronLOut);
	
	}
	if(ControllMode == OnlyRollHeading){
		hal.console->printf("OnlyRollHeading \n ");
		headingPIDOut = Heading.ComputePID(des_head,PSI,PSI_dot,Kp_head,Ki_head,Kd_head,dt);
		// Compute roll PID
		rollPIDOut = Roll.ComputePID(headingPIDOut, PHI, PHI_dot, Kp_roll,Ki_roll,Kd_roll,dt);
		
		//# preexist code
		// Constrain all control surface outputs to the range -1 to 1
		float aileronL = -1*constrain(rollPIDOut, -1, 1);
		float aileronR = constrain(rollPIDOut, -1, 1);

		#define SERVO_MIN 1000 // Minimum duty cycle
		#define SERVO_MID 1500 // Mid duty cycle
		#define SERVO_MAX 2000 // Maximum duty cycle

		// Compute duty cycle for PWM output from generic control
		int16_t aileronLOut = ((aileronL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t aileronROut = ((aileronR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		// Output PWM
		hal.rcout->write(2, aileronROut);
		hal.rcout->write(3, aileronLOut);
	}
	if(ControllMode == OnlyPitchRollHeading){
		hal.console->printf("OnlyPitchRollHeading \n ");
		headingPIDOut = Heading.ComputePID(des_head,PSI,PSI_dot,Kp_head,Ki_head,Kd_head,dt);
		// Compute roll PID
		rollPIDOut = Roll.ComputePID(headingPIDOut, PHI, PHI_dot, Kp_roll,Ki_roll,Kd_roll,dt);
		// Compute altitude PID
		altitudePIDOut = Altitude.ComputePID(des_alt,ALT,pd_dot,Kp_alt,Ki_alt,Kd_alt,dt);
		altitudePIDOut = -1*altitudePIDOut; //change sign

		// Compute climb rate PID
		climbratePIDOut = ClimbRate.ComputePID(altitudePIDOut,pd_dot,0,Kp_climb,Ki_climb,Kd_climb,dt);

		// Compute pitch PID
		pitchPIDOut = Pitch.ComputePID(climbratePIDOut, THETA, THETA_dot, Kp_pitch ,Ki_pitch,Kd_pitch,dt);
		
		//# preexist code
		// Constrain all control surface outputs to the range -1 to 1
		float aileronL = -1*constrain(rollPIDOut, -1, 1);
		float aileronR = constrain(rollPIDOut, -1, 1);
		float elevatorL = constrain(pitchPIDOut, -1, 1);
		float elevatorR = constrain(pitchPIDOut, -1, 1);

		#define SERVO_MIN 1000 // Minimum duty cycle
		#define SERVO_MID 1500 // Mid duty cycle
		#define SERVO_MAX 2000 // Maximum duty cycle

		// Compute duty cycle for PWM output from generic control
		int16_t aileronLOut = ((aileronL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t aileronROut = ((aileronR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t elevatorLOut = ((elevatorL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t elevatorROut = ((elevatorR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;

		// Output PWM
		hal.rcout->write(1, elevatorLOut);
		hal.rcout->write(2, aileronROut);
		hal.rcout->write(3, aileronLOut);
	}
	if(ControllMode == OnlySpeedRollHeading){
		hal.console->printf("OnlySpeedRollHeading \n ");
		headingPIDOut = Heading.ComputePID(des_head,PSI,PSI_dot,Kp_head,Ki_head,Kd_head,dt);
		// Compute roll PID
		rollPIDOut = Roll.ComputePID(headingPIDOut, PHI, PHI_dot, Kp_roll,Ki_roll,Kd_roll,dt);
		// Compute speed PID
		PID Speed;
		float VXYZ = sqrt(VX*VX+VY*VY+VZ*VZ);
		speedPIDOut = Pitch.ComputePID(des_speed, VXYZ, u_dot, Kp_speed,Ki_speed,Kd_speed,dt);
		//# preexist code
		// Constrain all control surface outputs to the range -1 to 1
		float aileronL = -1*constrain(rollPIDOut, -1, 1);
		float aileronR = constrain(rollPIDOut, -1, 1);

		float throttle = constrain(speedPIDOut, -1, 1);
		#define SERVO_MIN 1000 // Minimum duty cycle
		#define SERVO_MID 1500 // Mid duty cycle
		#define SERVO_MAX 2000 // Maximum duty cycle

		// Compute duty cycle for PWM output from generic control
		int16_t aileronLOut = ((aileronL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t aileronROut = ((aileronR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		int16_t throttleOut = ((throttle+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
		// Output PWM
		hal.rcout->write(0, throttleOut);
		hal.rcout->write(2, aileronROut);
		hal.rcout->write(3, aileronLOut);
	}
	
	if(ControllMode == TakeOff){
		//take off condition
		if( abs(des_alt - ALT) < 0.5){
			TakeOffBool = true;
			return TakeOffBool;
		}else{
			return TakeOffBool;
		}
	}else{
		return true;
	}
}








AP_HAL_MAIN();
