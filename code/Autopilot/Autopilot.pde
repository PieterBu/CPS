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


//create object for AvoidNaN
AvoidNaN InTHETA, InPHI, InPSI, InVX, InVY, InVZ , InP, InR, InQ, InAX, InAY, InAZ, InALT;
float AX, AY, AZ, P, Q, R, THETA, PHI, PSI, ALT, VX, VY, VZ, pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot, PHI_dot, THETA_dot, PSI_dot;
bool TakeOff();

//TEST: Serial Communication
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(10, 11);
//int incomingByte = 0;   // for incoming serial data



//Dt
float dt=0.000000125;

//desired
float des_head = 1.5;
float des_speed = 20;

float des_roll = 0; 
float des_alt = -125;
float des_climb = 7;
float des_pitch = 90; 
 
 
// Gains for pidHeading
float Kp_head = 5;
float Ki_head = 0.1;
float Kd_head = 0.005;

// Gains for pidRoll
float Kp_roll = 0.25;
float Ki_roll = 0.5;
float Kd_roll = 0.000001;

// Gains for pidAltitude
float Kp_alt = 0.85;//5.5;//5;
float Ki_alt = 3.5; //5;
float Kd_alt = 0; //0.005;

// Gains for pidClimbRate
float Kp_climb = 0.0021;//3;
float Ki_climb = 0.06; //1;
float Kd_climb = 0; //no derivative term

// Gains for pidPitch
float Kp_pitch = 0.31; //0.00005;
float Ki_pitch = 0.73; //1;
float Kd_pitch = 0; //0.00000005;

// Gains for pidSpeed
float Kp_speed = 0.5;
float Ki_speed = 1;
float Kd_speed = 0;


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

		
		float speedPIDOut, pitchPIDOut, rollPIDOut;
        if(TakeOff()){
			
			Ki_climb = 1; //Make it unstable
			
			// Compute heading PID
			PID Heading;
			float headingPIDOut = Heading.ComputePID(PSI,PSI,PSI_dot,Kp_head,Ki_head,Kd_head,dt);
			headingPIDOut = 5;																		//somthing is not working her	
			// Compute roll PID 
			PID Roll;
			rollPIDOut = Roll.ComputePID(des_roll, PHI, PHI_dot, Kp_roll,Ki_roll,Kd_roll,dt);
			// Compute altitude PID
			PID Altitude;   
			float altitudePIDOut = Altitude.ComputePID(des_alt,ALT,pd_dot,Kp_alt,Ki_alt,Kd_alt,dt);
			altitudePIDOut = -1*altitudePIDOut; //change sign
			// Compute climb rate PID
			PID ClimbRate; 
			float climbratePIDOut = ClimbRate.ComputePID(altitudePIDOut,pd_dot,0,Kp_climb,Ki_climb,Kd_climb,dt);
			// Compute pitch PID
			PID Pitch;
			pitchPIDOut = Pitch.ComputePID(climbratePIDOut, THETA, THETA_dot, Kp_pitch,Ki_pitch,Kd_pitch,dt);
			// Compute speed PID
			PID Speed;
			speedPIDOut = Pitch.ComputePID(des_speed, VX, u_dot, Kp_speed,Ki_speed,Kd_speed,dt);
			
			//Simplex switch
			if(false){
			
			}
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
        if(time >= nextPrint) {
            // Print some status
            nextPrint += 1000000;
            hal.console->printf("** PERIOD **\r\n");
        }
        // Output PWM
        hal.rcout->write(0, throttleOut);
        hal.rcout->write(1, elevatorLOut);
        hal.rcout->write(2, aileronROut);
        hal.rcout->write(3, aileronLOut);
    }
}

bool TakeOff(){
	static bool TakeOff = false;
	if(!TakeOff){
		// Compute heading PID
		PID Heading;
		float headingPIDOut = Heading.ComputePID(PSI,PSI,PSI_dot,Kp_head,Ki_head,Kd_head,dt);
		headingPIDOut = 5;																		//somthing is not working here
		/*
		hal.console->printf("PSI: %f \n ", PSI);
		hal.console->printf("headingError: %f \n ",des_head - PSI);
		hal.console->printf("headingPIDOut: %f \n ", headingPIDOut);
		*/
		
		// Compute roll PID 
		PID Roll;
		float rollPIDOut = Roll.ComputePID(des_roll, PHI, PHI_dot, Kp_roll,Ki_roll,Kd_roll,dt);
		/*
		hal.console->printf("PHI: %f \n ", PHI);
		hal.console->printf("rollError: %f \n ",des_roll - PHI);
		hal.console->printf("rollPIDOut: %f \n ", rollPIDOut);
		*/
		
		// Compute altitude PID
		PID Altitude;   
		float altitudePIDOut = Altitude.ComputePID(des_alt,ALT,pd_dot,Kp_alt,Ki_alt,Kd_alt,dt);
		altitudePIDOut = -1*altitudePIDOut; //change sign
		/*
		hal.console->printf("ALT: %f \n ", ALT);
		hal.console->printf("altitudeError: %f \n ",des_alt - ALT);
		hal.console->printf("altitudePIDOut: %f \n ", altitudePIDOut);
		*/
		
		
		// Compute climb rate PID
		PID ClimbRate; 
		float climbratePIDOut = ClimbRate.ComputePID(altitudePIDOut,pd_dot,0,Kp_climb,Ki_climb,Kd_climb,dt);
		/*
		hal.console->printf("pd_dot: %f \n ", pd_dot);
		hal.console->printf("climbrateError: %f \n ",altitudePIDOut - pd_dot);
		hal.console->printf("climbratePIDOut: %f \n ", climbratePIDOut);
		*/

		// Compute pitch PID
		PID Pitch;
		float pitchPIDOut = Pitch.ComputePID(climbratePIDOut, THETA, THETA_dot, Kp_pitch,Ki_pitch,Kd_pitch,dt);
		/*
		hal.console->printf("theta: %f \n ", THETA);
		hal.console->printf("pitchError: %f \n ",climbratePIDOut - THETA);
		hal.console->printf("pitchPIDOut: %f \n ", pitchPIDOut);
		*/
		
		// Compute speed PID
		PID Speed;
		float speedPIDOut = Pitch.ComputePID(des_speed, VX, u_dot, Kp_speed,Ki_speed,Kd_speed,dt);
		/*SpeedControll Information
		hal.console->printf("VX: %f \n ",VX);
		hal.console->printf("error: %f \n ",des_speed -VX);
		hal.console->printf("speedPIDOut: %f \n ",speedPIDOut);
		*/
		
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
		
		//take off condition
		if( abs(des_alt - ALT) < 1){
			TakeOff = true;
			return TakeOff;
		}else{
			return TakeOff;
		}
	}
}

AP_HAL_MAIN();
