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


//TEST: Serial Communication
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(10, 11);
//int incomingByte = 0;   // for incoming serial data

//Dt
float dt=0.000000125;

//desired
float des_head = 45;
float des_speed = 20;

float des_roll = 0; 
float des_alt = -50;
float des_climb = 7;
float des_pitch = 90; 
 
 
// Gains for pidHeading
float Kp_head = 5;
float Ki_head = 0.001;
float Kd_head = 0.00005;

// Gains for pidRoll
float Kp_roll = 0.25;
float Ki_roll = 0.5;
float Kd_roll = 0.000001;

// Gains for pidAltitude
float Kp_alt = 5.5;//5;
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
        //MEIN CODE
		
                
        ///hal.console->printf("IN: %f \n ",dataSample.data.f[I_PHI]);
		
		//Reading inputs creating objects to avoid NaN
		
		
		float AX	= InAX.ReplaceNaN( dataSample.data.f[I_AX] );
        float AY	= InAY.ReplaceNaN( dataSample.data.f[I_AY] );
        float AZ	= InAZ.ReplaceNaN( dataSample.data.f[I_AZ] );
        
        float P		= InP.ReplaceNaN( dataSample.data.f[I_P] );
        float Q		= InQ.ReplaceNaN( dataSample.data.f[I_Q] );
        float R		= InR.ReplaceNaN( dataSample.data.f[I_R] );
		
		float THETA 	= InTHETA.ReplaceNaN( dataSample.data.f[I_THETA] );
        float PHI 		= InPHI.ReplaceNaN( dataSample.data.f[I_PHI] );
        float PSI 		= InPSI.ReplaceNaN( dataSample.data.f[I_PSI] );
        
        float ALT 		= InALT.ReplaceNaN( dataSample.data.f[I_ALT] );
        
        float VX 		= InVX.ReplaceNaN( dataSample.data.f[I_VX] );
        float VY		= InVY.ReplaceNaN( dataSample.data.f[I_VY] );
        float VZ		= InVZ.ReplaceNaN( dataSample.data.f[I_VZ] );         
              
        //Start Calculation         

        float pn_dot = (cos(THETA)*cos(PSI))*VX +(sin(PHI)*sin(THETA)*cos(PSI)-cos(PHI)*sin(PSI))*VY+(cos(PHI)*sin(THETA)*cos(PSI)+sin(PHI)*sin(PSI))*VZ;
        float pe_dot = (cos(THETA)*sin(PSI))*VX +(sin(PHI)*sin(THETA)*sin(PSI)+cos(PHI)*cos(PSI))*VY+(cos(PHI)*sin(THETA)*sin(PSI)-sin(PHI)*cos(PSI))*VZ;
        float pd_dot = -sin(THETA)*VX+sin(PHI)*cos(THETA)*VY+cos(PHI)*cos(THETA)*VZ;

        //CALCULATION OF u_dot, v_dot, w_dot

        float u_dot = (R*VY-Q*VZ)+AX;
        float v_dot = (P*VZ-R*VY)+AY;
        float w_dot = (Q*VX-P*VY)+AZ;

        //CALCULATION OF PHI_dot, THETA_dot, PSI_dot
        float PHI_dot = P+sin(PHI)*tan(THETA)*Q+cos(PHI)*tan(THETA)*R;
        float THETA_dot = cos(PHI)*Q-sin(PHI)*R;
        float PSI_dot = sin(PHI)/cos(THETA)*Q+cos(PHI)/cos(THETA)*R;
    
        // Use hal.console to receive a new target

        // Compute error in heading, ensuring it is in the range -Pi to Pi
		
        // Compute heading PID
        
        PID Heading;
        float headingPIDOut = Heading.ComputePID(des_head,PSI,PSI_dot,Kp_head,Ki_head,Kd_head,dt);
        
        // Constrain output of heading PID such that it is a valid target roll

        // Compute roll PID 
        PID Roll;
			//PID in Kaskade
		//float rollPIDOut = Roll.ComputePID(headingPIDOut, PHI, PHI_dot, Kp_roll,Ki_roll,Kd_roll,dt);
			//PID manuel desired Values
		float rollPIDOut = Roll.ComputePID(des_roll, PHI, PHI_dot, Kp_roll,Ki_roll,Kd_roll,dt);
		
		hal.console->printf("PHI: %f \n ", PHI);
        hal.console->printf("rollError: %f \n ",des_roll - PHI);
        hal.console->printf("rollPIDOut: %f \n ", rollPIDOut);
        
		
        // Compute altitude PID
        PID Altitude;   
			//PID in Kaskade
		//float altitudePIDOut = Altitude.ComputePID(75,-1*ALT,pd_dot,Kp_alt,Ki_alt,Kd_alt,dt);
			//PID manuel desired Values
		float altitudePIDOut = Altitude.ComputePID(des_alt,ALT,pd_dot,Kp_alt,Ki_alt,Kd_alt,dt);
		altitudePIDOut = -1*altitudePIDOut;
        /*
        hal.console->printf("ALT: %f \n ", ALT);
        hal.console->printf("altitudeError: %f \n ",des_alt - ALT);
        hal.console->printf("altitudePIDOut: %f \n ", altitudePIDOut);
        */
        // Compute climb rate PID
		PID ClimbRate; 
			//PID in Kaskade
        float climbratePIDOut = ClimbRate.ComputePID(altitudePIDOut,pd_dot,0,Kp_climb,Ki_climb,Kd_climb,dt);
		/*
		hal.console->printf("pd_dot: %f \n ", pd_dot);
        hal.console->printf("climbrateError: %f \n ",altitudePIDOut - pd_dot);
        hal.console->printf("climbratePIDOut: %f \n ", climbratePIDOut);
		*/
			//PID manuel desired Values
		//float climbratePIDOut = ClimbRate.ComputePID(des_climb,pd_dot,0,Kp_climb,Ki_climb,Kd_climb,dt);
			
        // Constrain output of climb rate PID such that it is a valid target pitch

        // Compute pitch PID
		PID Pitch;
			//PID in Kaskade
        float pitchPIDOut = Pitch.ComputePID(climbratePIDOut, THETA, THETA_dot, Kp_pitch,Ki_pitch,Kd_pitch,dt);
		/*
		hal.console->printf("theta: %f \n ", THETA);
        hal.console->printf("pitchError: %f \n ",climbratePIDOut - THETA);
        hal.console->printf("pitchPIDOut: %f \n ", pitchPIDOut);
		*/
		
		
			//PID manuel desired Values
        //float pitchPIDOut = Pitch.ComputePID(des_pitch, THETA, THETA_dot, Kp_pitch,Ki_pitch,Kd_pitch,dt);
        
        
        // Compute speed PID
		PID Speed;
	    float speedPIDOut = Pitch.ComputePID(des_speed, VX, u_dot, Kp_speed,Ki_speed,Kd_speed,dt);
        /*SpeedControll Information
        hal.console->printf("VX: %f \n ",VX);
        hal.console->printf("error: %f \n ",des_speed -VX);
        hal.console->printf("speedPIDOut: %f \n ",speedPIDOut);
        */
 
		/*	
		hal.console->printf("ALT: %f \n ",ALT);
		hal.console->printf("pd_DOT: %f \n ",pd_dot);
		hal.console->printf("THETA: %f \n ",THETA);
		//Printing values
		/*
		hal.console->printf("ALT: %f \n ",ALT);
        hal.console->printf("OUTalt: %f \n ",altitudePIDOut);
        
        hal.console->printf("VX: %f \n ",VX);
        hal.console->printf("OUTspeed: %f \n ",speedPIDOut);
        
        hal.console->printf("pd_DOT: %f \n ",pd_dot);
        hal.console->printf("OUTclimbrate: %f \n ",climbratePIDOut);
        
        */
        /*
        hal.console->printf("THETA: %f \n ",THETA);
        hal.console->printf("OUTpitch: %f \n ",pitchPIDOut);
       
        hal.console->printf("AX: %f \n ",AX);
        hal.console->printf("AY: %f \n ",AY);
        hal.console->printf("AZ: %f \n ",AZ);
       
        hal.console->printf("P: %f \n ",P);
        hal.console->printf("Q: %f \n ",Q);
        hal.console->printf("R: %f \n ",R);
              
        hal.console->printf("THETA: %f \n ",THETA);
        hal.console->printf("PHI: %f \n ",PHI);
        hal.console->printf("PSI: %f \n ",PSI);    
        
        hal.console->printf("ALT: %f \n ",ALT);   
        
        hal.console->printf("VX: %f \n ",VX);
        hal.console->printf("VY: %f \n ",VY);
        hal.console->printf("VZ: %f \n ",VZ);
		*/
		
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

AP_HAL_MAIN();
