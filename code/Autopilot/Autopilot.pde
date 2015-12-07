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

//TEST: Serial Communication
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(10, 11);
//int incomingByte = 0;   // for incoming serial data

// Gains for pidHeading


// Gains for pidRoll


// Gains for pidAltitude


// Gains for pidClimbRate


// Gains for pidPitch


// Gains for pidSpeed


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

    //mySerial.begin(57600);
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
        
        //Calculation of pn_dot, pe_dot, pd_dot
        // printout of u,v,w
        /*
        hal.console->printf("VX: %f \n ",dataSample.data.raw[I_VX]);
        hal.console->printf("VY: %f \n",dataSample.data.raw[I_VY]);
        hal.console->printf("VZ: %f \n\n",dataSample.data.raw[I_VZ]);
        */
                
        ///hal.console->printf("IN: %f \n ",dataSample.data.f[I_PHI]);
		
		//creating objects to avoid NaN
		AvoidNaN In_THETA, In_PHI, In_PSI, In_VX, In_VY, In_VZ , In_P, In_R, In_Q, IN_AX, IN_AY, IN_AZ;
				
        //Calculation 
        float THETA 	= In_THEAT.ReplaceNaN( dataSample.data.f[I_THETA] );
        float PHI 		= In_PHI.ReplaceNaN( dataSample.data.f[I_PHI] );
        float PSI 		= In_PSI.ReplaceNaN( dataSample.data.f[I_PSI] );
        
        float VX 		= In_VX.ReplaceNaN( dataSample.data.f[I_VX] );
        float VY		= In_VY.ReplaceNaN( dataSample.data.f[I_VY] );
        float VZ		= In_VZ.ReplaceNaN( dataSample.data.f[I_VZ] );

        float pn_dot = (cos(THETA)*cos(PSI))*VX +(sin(PHI)*sin(THETA)*cos(PSI)-cos(PHI)*sin(PSI))*VY+(cos(PHI)*sin(THETA)*cos(PSI)+sin(PHI)*sin(PSI))*VZ;
        float pe_dot = (cos(THETA)*sin(PSI))*VX +(sin(PHI)*sin(THETA)*sin(PSI)+cos(PHI)*cos(PSI))*VY+(cos(PHI)*sin(THETA)*sin(PSI)-sin(PHI)*cos(PSI))*VZ;
        float pd_dot = -sin(THETA)*VX+sin(PHI)*cos(THETA)*VY+cos(PHI)*cos(THETA)*VZ;

        //CALCULATION OF u_dot, v_dot, w_dot
        float P		= In_P.ReplaceNaN( dataSample.data.f[I_P] );
        float Q		= In_Q.ReplaceNaN( dataSample.data.f[I_Q] );
        float R		= In_R.ReplaceNaN( dataSample.data.f[I_R] );

        float AX	= In_AX.ReplaceNaN( dataSample.data.f[I_AX] );
        float AY	= In_AY.ReplaceNaN( dataSample.data.f[I_AY] );
        float AZ	= In_AZ.ReplaceNaN( dataSample.data.f[I_AZ] );

        float u_dot = (R*VY-Q*VZ)+AX;
        float v_dot = (P*VZ-R*VY)+AY;
        float w_dot = (Q*VX-P*VY)+AZ;

        //CALCULATION OF PHI_dot, THETA_dot, PSI_dot
        float PHI_dot = P+sin(PHI)*tan(THETA)*Q+cos(PHI)*tan(THETA)*R;
        float THETA_dot = cos(PHI)*Q-sin(PHI)*R;
        float PSI_dot = sin(PHI)/cos(THETA)*Q+cos(PHI)/cos(THETA)*R;



        //Befehle kann man Online finden
        //hal.console->printf(); //in Google suchen
    
        // Use hal.console to receive a new target

        // Compute error in heading, ensuring it is in the range -Pi to Pi
		
        // Compute heading PID
        PID Heading;
        float headingPIDOut=Heading.ComputePID(0,PSI,PSI_dot,1,0,0,0);
        
        // Constrain output of heading PID such that it is a valid target roll

        // Compute roll PID 
        PID Roll;
        float rollPIDOut = Roll.ComputePID(headingPIDOut, PHI, PHI_dot, 1, 0, 0, 0);
    
        // Compute altitude PID
        PID Altitude;
        float ALT = dataSample.data.f[I_ALT];
        float altitudePIDOut = Altitude.ComputePID(100,ALT,pd_dot,2,0,0,0);
        hal.console->printf("ALT: %f \n ",ALT);
        hal.console->printf("OUTalt: %f \n ",altitudePIDOut);
       
        // Compute climb rate PID
		PID ClimbRate; 
        float climbratePIDOut = ClimbRate.ComputePID(20,pd_dot,0,1,0,0,0);
        hal.console->printf("pd_DOT: %f \n ",pd_dot);
        hal.console->printf("OUTclimbrate: %f \n ",climbratePIDOut);
        // Constrain output of climb rate PID such that it is a valid target pitch

        // Compute pitch PID
		PID Pitch;
        float pitchPIDOut = Pitch.ComputePID(climbratePIDOut, THETA, THETA_dot, 1, 0, 0, 0);
        hal.console->printf("THETA: %f \n ",THETA);
        hal.console->printf("OUTptch: %f \n ",pitchPIDOut);
        // Compute speed PID
		PID Speed;
		float speedPIDOut=Speed.ComputePID(1,VX,u_dot,1,0,0,0);

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
