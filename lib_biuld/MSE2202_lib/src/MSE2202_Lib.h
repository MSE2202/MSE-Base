/*
 Western Engineering MSE 2202 Library
 2023 E J Porter

 
  
 */



 #ifndef MSE2202_LIB_H
  #define MSE2202_LIB_H 1

#include <Arduino.h>
#include "esp32-hal.h"
#include "esp32-hal-ledc.h"

//************  Motion *********************************

#define LEDCMAXCHANNELS 8

class Motion
{
public:
	    Motion();
	    ~Motion(){ end(); }
/*
  driveBegin with set up two motors as a drive base. DriveID = D1 or D2 ( only two Drives can be begun, this will limit number of servoBegin and motorBegin. Since there is only 8 channels total
 and driveBegin uses 4 channels each.
*/
	    void driveBegin(char cDriveID[2], int iLeftMotorPin1, int iLeftMotorPin2,int iRightMotorPin1, int iRightMotorPin2); 
/*
  Set up one motor to be controlled.  cMotorID = M1 to M4 ( limited number of motors and will be lessened if servos or drives are used. Since there is only 8 channels total
 and MotorBegin uses 2 channels each.
*/		
		void MotorBegin(char cMotorID[2], int iMotorPin1, int iMotorPin2);
/*
  Set up one servo to be controlled.  cServoID = S1 to S8 ( limited number of servos and will be lessened if motors or drives are used. Since there is only 8 channels total
 and servoBegin uses 1 channel each.
*/			
		void ServoBegin(char cServoID[2], int iServoPin1);
/*
  Will run the motor(s) "Forward" at speed in ucSpeed ( 0 to 255),Can be both Drive or Motor ID = M1 to M4 or D1, D2
*/			
		void Forward(char cID[2], unsigned int ucSpeed);
/*
  Will run the motor(s) "Forward" at ucLeftSpeed( 0 to 255) for left motor and ucRightSpeed ( 0 to 255) for right motor.  Only for Drive ID = D1, D2
*/		
		void Forward(char cID[2],unsigned char ucLeftSpeed, unsigned int ucRightSpeed );
/*
  Will run the motor(s) "Reverse" at speed in ucSpeed ( 0 to 255),Can be both Drive or Motor ID = M1 to M4 or D1, D2
*/		
		void Reverse(char cID[2],unsigned int ucSpeed);
/*
  Will run the motor(s) "Reverse" at ucLeftSpeed( 0 to 255) for left motor and ucRightSpeed ( 0 to 255) for right motor.  Only for Drive ID = D1, D2
*/		
		void Reverse(char cID[2],unsigned int ucLeftSpeed,unsigned int ucRightSpeed );
/*
  Will run the motor(s) "Left" (Right motor forward and  Left motor in reverse, zero point turn) at speed in ucSpeed ( 0 to 255), Only for Drive ID = D1, D2
*/		
		void Left(char cID[2],unsigned int ucSpeed);
/*
  Will run the motor(s) "Left" (Right motor forward at ucRightSpeed speed and  Left motor in reverse at ucLeftSpeed speed , will do sweep turn), Only for Drive ID = D1, D2
*/		
		void Left(char cID[2],unsigned int ucLeftSpeed,unsigned int ucRightSpeed );
/*
  Will run the motor(s) "Right" (Right motor reverse and  Left motor in forward, zero point turn) at speed in ucSpeed ( 0 to 255), Only for Drive ID = D1, D2
*/		
		void Right(char cID[2],unsigned int ucSpeed);
/*
  Will run the motor(s) "Right" (Right motor reverse at ucRightSpeed speed and  Left motor in forward at ucLeftSpeed speed , will do sweep turn), Only for Drive ID = D1, D2
*/		
		void Right(char cID[2],unsigned int ucLeftSpeed,unsigned int ucRightSpeed );
/*
  Will run the Stop motor(s) 55), Can be Drive,  Motor or Servo ID = M1 to M4 or D1, D2
*/		
		void Stop(char cID[2]);
		void end();

	  
private:
        unsigned char ucLEDcLastUnUsedChannel;	
		//unsigned char LEDCMAXCHANNELS = 8;
		unsigned char ucLEDcDriveChannels[8];
		unsigned char ucLEDcMotorChannels[8];
		unsigned char ucLEDcServoChannels[8];
		unsigned char Get_LEDcChannel();



};




#endif /* MSE2202_LIB_H */
