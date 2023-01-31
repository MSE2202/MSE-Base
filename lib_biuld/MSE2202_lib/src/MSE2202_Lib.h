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
		
		void Forward(char cID[2], unsigned int uiSpeed);
		void Forward(unsigned int uiLeftSpeed,unsigned int uiRightSpeed );
		void Reverse(unsigned int uiSpeed);
		void Reverse(unsigned int uiLeftSpeed,unsigned int uiRightSpeed );
		void Left(unsigned int uiSpeed);
		void Left(unsigned int uiLeftSpeed,unsigned int uiRightSpeed );
		void Right(unsigned int uiSpeed);
		void Right(unsigned int uiLeftSpeed,unsigned int uiRightSpeed );
		void Stop();
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
