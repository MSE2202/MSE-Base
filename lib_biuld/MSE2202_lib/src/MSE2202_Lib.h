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
	    
	    void driveBegin(int iMotorPin1, int iMotorPin2,int iRightMotorPin1, int iRightMotorPin2);
		
		void Forward(unsigned int uiSpeed);
		void Forward(unsigned int uiLeftSpeed,unsigned int uiRightSpeed );
		void Reverse(unsigned int uiSpeed);
		void Left(unsigned int uiSpeed);
		void Right(unsigned int uiSpeed);
		void Stop();
		void end();

	  
private:
        unsigned char ucLEDcLastUnUsedChannel;	
		//unsigned char LEDCMAXCHANNELS = 8;
		unsigned char ucLEDcDriveChannels[4];
		
		
		unsigned char ucMotion_Direction;
		unsigned char ucMotion_Speed;

		//const unsigned char cui8StartingSpeed = 140;

		//unsigned int ui8LeftWorkingSpeed = cui8StartingSpeed;
		//unsigned int ui8RightWorkingSpeed = cui8StartingSpeed;

		//unsigned char ucMotorState = 0;
		
		//double dManualSpeed;
		//double dForwardSpeed;
		//double dReverseSpeed;
		//double dLeftSpeed;
		//double dRightSpeed;
		unsigned char Get_LEDcChannel();



};




#endif /* MSE2202_LIB_H */
