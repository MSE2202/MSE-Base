/*
 Western Engineering MSE 2202 Library
 2023 E J Porter

 
  
 */

#include "MSE2202_Lib.h"

#define DEBUGPRINT 1
#define ACCELERATIONRATE 1;

Motion::Motion()
{
	ucLEDcLastUnUsedChannel = 0;  
}

unsigned char Motion::Get_LEDcChannel()
{
	unsigned char uc_NewChannel;
	
	uc_NewChannel = ucLEDcLastUnUsedChannel;
	ucLEDcLastUnUsedChannel++;
	if(ucLEDcLastUnUsedChannel >= LEDCMAXCHANNELS)
	{
		
		return(100);
	}
	return(uc_NewChannel);
	
}

void Motion::driveBegin(char cDriveID[2], int iLeftMotorPin1, int iLeftMotorPin2,int iRightMotorPin1, int iRightMotorPin2)
{
	char c_driveID; 
	
	if((cDriveID[0] == 'd') || (cDriveID[0] == 'D'))
	{
		if(cDriveID[1] == '1')
		{
			c_driveID = 1;
		}
		else if(cDriveID[1] == '2')
		{
			c_driveID = 2;
		}
		else
		{
			return(101);
		}
	
		//setup PWM for motors
		ucLEDcDriveChannels[(c_driveID * 4) - 4] = Get_LEDcChannel();
		ledcAttachPin(iLeftMotorPin1, ucLEDcDriveChannels[0]); // assign Motors pins to channels
		 // Initialize channels 
	  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
	  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
		ledcSetup(ucLEDcDriveChannels[(c_driveID * 4) - 4], 20000, 8); // 20mS PWM, 8-bit resolution
		
		ucLEDcDriveChannels[(c_driveID * 4) - 3] = Get_LEDcChannel();
		ledcAttachPin(iLeftMotorPin2, ucLEDcDriveChannels[1]);
		ledcSetup(ucLEDcDriveChannels[(c_driveID * 4) - 3], 20000, 8);
		
		ucLEDcDriveChannels[(c_driveID * 4) - 2] = Get_LEDcChannel();
		ledcAttachPin(iRightMotorPin1, ucLEDcDriveChannels[2]);
		ledcSetup(ucLEDcDriveChannels[(c_driveID * 4) - 2], 20000, 8);
		
		ucLEDcDriveChannels[(c_driveID * 4) - 1] = Get_LEDcChannel();
		ledcAttachPin(iRightMotorPin2, ucLEDcDriveChannels[3]);
		ledcSetup(ucLEDcDriveChannels[(c_driveID * 4) - 1], 20000, 8);
	}
	else
	{
		return(101);
	}
}

void Motion::MotorBegin(char cMotorID[2], int iMotorPin1, int iMotorPin2)
{
	char c_motorID; 
	
	if((cMotorID[0] == "m") || (cMotorID[0] == "M"))
	{
		if((cMotorID[1] >= "1") &&  (cMotorID[1] <= "4")
		{
			c_motorID = cMotorID[1] - 0x30;
		}
		else 
		{
			return(101);
		}
		//setup PWM for motors
		ucLEDcMotorChannels[(c_motorID * 2) - 2] = Get_LEDcChannel();
		ledcAttachPin(iMotorPin1, ucLEDcMotorChannels[0]); // assign Motors pins to channels
		 // Initialize channels 
	  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
	  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
		ledcSetup(ucLEDcMotorChannels[(c_motorID * 2) - 2], 20000, 8); // 20mS PWM, 8-bit resolution
		
		ucLEDcMotorChannels[(c_motorID * 2) - 1] = Get_LEDcChannel();
		ledcAttachPin(iMotorPin2, ucLEDcMotorChannels[1]);
		ledcSetup(ucLEDcMotorChannels[(c_motorID * 2) - 1], 20000, 8);
		

		
	}
	else
	{
		retrun(101);
	}
	
	
}

void Motion::ServoBegin(char cServoID[2], int iServoPin1)
{
	
	char c_servoID; 
	
	if((cServoID[0] == "m") || (cServoID[0] == "M"))
	{
		if((cServoID[1] >= "1") &&  (cServoID[1] <= "8")
		{
			c_servoID = cServoID[1] - 0x30;
		}
		else 
		{
			return(101);
		}
		//setup PWM for motors
		ucLEDcServoChannels[(c_servoID - 1] = Get_LEDcChannel();
		ledcAttachPin(iMotorPin1, ucLEDcServoChannels[0]); // assign Motors pins to channels
		 // Initialize channels 
	  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
	  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
		ledcSetup(ucLEDcServoChannels[(c_servoID * 2) - 2],  50,14);// channel 1, 50 Hz, 14-bit width
	}
	else
	{
		retrun(101);
	}
}

void Motion::Forward(char cID[2], unsigned int ucSpeed)
{ 
    char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == "1")
			{
				c_ID = 1;
			}
			else if(cID[1] == "2")
			{
				c_ID = 2;
			}
			else
			{
				return(101);
			}
			
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],uiSpeed);
			//Right Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],uiSpeed);
			break;
		}
		case 'm':
		case 'M':
		{
			if((cID[1] >= "1") &&  (cID[1] <= "4")
			{
				c_ID = cID[1] - 0x30;
			}
			else 
			{
				return(101);
			}
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],uiSpeed);
			break;
		}
		default:
		{
			return(101);
		}
	}
	
	
	
}



void Motion::Forward(char cID[2],unsigned int ucLeftSpeed,unsigned int ucRightSpeed)
{
	 char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == "1")
			{
				c_ID = 1;
			}
			else if(cID[1] == "2")
			{
				c_ID = 2;
			}
			else
			{
				return(101);
			}
			
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],uiLeftSpeed);
			//Right Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],uiRightSpeed);
			break;
		}
		
		default:
		{
			return(101);
		}
	}
		
}

void Motion::Reverse(char cID[2],unsigned int ucSpeed)
{
	char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == "1")
			{
				c_ID = 1;
			}
			else if(cID[1] == "2")
			{
				c_ID = 2;
			}
			else
			{
				return(101);
			}
			
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],uiSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],0);
			//Right Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],uiSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],0);
			break;
		}
		case 'm':
		case 'M':
		{
			if((cID[1] >= "1") &&  (cID[1] <= "4")
			{
				c_ID = cID[1] - 0x30;
			}
			else 
			{
				return(101);
			}
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],uiSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],0);
			break;
		}
		default:
		{
			return(101);
		}
	}
	
}

void Motion::Reverse(char cID[2],unsigned int ucLeftSpeed,unsigned int ucRightSpeed)
{
	 char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == "1")
			{
				c_ID = 1;
			}
			else if(cID[1] == "2")
			{
				c_ID = 2;
			}
			else
			{
				return(101);
			}
			
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],uiLeftSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],0);
			//Right Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],uiRightSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],0);
			break;
		}
		
		default:
		{
			return(101);
		}
	}
	
}
void Motion::Left(char cID[2],unsigned int ucSpeed)
{
	char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == "1")
			{
				c_ID = 1;
			}
			else if(cID[1] == "2")
			{
				c_ID = 2;
			}
			else
			{
				return(101);
			}
			
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],ucSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],0);
			//Right Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],ucSpeed);
			break;
		}
		
		default:
		{
			return(101);
		}
	}
	
}

void Motion::Left(char cID[2],unsigned int ucLeftSpeed,unsigned int ucRightSpeed)
{
	char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == "1")
			{
				c_ID = 1;
			}
			else if(cID[1] == "2")
			{
				c_ID = 2;
			}
			else
			{
				return(101);
			}
			
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],uiLeftSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],0);
			//Right Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],uiRightSpeed);
			break;
		}
		
		default:
		{
			return(101);
		}
	}
	
}
void Motion::Right(char cID[2],unsigned int ucSpeed)
{
	char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == "1")
			{
				c_ID = 1;
			}
			else if(cID[1] == "2")
			{
				c_ID = 2;
			}
			else
			{
				return(101);
			}
			
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],uiSpeed);
			//Right Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],uiSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],0);
			break;
		}
		
		default:
		{
			return(101);
		}
	}
	
}

void Motion::Right(char cID[2],unsigned int ucLeftSpeed,unsigned int ucRightSpeed)
{
	char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == "1")
			{
				c_ID = 1;
			}
			else if(cID[1] == "2")
			{
				c_ID = 2;
			}
			else
			{
				return(101);
			}
			
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],uiLeftSpeed);
			//Right Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],uiRightSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],0);
			break;
		}
		
		default:
		{
			return(101);
		}
	}
	
}
void Motion::Stop(char cID[2],)
{
	 char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == "1")
			{
				c_ID = 1;
			}
			else if(cID[1] == "2")
			{
				c_ID = 2;
			}
			else
			{
				return(101);
			}
			
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],0);
			//Right Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],0);
			break;
		}
		case 'm':
		case 'M':
		{
			if((cID[1] >= "1") &&  (cID[1] <= "4")
			{
				c_ID = cID[1] - 0x30;
			}
			else 
			{
				return(101);
			}
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],0);
			break;
		}
		default:
		{
			return(101);
		}
	}
	
}


void Motion::end()
{
	for(unsigned char ucLEDcIndex = 0; ucLEDcIndex < ucLEDcLastUnUsedChannel; ucLEDcIndex++)
	{
		ledcWrite(ucLEDcDriveChannels[ucLEDcIndex],0);
	}
	
	
	ucLEDcLastUnUsedChannel = 0; 
}



