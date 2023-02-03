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
			Serial.printf("Incorrect ID Designator number %s\n", cDriveID);
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
		Serial.printf("Incorrect ID Designator %s\n", cDriveID);
	}
}

void Motion::motorBegin(char cMotorID[2], int iMotorPin1, int iMotorPin2)
{
	char c_motorID; 
	
	if((cMotorID[0] == 'M') || (cMotorID[0] == 'M'))
	{
		if((cMotorID[1] >= '1') &&  (cMotorID[1] <= '4'))
		{
			c_motorID = cMotorID[1] - 0x30;
		}
		else 
		{
			Serial.printf("Incorrect ID Designator number %s\n", cMotorID);
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
		Serial.printf("Incorrect ID Designator %s\n", cMotorID);
	}
	
	
}

void Motion::servoBegin(char cServoID[2], int iServoPin1)
{
	
	char c_servoID; 
	
	if((cServoID[0] == 'M') || (cServoID[0] == 'M'))
	{
		if((cServoID[1] >= '1') &&  (cServoID[1] <= '8'))
		{
			c_servoID = cServoID[1] - 0x30;
		}
		else 
		{
			Serial.printf("Incorrect ID Designator number %s\n", cServoID);
		}
		//setup PWM for motors
		ucLEDcServoChannels[(c_servoID - 1)] = Get_LEDcChannel();
		ledcAttachPin(iServoPin1, ucLEDcServoChannels[0]); // assign Motors pins to channels
		 // Initialize channels 
	  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
	  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
		ledcSetup(ucLEDcServoChannels[(c_servoID * 2) - 2],  50,14);// channel 1, 50 Hz, 14-bit width
	}
	else
	{
		Serial.printf("Incorrect ID Designator number %s\n", cServoID);
	}
}

void Motion::Forward(char cID[2], unsigned char ucSpeed)
{ 
    char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == '1')
			{
				c_ID = 1;
			}
			else if(cID[1] == '2')
			{
				c_ID = 2;
			}
			else
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],ucSpeed);
			//Right Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],ucSpeed);
			break;
		}
		case 'm':
		case 'M':
		{
			if((cID[1] >= '1') &&  (cID[1] <= '4'))
			{
				c_ID = cID[1] - 0x30;
			}
			else 
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],ucSpeed);
			break;
		}
		default:
		{
			Serial.printf("Incorrect ID Designator %s\n", cID);
		}
	}
	
	
	
}



void Motion::Forward(char cID[2],unsigned char ucLeftSpeed,unsigned char ucRightSpeed)
{
	 char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == '1')
			{
				c_ID = 1;
			}
			else if(cID[1] == '2')
			{
				c_ID = 2;
			}
			else
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],ucLeftSpeed);
			//Right Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],ucRightSpeed);
			break;
		}
		
		default:
		{
			Serial.printf("Incorrect ID Designator %s\n", cID);
		}
	}
		
}

void Motion::Reverse(char cID[2],unsigned char ucSpeed)
{
	char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == '1')
			{
				c_ID = 1;
			}
			else if(cID[1] == '2')
			{
				c_ID = 2;
			}
			else
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],ucSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],0);
			//Right Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],ucSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],0);
			break;
		}
		case 'm':
		case 'M':
		{
			if((cID[1] >= '1') &&  (cID[1] <= '4'))
			{
				c_ID = cID[1] - 0x30;
			}
			else 
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],ucSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],0);
			break;
		}
		default:
		{
			Serial.printf("Incorrect ID Designator %s\n", cID);
		}
	}
	
}

void Motion::Reverse(char cID[2],unsigned char ucLeftSpeed,unsigned char ucRightSpeed)
{
	 char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == '1')
			{
				c_ID = 1;
			}
			else if(cID[1] == '2')
			{
				c_ID = 2;
			}
			else
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],ucLeftSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],0);
			//Right Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],ucRightSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],0);
			break;
		}
		
		default:
		{
			Serial.printf("Incorrect ID Designator %s\n", cID);
		}
	}
	
}
void Motion::Left(char cID[2],unsigned char ucSpeed)
{
	char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == '1')
			{
				c_ID = 1;
			}
			else if(cID[1] == '2')
			{
				c_ID = 2;
			}
			else
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
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
			Serial.printf("Incorrect ID Designator %s\n", cID);
		}
	}
	
}

void Motion::Left(char cID[2],unsigned char ucLeftSpeed,unsigned char ucRightSpeed)
{
	char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == '1')
			{
				c_ID = 1;
			}
			else if(cID[1] == '2')
			{
				c_ID = 2;
			}
			else
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],ucLeftSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],0);
			//Right Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],ucRightSpeed);
			break;
		}
		
		default:
		{
			Serial.printf("Incorrect ID Designator %s\n", cID);
		}
	}
	
}
void Motion::Right(char cID[2],unsigned char ucSpeed)
{
	char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == '1')
			{
				c_ID = 1;
			}
			else if(cID[1] == '2')
			{
				c_ID = 2;
			}
			else
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],ucSpeed);
			//Right Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],ucSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],0);
			break;
		}
		
		default:
		{
			Serial.printf("Incorrect ID Designator %s\n", cID);
		}
	}
	
}

void Motion::Right(char cID[2],unsigned char ucLeftSpeed,unsigned char ucRightSpeed)
{
	char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == '1')
			{
				c_ID = 1;
			}
			else if(cID[1] == '2')
			{
				c_ID = 2;
			}
			else
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],ucLeftSpeed);
			//Right Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],ucRightSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],0);
			break;
		}
		
		default:
		{
			Serial.printf("Incorrect ID Designator %s\n", cID);
		}
	}
	
}
void Motion::Stop(char cID[2])
{
	 char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == '1')
			{
				c_ID = 1;
			}
			else if(cID[1] == '2')
			{
				c_ID = 2;
			}
			else
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
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
			if((cID[1] >= '1') &&  (cID[1] <= '4'))
			{
				c_ID = cID[1] - 0x30;
			}
			else 
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],0);
			break;
		}
		default:
		{
			Serial.printf("Incorrect ID Designator %s\n", cID);
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

//----------------------------------------------------------------------------------------------------------
//Encoders
//Encoders* Encoders::anchorLeftSpd = 0;
 /*  #include "esp_log.h"
  
 
void IRAM_ATTR Encoders::LeftSpd_Encoder_ISR()
{

	portENTER_CRITICAL_ISR(&(this->mux));

	
   volatile static int32_t ENC_vsi32LastTime;
   volatile static int32_t ENC_vsi32ThisTime;
  
 
   // if the last interrupts data wasn't collected, count the miss
  if(this->ENC_btLeftEncoderSpdDataFlag)
  {
    this->ENC_vuiLeftEncoderSpdMissed += 1;
    
  }
 
  //how much time elapsed since last interrupt
  
  asm volatile("esync; rsr %0,ccount":"=a" (ENC_vsi32ThisTime )); // @ 240mHz clock each tick is ~4nS 
  this->ENC_vl32LeftEncoderARawTime = ENC_vsi32ThisTime - ENC_vsi32LastTime;
  ENC_vsi32LastTime = ENC_vsi32ThisTime;
  this->ENC_btLeftEncoderSpdDataFlag = true;

  //if Read Left direction pin if low count up otherwise wheel is going backwards count down
  //odometer reading
  if(digitalRead(17))  // MSE-Dunio port pin
  {
    this->ENC_vlLeftOdometer += 1;
  }
  else
  {
    this->ENC_vlLeftOdometer -= 1;
  }


	portEXIT_CRITICAL_ISR(&(this->mux));

}




Encoders::Encoders()
{
	
}


void Encoders::Begin(unsigned char ucEncoderType, void (*ISR_callback)(void))
{
	switch(ucEncoderType)
	{
		case 0:
		{
			//these pins are hardwired in the MSE-Duino V4.2 board
			pinMode(17, INPUT_PULLUP);   //#define ENCODER_LEFT_DIR 17  //when DIP Switch S1-3 is ON, Left encoder Direction signal is connected to pin 10 GPIO17 (J17); 
			pinMode(18, INPUT_PULLUP);   //#define ENCODER_LEFT_SPD 18  //when DIP Switch S1-4 is ON, Left encoder Speed signal is connected to pin 11 GPIO18 (J18);
			pinMode(13, INPUT_PULLUP);  //#define ENCODER_RIGHT_DIR 13  //when DIP Switch S1-9 is ON, Right encoder Direction signal is connected to pin 21 GPIO13 (J13); 
			pinMode(14, INPUT_PULLUP);  //#define ENCODER_RIGHT_SPD 14  //when DIP Switch S1-10 is ON, Right encoder Speed signal is connected to pin 22 GPIO14 (J14); 

			
			// enable GPIO interrupt on change
			//attachInterrupt(17, ENC_isrLeftDir, CHANGE);
			attachInterrupt(18, ISR_callback, CHANGE);
			//attachInterrupt(13, ENC_isrRightDir, CHANGE);
	//		attachInterrupt(14, ENC_isrRightSPD, CHANGE);
			break;
		}
		case 1:
		{
			//these pins are hardwired in the MSE-Duino V4.2 board
			pinMode(15, INPUT_PULLUP);     //#define ENCODER_LEFT_A 15    //when DIP Switch S1-1 is ON, Left encoder A signal is connected to pin 8 GPIO15 (J15); 
			pinMode(16, INPUT_PULLUP);     //#define ENCODER_LEFT_B 16    //when DIP Switch S1-2 is ON, Left encoder B signal is connected to pin 9 GPIO16 (J16);
			pinMode(11, INPUT_PULLUP);     //#define ENCODER_RIGHT_A 11    //when DIP Switch S1-7 is ON, Right encoder A signal is connected to pin 19 GPIO11 (J11);
			pinMode(12, INPUT_PULLUP);     //#define ENCODER_RIGHT_B 12    //when DIP Switch S1-8 is ON, Right encoder B signal is connected to pin 20 GPIO12 (J12); 
			
			// enable GPIO interrupt on change
	//		attachInterrupt(15, ENC_isrLeftA, CHANGE);
	//		attachInterrupt(16, ENC_isrLeftB, CHANGE);
	//		attachInterrupt(11, ENC_isrRightA, CHANGE);
	//		attachInterrupt(12, ENC_isrRightB, CHANGE);
			break;
		}
		case 2:
		{
			//these pins are hardwired in the MSE-Duino V4.2 board
			pinMode(15, INPUT_PULLUP);     //#define ENCODER_LEFT_A 15    //when DIP Switch S1-1 is ON, Left encoder A signal is connected to pin 8 GPIO15 (J15); 
			pinMode(16, INPUT_PULLUP);     //#define ENCODER_LEFT_B 16    //when DIP Switch S1-2 is ON, Left encoder B signal is connected to pin 9 GPIO16 (J16);
			pinMode(11, INPUT_PULLUP);     //#define ENCODER_RIGHT_A 11    //when DIP Switch S1-7 is ON, Right encoder A signal is connected to pin 19 GPIO11 (J11);
			pinMode(12, INPUT_PULLUP);     //#define ENCODER_RIGHT_B 12    //when DIP Switch S1-8 is ON, Right encoder B signal is connected to pin 20 GPIO12 (J12); 
			
			pinMode(17, INPUT_PULLUP);   //#define ENCODER_LEFT_DIR 17  //when DIP Switch S1-3 is ON, Left encoder Direction signal is connected to pin 10 GPIO17 (J17); 
			pinMode(18, INPUT_PULLUP);   //#define ENCODER_LEFT_SPD 18  //when DIP Switch S1-4 is ON, Left encoder Speed signal is connected to pin 11 GPIO18 (J18);
			pinMode(13, INPUT_PULLUP);  //#define ENCODER_RIGHT_DIR 13  //when DIP Switch S1-9 is ON, Right encoder Direction signal is connected to pin 21 GPIO13 (J13); 
			pinMode(14, INPUT_PULLUP);  //#define ENCODER_RIGHT_SPD 14  //when DIP Switch S1-10 is ON, Right encoder Speed signal is connected to pin 22 GPIO14 (J14); 
			
			// enable GPIO interrupt on change
	//		attachInterrupt(15, ENC_isrLeftA, CHANGE);
	//		attachInterrupt(16, ENC_isrLeftB, CHANGE);
	//		attachInterrupt(11, ENC_isrRightA, CHANGE);
	//		attachInterrupt(12, ENC_isrRightB, CHANGE);
			
			// enable GPIO interrupt on change
			//attachInterrupt(17, ENC_isrLeftDir, CHANGE);
	//		attachInterrupt(18, ENC_isrLeftSpd, CHANGE);
			//attachInterrupt(13, ENC_isrRightDir, CHANGE);
	//		attachInterrupt(14, ENC_isrRightSPD, CHANGE);
			break;
		}
		
		default:
		{
			Serial.printf("Incorrect ucEncoderType type %i\n", ucEncoderType);
		}
	}
}





 */



//Encoder interrupt service routines - entered every change in in encoder pin H-> L and L ->H

/* void Encoders::ENC_isrLSpd()
{
   volatile static int32_t ENC_vsi32LastTime;
   volatile static int32_t ENC_vsi32ThisTime;
  
 
   // if the last interrupts data wasn't collected, count the miss
  if(anchorLeftSpd->ENC_btLeftEncoderSpdDataFlag)
  {
    anchorLeftSpd->ENC_vuiLeftEncoderSpdMissed += 1;
    
  }
 
  //how much time elapsed since last interrupt
  
  asm volatile("esync; rsr %0,ccount":"=a" (ENC_vsi32ThisTime )); // @ 240mHz clock each tick is ~4nS 
  anchorLeftSpd->ENC_vl32LeftEncoderARawTime = ENC_vsi32ThisTime - ENC_vsi32LastTime;
  ENC_vsi32LastTime = ENC_vsi32ThisTime;
  anchorLeftSpd->ENC_btLeftEncoderSpdDataFlag = true;

  //if Read Left direction pin if low count up otherwise wheel is going backwards count down
  //odometer reading
  if(digitalRead(17))
  {
    anchorLeftSpd->ENC_vlLeftOdometer += 1;
  }
  else
  {
    anchorLeftSpd->ENC_vlLeftOdometer -= 1;
  }

  
}
static void Encoders::ENC_isrLeftSpd()
{
	anchorLeftSpd->ENC_isrLSpd();
} */

/* static void Encoders::ENC_isrLeftSpd()
{
}

static void Encoders::ENC_isrRightSPD()
{
}

static void Encoders::ENC_isrLeftA()
{
}

static void Encoders::ENC_isrLeftB()
{
}

static void Encoders::ENC_isrRightA()
{
}

static void Encoders::ENC_isrRightB()
{
}
 */
// void Encoders::end()
// {
	
// }
