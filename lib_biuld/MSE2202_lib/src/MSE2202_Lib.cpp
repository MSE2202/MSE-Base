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

void Motion::driveBegin(const int ciLeftMotorPin1, const int ciLeftMotorPin2,const int ciRightMotorPin1, const int ciRightMotorPin2)
{
	//setup PWM for motors
	ucLEDcDriveChannels[0] = Get_LEDcChannel();
    ledcAttachPin(ciLeftMotorPin1, ucLEDcDriveChannels[0]); // assign Motors pins to channels
	 // Initialize channels 
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
    ledcSetup(ucLEDcDriveChannels[0], 20000, 8); // 20mS PWM, 8-bit resolution
	
	ucLEDcDriveChannels[1] = Get_LEDcChannel();
    ledcAttachPin(ciLeftMotorPin2, ucLEDcDriveChannels[1]);
	ledcSetup(ucLEDcDriveChannels[1], 20000, 8);
	
	ucLEDcDriveChannels[2] = Get_LEDcChannel();
    ledcAttachPin(ciRightMotorPin1, ucLEDcDriveChannels[2]);
	ledcSetup(ucLEDcDriveChannels[2], 20000, 8);
	
	ucLEDcDriveChannels[3] = Get_LEDcChannel();
    ledcAttachPin(ciRightMotorPin2, ucLEDcDriveChannels[3]);
	ledcSetup(ucLEDcDriveChannels[3], 20000, 8);
}

void Motion::Forward(unsigned int uiSpeed)
{
	//Left Motor
	ledcWrite(ucLEDcDriveChannels[0],0);
	ledcWrite(ucLEDcDriveChannels[1],uiSpeed);
	//Right Motor
    ledcWrite(ucLEDcDriveChannels[2],0);
	ledcWrite(ucLEDcDriveChannels[3],uiSpeed);
}

void Motion::Forward(unsigned int uiLeftSpeed,unsigned int uiRightSpeed)
{
	//Left Motor
	ledcWrite(ucLEDcDriveChannels[0],0);
	ledcWrite(ucLEDcDriveChannels[1],uiLeftSpeed);
	//Right Motor
    ledcWrite(ucLEDcDriveChannels[2],0);
	ledcWrite(ucLEDcDriveChannels[3],uiRightSpeed);
}

void Motion::Reverse(unsigned int uiSpeed)
{
	//Left Motor
	ledcWrite(ucLEDcDriveChannels[0],uiSpeed);
	ledcWrite(ucLEDcDriveChannels[1],0);
	//Right Motor
    ledcWrite(ucLEDcDriveChannels[2],uiSpeed);
	ledcWrite(ucLEDcDriveChannels[3],0);
}

void Motion::Reverse(unsigned int uiLeftSpeed,unsigned int uiRightSpeed)
{
	//Left Motor
	ledcWrite(ucLEDcDriveChannels[0],uiLeftSpeed);
	ledcWrite(ucLEDcDriveChannels[1],0);
	//Right Motor
    ledcWrite(ucLEDcDriveChannels[2],uiRightSpeed);
	ledcWrite(ucLEDcDriveChannels[3],0);
}
void Motion::Left(unsigned int uiSpeed)
{
	//Left Motor
	ledcWrite(ucLEDcDriveChannels[0],uiSpeed);
	ledcWrite(ucLEDcDriveChannels[1],0);
	//Right Motor
    ledcWrite(ucLEDcDriveChannels[2],0);
	ledcWrite(ucLEDcDriveChannels[3],uiSpeed);
}

void Motion::Left(unsigned int uiLeftSpeed,unsigned int uiRightSpeed)
{
	//Left Motor
	ledcWrite(ucLEDcDriveChannels[0],uiLeftSpeed);
	ledcWrite(ucLEDcDriveChannels[1],0);
	//Right Motor
    ledcWrite(ucLEDcDriveChannels[2],0);
	ledcWrite(ucLEDcDriveChannels[3],uiRightSpeed);
}
void Motion::Right(unsigned int uiSpeed)
{
	//Left Motor
	ledcWrite(ucLEDcDriveChannels[0],0);
	ledcWrite(ucLEDcDriveChannels[1],uiSpeed);
	//Right Motor
    ledcWrite(ucLEDcDriveChannels[2],uiSpeed);
	ledcWrite(ucLEDcDriveChannels[3],0);
}

void Motion::Right(unsigned int uiLeftSpeed,unsigned int uiRightSpeed)
{
	//Left Motor
	ledcWrite(ucLEDcDriveChannels[0],0);
	ledcWrite(ucLEDcDriveChannels[1],uiLeftSpeed);
	//Right Motor
    ledcWrite(ucLEDcDriveChannels[2],uiRightSpeed);
	ledcWrite(ucLEDcDriveChannels[3],0);
}
void Motion::Stop()
{
	//Left Motor
	ledcWrite(ucLEDcDriveChannels[0],0);
	ledcWrite(ucLEDcDriveChannels[1],0);
	//Right Motor
    ledcWrite(ucLEDcDriveChannels[2],0);
	ledcWrite(ucLEDcDriveChannels[3],0);
}


void Motion::end()
{
	ledcWrite(ucLEDcDriveChannels[0],0);
	ledcWrite(ucLEDcDriveChannels[1],0);
	ledcWrite(ucLEDcDriveChannels[2],0);
	ledcWrite(ucLEDcDriveChannels[3],0);
	
	ucLEDcLastUnUsedChannel = 0; 
}


/* void setupMotion (void)
{
	
  dManualSpeed = 0;
  dForwardSpeed = 250;  // max 255; min ~150 before motor stall
  dReverseSpeed = 250;
  dLeftSpeed = 170;
  dRightSpeed = 170;
  
  //setup PWM for motors
  ledcAttachPin(ciMotorLeftA, 1); // assign Motors pins to channels
  ledcAttachPin(ciMotorLeftB, 2);
  ledcAttachPin(ciMotorRightA, 3);
  ledcAttachPin(ciMotorRightB, 4);

  // Initialize channels 
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(1, 20000, 8); // 20mS PWM, 8-bit resolution
  ledcSetup(2, 20000, 8);
  ledcSetup(3, 20000, 8);
  ledcSetup(4, 20000, 8);
 	
   ucMotion_Direction = 0;
   ucMotion_Speed = 0;
}


void ResetSpeeds()
{
  ui8LeftWorkingSpeed = cui8StartingSpeed;
  ui8RightWorkingSpeed = cui8StartingSpeed;

  
}

void MoveTo(uint8_t ui8Direction, uint8_t ui8LeftSpeed, uint8_t ui8RightSpeed)
{
    int  iPrintOnce;
      
   
     switch(ui8Direction)
      {
      
      
        //forward
        case 1:
        {
            
          if(ui8LeftWorkingSpeed >= ui8LeftSpeed)
          {
            ui8LeftWorkingSpeed = ui8LeftSpeed;
          }
          else
          {
          ui8LeftWorkingSpeed = ui8LeftWorkingSpeed + ACCELERATIONRATE;
          }
          if(ui8RightWorkingSpeed >= ui8RightSpeed)
          {
            ui8RightWorkingSpeed = ui8RightSpeed;
          }
          else
          {
            ui8RightWorkingSpeed = ui8RightWorkingSpeed + ACCELERATIONRATE;
          }
          
          ledcWrite(2,0);
          ledcWrite(1,ui8LeftWorkingSpeed);
          ledcWrite(4,0);
          ledcWrite(3,ui8RightWorkingSpeed);
          
          break;
        }
        //Left
        case 2:
        {
          if(ui8LeftWorkingSpeed >= ui8LeftSpeed)
          {
            ui8LeftWorkingSpeed = ui8LeftSpeed;
          }
          else
          {
          ui8LeftWorkingSpeed = ui8LeftWorkingSpeed + ACCELERATIONRATE;
          }
          if(ui8RightWorkingSpeed >= ui8RightSpeed)
          {
            ui8RightWorkingSpeed = ui8RightSpeed;
          }
          else
          {
            ui8RightWorkingSpeed = ui8RightWorkingSpeed + ACCELERATIONRATE;
          }
         
          ledcWrite(1,0);
          ledcWrite(2,ui8LeftWorkingSpeed);
          ledcWrite(4,0);
          ledcWrite(3,ui8RightWorkingSpeed);
        
          break;
        }
        //Right
        case 3:
        {
          if(ui8LeftWorkingSpeed >= ui8LeftSpeed)
          {
            ui8LeftWorkingSpeed = ui8LeftSpeed;
          }
          else
          {
          ui8LeftWorkingSpeed = ui8LeftWorkingSpeed + ACCELERATIONRATE;
          }
          if(ui8RightWorkingSpeed >= ui8RightSpeed)
          {
            ui8RightWorkingSpeed = ui8RightSpeed;
          }
          else
          {
            ui8RightWorkingSpeed = ui8RightWorkingSpeed + ACCELERATIONRATE;
          }
         
          ledcWrite(2,0);
          ledcWrite(1,ui8LeftWorkingSpeed);
          ledcWrite(3,0);
          ledcWrite(4,ui8RightWorkingSpeed);
       
          break;
        }
        //Reverse
        case 4:
        {
             
          if(ui8LeftWorkingSpeed >= ui8LeftSpeed)
          {
            ui8LeftWorkingSpeed = ui8LeftSpeed;
          }
          else
          {
          ui8LeftWorkingSpeed = ui8LeftWorkingSpeed + ACCELERATIONRATE;
          }
          if(ui8RightWorkingSpeed >= ui8RightSpeed)
          {
            ui8RightWorkingSpeed = ui8RightSpeed;
          }
          else
          {
            ui8RightWorkingSpeed = ui8RightWorkingSpeed + ACCELERATIONRATE;
          }
         
          ledcWrite(1,0);
          ledcWrite(2,ui8LeftWorkingSpeed);
          ledcWrite(3,0);
          ledcWrite(4,ui8RightWorkingSpeed);
       
          break;
        }
     
        
      }
 }

void move(uint8_t ui8Speed)
{
    int  iPrintOnce;

     switch(ucMotorState)
      {
        //Stop, coast mode
        case 0:
        {
          //if 0 is put in both INs motors will coast stop 
          ledcWrite(2,0);
          ledcWrite(1,0);
          ledcWrite(4,0);
          ledcWrite(3,0);
        //ucWorkingButtonState = 9;
      #ifdef DEBUGPRINT  
          if(iPrintOnce != 0)
           {
            iPrintOnce = 0;
            Serial.print(F("stop-coasting"));
            Serial.println(ui8Speed);
          }
      #endif    
          break;
        }
      
        //Forward
        case 1:
        {
          //ui8speed = dForwardSpeed;
          ledcWrite(2,0);
          ledcWrite(1,ui8Speed);
          ledcWrite(4,0);
          ledcWrite(3,ui8Speed);
          //ucWorkingButtonState = 9;
        #ifdef DEBUGPRINT  
          if(iPrintOnce != 1)
           {
            iPrintOnce = 1;
            Serial.print(F("Forward "));
            Serial.println(ui8Speed);
           }
         #endif   
          break;
        }
        //Left
        case 2:
        {
          ui8Speed = dLeftSpeed;
          ledcWrite(2,0);
          ledcWrite(1,ui8Speed);
          ledcWrite(3,0);
          ledcWrite(4,ui8Speed);
         //ucWorkingButtonState = 9;
         #ifdef DEBUGPRINT  
          if(iPrintOnce != 3)
           {
            iPrintOnce = 3;
            Serial.print(F("Left "));
            Serial.println(ui8Speed);
           }
         #endif 
          break;
        }
        //Right
        case 3:
        {
          ui8Speed = dRightSpeed;
          ledcWrite(1,0);
          ledcWrite(2,ui8Speed);
          ledcWrite(4,0);
          ledcWrite(3,ui8Speed);
          // ucWorkingButtonState = 9;
          #ifdef DEBUGPRINT  
          if(iPrintOnce != 4)
           {
            iPrintOnce = 4;
            Serial.print(F("Right "));
            Serial.println(ui8Speed);
           }
         #endif   
          break;
        }
        //Reverse
        case 4:
        {
          // ui8speed = dReverseSpeed;
          ledcWrite(1,0);
          ledcWrite(2,ui8Speed);
          ledcWrite(3,0);
          ledcWrite(4,ui8Speed);
         // ucWorkingButtonState = 9;
         #ifdef DEBUGPRINT  
          if(iPrintOnce != 2)
           {
            iPrintOnce = 2;
            Serial.print(F("Reverse "));
            Serial.println(ui8Speed);
           }
         #endif  
          break;
        }
        //Stop  braking mode
        case 5:
        {
          //if 255 is put in both INs brakes will be applied 
          ledcWrite(2,255);
          ledcWrite(1,255);
          ledcWrite(4,255);
          ledcWrite(3,255);
        //ucWorkingButtonState = 9;
      #ifdef DEBUGPRINT  
          if(iPrintOnce != 0)
           {
            iPrintOnce = 0;
            Serial.print(F("stop-braking"));
            Serial.println(ui8Speed);
          }
      #endif    
          break;
        }
      }
} */


