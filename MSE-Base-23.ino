#include <MSE2202_Lib.h>

#include <MSE2202_Lib.h>

/*

 MSE 2202 MSEBot base code for Labs 3 and 4
 Language: Arduino
 Authors: Michael Naish and Eugen Porter
 Date: 16/01/17
 
 Rev 1 - Initial version
 Rev 2 - Update for MSEduino V0.2
 ...
 Rev 4 - revisit for MSEDuino V4.2 2023
 */

//  To program and use ESP32-S3
//   
//  File->Preferences:
//  Additional Boards Manager URLs: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
//
//
//  Tools->:
//  Board: "Adafruit Feather ESP32-S3 No PSRAM"
//  Upload Speed: "921600"
//  USB CDC On Boot: "Enabled"
//  USB Firmware MSC on Boot: "Disabled"
//  USB DFU On Bot: "Disabled"
//  Upload Mode:"UART0/Hardware CDC"
//  SPU Frequency: "240MHz (WiFi)"
//  Flash Mode: "QIO 80MHz"
//  Flash SIze: "4MB (32Mb)"
//  Partition Scheme: "Default 4MB with spiffs (1.2MB app/1.5MB SPIFFS)"
//  Core Debug Level: "Verbose"
//  PSRAM: 'Disabled"
//  Arduino Runs On: "Core 1"
//  Events Run On: "Core 1"
//
//  To program, press and hold the reset button then press and hold program button, release the reset button then release the program button 
//



#include <Adafruit_NeoPixel.h>
#include <MSE2202_lib.h>


// Uncomment keywords to enable debugging output


//#define DEBUG_DRIVE_SPEED 1
#define DEBUG_ENCODER_COUNT 1

//port pin constants

#define MODE_BUTTON  0      //GPIO0 pin 27 Push Button 1

#define LEFT_MOTOR_A 35    //GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B 36    //GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A 37    //GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B 38    //GPIO38 pin 31 (J38) Motor 2 B

#define SHOULDER_SERVO 41    //GPIO41 pin 34 (J41) Servo 1
#define CLAW_SERVO 42       //GPIO42 pin 35 (J42) Servo 2

#define BRDTST_POT_R1  1    //when DIP Switch S1-12 is ON, Analog AD0 (pin 39, GPIO1 is connected to Poteniometer R1

#define MOTOR_ENABLE_SWITCH 3     //DIP Switch S1-5 pulls Digital pin D3 to ground when ON; pin 15 GPIO3 (J3); When DIP Switch S1-5 is off can be used as analog AD1-2

#define ENCODER_RIGHT_A 11    //when DIP Switch S1-7 is ON, Right encoder A signal is connected to pin 19 GPIO11 (J11); When DIP Switch S1-7 is off can be used as analog AD2-0
#define ENCODER_RIGHT_B 12    //when DIP Switch S1-8 is ON, Right encoder B signal is connected to pin 20 GPIO12 (J12); When DIP Switch S1-8 is off can be used as analog AD2-1
#define ENCODER_RIGHT_DIR 13  //when DIP Switch S1-9 is ON, Right encoder Direction signal is connected to pin 21 GPIO13 (J13); When DIP Switch S1-9 is off can be used as analog AD2-2
#define ENCODER_RIGHT_SPD 14  //when DIP Switch S1-10 is ON, Right encoder Speed signal is connected to pin 22 GPIO14 (J14); When DIP Switch S1-10 is off can be used as analog AD2-3

#define ENCODER_LEFT_A 15    //when DIP Switch S1-1 is ON, Left encoder A signal is connected to pin 8 GPIO15 (J15); When DIP Switch S1-1 is off can be used as analog AD2-4
#define ENCODER_LEFT_B 16    //when DIP Switch S1-2 is ON, Left encoder B signal is connected to pin 9 GPIO16 (J16); When DIP Switch S1-2 is off can be used as analog AD2-5
#define ENCODER_LEFT_DIR 17  //when DIP Switch S1-3 is ON, Left encoder Direction signal is connected to pin 10 GPIO17 (J17); When DIP Switch S1-3 is off can be used as analog AD2-6
#define ENCODER_LEFT_SPD 18  //when DIP Switch S1-4 is ON, Left encoder Speed signal is connected to pin 11 GPIO18 (J18); When DIP Switch S1-4 is off can be used as analog AD2-7

#define STEPPER_DIR 39    //GPIO39 pin 32 (J39) STEPPER Motor direction pin
#define STEPPER_CLK 40    //GPIO40 pin 33 (J40) stepper motor clock pin

#define SMART_LED     21     //when DIP Switch S1-11 is ON, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT    1       //number of SMART LEDs in use
//constants



const int ci_Claw_Servo_Open = 1650;         // Experiment to determine appropriate value
const int ci_Claw_Servo_Closed = 1880;        //  "
const int ci_Shoulder_Servo_Retracted = 690;      //  "
const int ci_Shoulder_Servo_Extended = 1300;      //  "

const int ci_Display_Time = 100;  //100 ms


//variables
boolean bt_Motors_Enabled = true;

unsigned char ucDriveSpeed;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_2_Second_timer = 0;
unsigned long ul_Display_Time;


unsigned int uiMode_Debounce;


unsigned int  ui_Robot_Mode_Index = 0;
unsigned int  uiMode_PB_Debounce; 


boolean bt_3_S_Time_Up = false;
boolean bt_2_S_Time_Up = false;

unsigned char  ucDriveIndex;

// Declare our SK6812 SMART LED object:
Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);
// Argument 1 = Number of LEDs (pixels) in use
// Argument 2 = ESP32 pin number 
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
unsigned char LEDMaxBrightness = 50;
unsigned char LEDBrightnessIndex= 0;
unsigned char LEDBrightnessLevels[] = {5,15,30,45,60,75,90,105,120,135,150,165,180,195,210,225,240,255,240,225,210,195,180,165,150,135,120,105,90,75,60,45,30,15};
//Smart LED mode chart
            //0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
  SmartLEDs.Color(255,0,0),  // Red - Stop
  SmartLEDs.Color(0,255,0),  // Green - Run
  SmartLEDs.Color(0,0,255),  // Blue - Calibrate motors
  SmartLEDs.Color(50,50,50),  // colour? - empty case
  SmartLEDs.Color(50,50,50),  // colour? - empty case
  SmartLEDs.Color(25,25,25)  // colour? - empty case
};

unsigned long ulPreviousMicros;
unsigned long ulCurrentMicros;


Motion Bot = Motion();
Encoders driveEncoders = Encoders();
void IRAM_ATTR LeftSpd_EncoderISR()
{
	driveEncoders.LeftSpd_Encoder_ISR();
}
void IRAM_ATTR RightSpd_EncoderISR()
{
	driveEncoders.RightSpd_Encoder_ISR();
}

//Function Declarations
void Indicator(void);


void setup()
 {

  Serial.begin(9600);

  Bot.driveBegin("D1",LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A,RIGHT_MOTOR_B );
  driveEncoders.Begin(0, LeftSpd_EncoderISR, RightSpd_EncoderISR);

  SmartLEDs.begin(); // INITIALIZE SMART LEDs object (REQUIRED)
  SmartLEDs.clear();
  SmartLEDs.setPixelColor(0,SmartLEDs.Color(255,0,0));// Set pixel colors to 'off'
 // SmartLEDs.setBrightness(LEDMaxBrightness);
  SmartLEDs.show();   // Send the updated pixel colors to the hardware.

  

  // set up motor enable switch
  pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);

   

 uiMode_PB_Debounce = 0;

}

void loop()
{

  ulCurrentMicros = micros();
  if ((ulCurrentMicros - ulPreviousMicros) >= 1000)    //enter if 1ms has passed
  {
    ulPreviousMicros = ulCurrentMicros;
     
    ul_3_Second_timer = ul_3_Second_timer + 1;
    if(ul_3_Second_timer > 3000)
    {
      ul_3_Second_timer = 0;
      bt_3_S_Time_Up = true;
    }

    ul_2_Second_timer = ul_2_Second_timer + 1;
    if(ul_2_Second_timer > 2000)
    {
      ul_2_Second_timer = 0;
      bt_2_S_Time_Up = true;
    }
  
  //Mode push button debounce and toggle
    if(!digitalRead(MODE_BUTTON))
    {
      //button is pressed
      if(uiMode_PB_Debounce <= 100)   //~10.0 mS debounce
      {
          uiMode_PB_Debounce = uiMode_PB_Debounce + 1;
          if(uiMode_PB_Debounce > 100)
          {
              uiMode_PB_Debounce = 1000;
          }
      }
      if(uiMode_PB_Debounce >= 1000)
      {
        uiMode_PB_Debounce = 1000;
      }
     
    }
    else
    {
        //button is released
      if(uiMode_PB_Debounce <= 101)
      {
          uiMode_PB_Debounce = 0;
         
      }
      else
      {
         uiMode_PB_Debounce = uiMode_PB_Debounce + 1;
          if(uiMode_PB_Debounce >= 1256)
          {
            uiMode_PB_Debounce = 0;
            ui_Robot_Mode_Index++;  //// button-based mode selection
            ui_Robot_Mode_Index = ui_Robot_Mode_Index & 7;  //keep mode index between 0 and 7
            ul_3_Second_timer = 0;
            bt_3_S_Time_Up = false;
                    
          }
      }
    }
  
  
     // check if drive motors should be powered
    bt_Motors_Enabled = !digitalRead(MOTOR_ENABLE_SWITCH); // if sw1-5 ia on then motors are enabled

  // modes 
    // 0 = default after power up/reset
    // 1 = Press mode button once to enter. Run robot.
    // 2 = Press mode button twice to enter. //add your code to do something 
    // 3 = Press mode button three times to enter. //add your code to do something 
    // 4 = Press mode button four times to enter.  //add your code to do something 
    // 5 = Press mode button five times to enter. //add your code to do something 
    // 6 = Press mode button six times to enter.  //add your code to do something 
    switch(ui_Robot_Mode_Index)
    {
      case 0:    //Robot stopped
      {
       Bot.Stop("D1");
       ucDriveIndex = 0;
       driveEncoders.clearEncoder();
       bt_2_S_Time_Up = false;
      
        break;
      } 
  
      case 1:    //Robot Run after 3 seconds
      {
        
        if(bt_3_S_Time_Up)  //pauses for 3 sec before running case 1 code
        {
          
          ucDriveSpeed = map(analogRead(BRDTST_POT_R1),0,4096,150,255);
#ifdef DEBUG_DRIVE_SPEED 
          Serial.print(F("Drive Speed: Pot R1 = "));
          Serial.print(analogRead(BRDTST_POT_R1));
          Serial.print(F(",mapped = "));
          Serial.println(ucDriveSpeed);
#endif
#ifdef DEBUG_ENCODER_COUNT
          driveEncoders.getEncoderRawCount();
          driveEncoders.getEncoderRawSpeed();
          Serial.print(F("Left Encoder count = "));
          Serial.print(driveEncoders.lRawEncoderLeftCount);
          Serial.print(F("   Right Encoder count = "));
          Serial.print(driveEncoders.lRawEncoderRightCount);
          Serial.print(F("   Left Encoder speed = "));
          Serial.print(driveEncoders.lRawEncoderLeftSpeed);
          Serial.print(F("   Right Encoder speed = "));
          Serial.println(driveEncoders.lRawEncoderRightSpeed);
          
#endif

        
         
          
         
          if(bt_Motors_Enabled)
          {
            if(bt_2_S_Time_Up)
            {
              bt_2_S_Time_Up = false;
            
              switch(ucDriveIndex)
              {
                case 0: 
                {
                  Bot.Stop("D1");
                  ucDriveIndex = 1;
                  break;
                }
                case 1: 
                {
                  Bot.Forward("D1",ucDriveSpeed, ucDriveSpeed);
                  ucDriveIndex = 2;
                  break;
                }
                case 2: 
                {
                  Bot.Reverse("D1",ucDriveSpeed);
                  ucDriveIndex = 3;
                  break;
                }
                case 3: 
                {
                  Bot.Left("D1",ucDriveSpeed);
                  ucDriveIndex = 4;
                  break;
                }
                case 4: 
                {
                  Bot.Right("D1",ucDriveSpeed);
                  ucDriveIndex = 0;
                  break;
                }
              
              
              }
            }
          }
          else
          {  
             Bot.Stop("D1");
          }

 
          
        }
        break;
      } 
    case 2:    //add your code to do something 
      {
        ui_Robot_Mode_Index = 0; //  !!!!!!!   remove if using the case
        break;
      }   
      case 3://add your code to do something 
      {
        
        ui_Robot_Mode_Index = 0; //  !!!!!!!   remove if using the case
        break;
      }  
      case 4://add your code to do something 
      {
        
        ui_Robot_Mode_Index = 0; //  !!!!!!!   remove if using the case
        break;
      }  
      case 5://add your code to do something 
      {
        ui_Robot_Mode_Index = 0; //  !!!!!!!   remove if using the case
        break;
      }  
      case 6://add your code to do something 
      {
        ui_Robot_Mode_Index = 0; //  !!!!!!!   remove if using the case
        break;
      } 
    }


    ul_Display_Time++;

    if(ul_Display_Time > ci_Display_Time)
    {
      ul_Display_Time = 0;

      //Change brightness to indicate heart beat 
      LEDBrightnessIndex++;
      if(LEDBrightnessIndex > 35)
      {
        LEDBrightnessIndex = 0;
      }
      SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);
      
      Indicator();
    }
  } 
}


// set mode indicator LED state
void Indicator()
{
  SmartLEDs.setPixelColor(0,ui_Mode_Indicator[ui_Robot_Mode_Index]);// Set pixel colors to = mode 
  SmartLEDs.show();   // Send the updated pixel colors to the hardware.
}






