/*

 MSE 2202 MSEBot base code for Labs 3 and 4
 Language: Arduino
 Authors: Eugen Porter and Michael Naish
 
 Rev 1 - Initial version 2016
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
//  To program, press and hold the reset button then press and hold program button, release the reset button then 
//  release the program button 
//

#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>

// Uncomment keywords to enable debugging output
#define DEBUG_DRIVE_SPEED 1
#define DEBUG_ENCODER_COUNT 1

// Port pin constants

#define MODE_BUTTON         0   // GPIO0  pin 27 for Push Button 1
                                   
#define LEFT_MOTOR_A        35  // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B        36  // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A       37  // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B       38  // GPIO38 pin 31 (J38) Motor 2 B
                                   
#define SHOULDER_SERVO      41  // GPIO41 pin 34 (J41) Servo 1
#define CLAW_SERVO          42  // GPIO42 pin 35 (J42) Servo 2
                                   
#define ENCODER_LEFT_A      15  // When DIP Switch S1-1 is on, Left encoder A signal is connected to pin 8 GPIO15 (J15)
                                // When DIP Switch S1-1 is off, J15 can be used as analog AD2-4
#define ENCODER_LEFT_B      16  // When DIP Switch S1-2 is on, Left encoder B signal is connected to pin 9 GPIO16 (J16)
                                // When DIP Switch S1-2 is off, J16 can be used as analog AD2-5
#define ENCODER_LEFT_DIR    17  // When DIP Switch S1-3 is on, Left encoder Direction signal is connected to pin 10 GPIO17 (J17)
                                // When DIP Switch S1-3 is off, J17 can be used as analog AD2-6
#define ENCODER_LEFT_SPD    18  // When DIP Switch S1-4 is on, Left encoder Speed signal is connected to pin 11 GPIO18 (J18)
                                // When DIP Switch S1-4 is off, J18 can be used as analog AD2-7
                                   
#define MOTOR_ENABLE_SWITCH 3   // DIP Switch S1-5 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
                                // When DIP Switch S1-5 is off, J3 can be used as analog AD1-2
                                   
#define ENCODER_RIGHT_A     11  // When DIP Switch S1-7 is on, Right encoder A signal is connected to pin 19 GPIO11 (J11)
                                // When DIP Switch S1-7 is off, J11 can be used as analog AD2-0
#define ENCODER_RIGHT_B     12  // When DIP Switch S1-8 is on, Right encoder B signal is connected to pin 20 GPIO12 (J12)
                                // When DIP Switch S1-8 is off, J12 can be used as analog AD2-1
#define ENCODER_RIGHT_DIR   13  // When DIP Switch S1-9 is on, Right encoder Direction signal is connected to pin 21 GPIO13 (J13)
                                // When DIP Switch S1-9 is off, J13 can be used as analog AD2-2
#define ENCODER_RIGHT_SPD   14  // When DIP Switch S1-10 is on, Right encoder Speed signal is connected to pin 22 GPIO14 (J14)
                                // When DIP Switch S1-10 is off, J14 can be used as analog AD2-3
                                   
#define BRDTST_POT_R1       1   // When DIP Switch S1-12 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1

#define STEPPER_DIR         39  // GPIO39 pin 32 (J39) STEPPER Motor direction pin
#define STEPPER_CLK         40  // GPIO40 pin 33 (J40) stepper motor clock pin
                                   
#define SMART_LED           21  // When DIP Switch S1-11 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1   // Number of SMART LEDs in use




#define LEFT  1
#define RIGHT 0


// Constants

const int ci_Display_Update = 100;                                            // Update interval for Smart LED in milliseconds

//=====================================================================================================================
//
// IMPORTANT: The constants in this section need to be set to appropriate values for your robot. 
// You will have to experiment to determine appropriate values.

const int ci_Claw_Servo_Open = 1650;                                          // Value for open position of claw
const int ci_Claw_Servo_Closed = 1880;                                        // Value for closed position of claw
const int ci_Shoulder_Servo_Retracted = 690;                                  // Value for shoulder of arm fully retracted
const int ci_Shoulder_Servo_Extended = 1300;                                  // Value for shoulder of arm fully extended

//
//=====================================================================================================================

// Variables

boolean bt_Motors_Enabled = true;                                             // Motors enabled flag
boolean bt_3_S_Time_Up = false;                                               // 3 second timer elapsed flag
boolean bt_2_S_Time_Up = false;                                               // 2 second timer elapsed flag
boolean bt_Direction;													      //stepper motor direction
boolean bt_Stepper_Step;
boolean bt_DoOnce;




unsigned char uc_Drive_Speed;                                                 // Motor drive speed (0-255)
unsigned char uc_Drive_Index;                                                 // State index for run mode (1)

unsigned int ui_DeadZone = 50;

int i_MaxStepsFromCenter = 1000;
int i_StepperCenter = i_MaxStepsFromCenter;
int i_StepCounter = 0;     
int i_StepperSetpoint;
unsigned int ui_PotArmSetpoint;
unsigned int ui_PotClawSetpoint;
unsigned int ui_PotShoulderSetpoint;


unsigned int  ui_Mode_PB_Debounce;                                            // Pushbutton debounce timer count



unsigned long ul_3_Second_timer = 0;                                          // 3 second timer count in milliseconds
unsigned long ul_2_Second_timer = 0;                                          // 2 second timer count in milliseconds
unsigned long ul_Display_Time;                                                // Heartbeat LED update timer
unsigned long ul_Previous_Micros;                                             // Last microsecond count
unsigned long ul_Current_Micros;                                              // Current microsecond count



// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {5,15,30,45,60,75,90,105,120,135,150,165,180,195,210,225,240,255,
                                       240,225,210,195,180,165,150,135,120,105,90,75,60,45,30,15};

unsigned int  ui_Robot_Mode_Index = 0;                                        // Robot operational state                              
unsigned int  ui_Mode_Indicator[6] = {                                        // Colours for different modes
  SmartLEDs.Color(255,0,0),                                                   //   Red - Stop
  SmartLEDs.Color(0,255,0),                                                   //   Green - Run
  SmartLEDs.Color(0,0,255),                                                   //   Blue - Calibrate motors
  SmartLEDs.Color(255,255,0),                                                 //   Yellow - empty case
  SmartLEDs.Color(0,255,255),                                                 //   Cyan - empty case
  SmartLEDs.Color(255,0,255)                                                  //   Magenta - empty case
};                                                                            

// Motor and encoder objects (classes defined in MSE2202_Lib)
Motion Bot = Motion();                                                        // Instance of Motion for motor control
Encoders driveEncoders = Encoders();                                          // Instance of Encoders for encoder data

// Interrupt Service Routines
void IRAM_ATTR LeftSpd_EncoderISR()                                           // ISR to update left encoder count
{
   driveEncoders.LeftSpd_Encoder_ISR();
}

void IRAM_ATTR RightSpd_EncoderISR()                                          // ISR to update right encoder count
{
	driveEncoders.RightSpd_Encoder_ISR();
}

// Function Declarations
void Indicator();                                                             // For mode/heartbeat on Smart LED

void setup()
{
   Serial.begin(9600);

   // Set up motors and encoders
   Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B); // Set up motors as Drive 1
   Bot.servoBegin("S1",CLAW_SERVO);   
   Bot.servoBegin("S2",SHOULDER_SERVO);

   driveEncoders.Begin(0, LeftSpd_EncoderISR, RightSpd_EncoderISR);           // Set up encoders

   // Set up SmartLED
   SmartLEDs.begin();                                                         // Initialize smart LEDs object (REQUIRED)
   SmartLEDs.clear();                                                         // Clear pixel
   SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,0));                         // Set pixel colors to 'off'
   SmartLEDs.show();                                                          // Send the updated pixel colors to the hardware

   // Set up mode pushbutton
   pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);                                // set up motor enable switch with internal pullup
   ui_Mode_PB_Debounce = 0;                                                   // Reset debounce timer count
   
   //Stepper motor 
  pinMode(STEPPER_DIR, OUTPUT);
  pinMode(STEPPER_CLK, OUTPUT);  
}

void loop()
{
   ul_Current_Micros = micros();                                              // Get current time in microseconds
   if ((ul_Current_Micros - ul_Previous_Micros) >= 1000)                      // Enter when 1 ms has elapsed
   {
      ul_Previous_Micros = ul_Current_Micros;                                 // Record current time in microseconds

      // 3 second timer, counts 3000 milliseconds
      ul_3_Second_timer = ul_3_Second_timer + 1;                              // Increment 3 second timer count
      if(ul_3_Second_timer > 3000)                                            // If 3 seconds have elapsed
      {
         ul_3_Second_timer = 0;                                               // Reset 3 second timer count
         bt_3_S_Time_Up = true;                                               // Indicate that 3 seconds have elapsed
      }
   
      // 2 second timer, counts 2000 milliseconds
      ul_2_Second_timer = ul_2_Second_timer + 1;                              // Increment 2 second timer count
      if(ul_2_Second_timer > 2000)                                            // If 2 seconds have elapsed
      {
         ul_2_Second_timer = 0;                                               // Reset 2 second timer count
         bt_2_S_Time_Up = true;                                               // Indicate that 2 seconds have elapsed
      }
   
      // Mode pushbutton debounce and toggle
      if(!digitalRead(MODE_BUTTON))                                           // If pushbutton GPIO goes LOW (nominal push)
      {
         // Start debounce
         if(ui_Mode_PB_Debounce <= 25)                                        // 25 millisecond debounce time
         {
            ui_Mode_PB_Debounce = ui_Mode_PB_Debounce + 1;                    // Increment debounce timer count
            if(ui_Mode_PB_Debounce > 25)                                      // If held for at least 25 mS
            {
               ui_Mode_PB_Debounce = 1000;                                    // Change debounce timer count to 1 second
            }
         }
         if(ui_Mode_PB_Debounce >= 1000)                                      // Maintain 1 second timer count until release
         {
            ui_Mode_PB_Debounce = 1000;
         }
      }
      else                                                                    // Pushbutton GPIO goes HIGH (nominal release)
      {
         if(ui_Mode_PB_Debounce <= 26)                                        // If release occurs within debounce interval
         {
            ui_Mode_PB_Debounce = 0;                                          // Reset debounce timer count
         }
         else
         {
            ui_Mode_PB_Debounce = ui_Mode_PB_Debounce + 1;                    // Increment debounce timer count
            if(ui_Mode_PB_Debounce >= 1025)                                   // If pushbutton was released for 25 mS
            {
               ui_Mode_PB_Debounce = 0;                                       // Reset debounce timer count
               ui_Robot_Mode_Index++;                                         // Switch to next mode
               ui_Robot_Mode_Index = ui_Robot_Mode_Index & 7;                 // Keep mode index between 0 and 7
               ul_3_Second_timer = 0;                                         // Reset 3 second timer count
               bt_3_S_Time_Up = false;                                        // Reset 3 second timer
			   bt_DoOnce = 1; 
            }
         }
      }
  
      // check if drive motors should be powered
      bt_Motors_Enabled = !digitalRead(MOTOR_ENABLE_SWITCH);                  // If SW1-5 is on (low signal), then motors are enabled

      // modes 
      // 0 = Default after power up/reset. Robot is stopped.
      // 1 = Press mode button once to enter. Run robot.
      // 2 = Press mode button twice to enter. //add your code to do something 
      // 3 = Press mode button three times to enter. //add your code to do something 
      // 4 = Press mode button four times to enter.  //add your code to do something 
      // 5 = Press mode button five times to enter. //add your code to do something 
      // 6 = Press mode button six times to enter.  //add your code to do something 
      switch(ui_Robot_Mode_Index)
      {
         case 0: // Robot stopped
         {
            Bot.Stop("D1");                                                   // Stop Drive 1
            uc_Drive_Index = 0;                                               // Reset drive index
            driveEncoders.clearEncoder();                                     // Clear encoder counts
            bt_2_S_Time_Up = false;                                           // Reset 2 second timer
            break;
         }  
      
         case 1: // Run robot
         {
            if(bt_3_S_Time_Up)                                                // Pause for 3 sec before running case 1 code
            {
               // Read pot to update drive motor speed
               uc_Drive_Speed = map(analogRead(BRDTST_POT_R1), 0, 4096, 150, 255);
#ifdef DEBUG_DRIVE_SPEED 
               Serial.print(F("Drive Speed: Pot R1 = "));
               Serial.print(analogRead(BRDTST_POT_R1));
               Serial.print(F(", mapped = "));
               Serial.println(uc_Drive_Speed);
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
               if(bt_Motors_Enabled)                                          // Run motors only if enabled
               {
                  if(bt_2_S_Time_Up)                                          // Update drive state after 2 seconds
                  {
                     bt_2_S_Time_Up = false;                                  // Reset 2 second timer
                     
                     switch(uc_Drive_Index)                                   // Cycle through drive states
                     {
                        case 0: // Stop
                        {
                           Bot.Stop("D1");                                    // Drive ID
                           uc_Drive_Index = 1;                                // Next state: drive forward
                           break;
                        }
                        case 1: // Drive forward
                        {
                           Bot.Forward("D1", uc_Drive_Speed, uc_Drive_Speed); // Drive ID, Left speed, Right speed
                           uc_Drive_Index = 2;                                // Next state: drive backward
                           break;
                        }
                        case 2: // Drive backward
                        {
                           Bot.Reverse("D1", uc_Drive_Speed);                 // Drive ID, Speed (same for both)                 
                           uc_Drive_Index = 3;                                // Next state: turn left
                           break;
                        }
                        case 3: // Turn left (counterclockwise)
                        {
                           Bot.Left("D1", uc_Drive_Speed);                    // Drive ID, Speed (same for both)
                           uc_Drive_Index = 4;                                // Next state: turn right
                           break;
                        }
                        case 4: // Turn right (clockwise)
                        {
                           Bot.Right("D1", uc_Drive_Speed);                   // Drive ID, Speed (same for both)
                           uc_Drive_Index = 0;                                // Next state: stop
                           break;
                        }
                     }
                  }
               }
               else                                                           // Stop when motors are disabled
               {  
                  Bot.Stop("D1");
               }
            }
            break;
         } 
          case 2:    //Stepper motor test with pot. 
      {
        if(bt_DoOnce)
        {
          Serial.println(F("Start this test with Arm in center position and pot at its center."));
          Serial.println(F("The center of the pot rotation will be considered to be Arm center."));
          i_StepperSetpoint = i_StepperCenter;
          i_StepCounter = 0; 
          bt_Stepper_Step = 0;
          bt_DoOnce = 0;
          Bot.Stop("D1");
        }

        ui_PotArmSetpoint = map(analogRead(BRDTST_POT_R1),0,4096,0,(i_MaxStepsFromCenter*2));
        Serial.print(F("Arm Stepper: Pot R1 = "));
        Serial.print(analogRead(BRDTST_POT_R1));
        Serial.print(F(",mapped = "));
        Serial.print(ui_PotArmSetpoint);
        Serial.print(F(", Direction = "));
        if(bt_Direction == LEFT)
        {
          Serial.print(F(" Left"));          
        } 
        else 
        {
          Serial.print(F(" Right"));          
        }                       
        Serial.print(F(", StepCounter = "));
        Serial.print(i_StepCounter);
        Serial.print(F(", StepperSetpoint = "));
        Serial.println(i_StepperSetpoint);
        

        if(bt_3_S_Time_Up)  //pauses for 3 sec before running case 2 code
        {
          if(ui_PotArmSetpoint > (i_StepperSetpoint + ui_DeadZone))
          {
            bt_Direction = LEFT;
            i_StepCounter = i_StepperSetpoint - ui_PotArmSetpoint;
          }
           if(ui_PotArmSetpoint < (i_StepperSetpoint - ui_DeadZone))
          {
            bt_Direction = RIGHT;
            i_StepCounter = i_StepperSetpoint - ui_PotArmSetpoint;
          }
          if (i_StepCounter)
          {
            
            digitalWrite(STEPPER_DIR, bt_Direction);
            if(bt_Stepper_Step)
            {
              bt_Stepper_Step = 0;
              digitalWrite(STEPPER_CLK, LOW);  
              i_StepCounter--;
              if(bt_Direction == LEFT)
              {
                if(i_StepCounter == 0)
                {
                  i_StepperSetpoint = ui_PotArmSetpoint;
                }
                else
                {
                  i_StepperSetpoint++;                      
                }
                
              }
              else
              {
                if(i_StepCounter == 0)
                {
                  i_StepperSetpoint = ui_PotArmSetpoint;
                }
                else
                {
                  i_StepperSetpoint--;
                }
                
              }
            }
            else
            {
              bt_Stepper_Step = 1;
              digitalWrite(STEPPER_CLK, HIGH);  
            }
          }
        }

        break;
      }      
         case 3: //Test claw via pot
         {
          ui_PotClawSetpoint = map(analogRead(BRDTST_POT_R1),0,4096,ci_Claw_Servo_Open,ci_Claw_Servo_Closed);
          Serial.print(F("Claw : Pot R1 = "));
          Serial.print(analogRead(BRDTST_POT_R1));
          Serial.print(F(",mapped = "));
          Serial.println(ui_PotClawSetpoint);
          Bot.ToPosition("S1",ui_PotClawSetpoint);

            break;
         }  
         case 4: //test shoulder Servo with pot
         {
          ui_PotShoulderSetpoint = map(analogRead(BRDTST_POT_R1),0,4096,ci_Shoulder_Servo_Retracted,ci_Shoulder_Servo_Extended);
          Serial.print(F("Shoulder : Pot R1 = "));
          Serial.print(analogRead(BRDTST_POT_R1));
          Serial.print(F(",mapped = "));
          Serial.println(ui_PotShoulderSetpoint);
          Bot.ToPosition("S2",ui_PotShoulderSetpoint);

            break;
         }  
         case 5: //add your code to do something 
         {
            ui_Robot_Mode_Index = 0; //  !!!!!!!  remove if using the case
            break;
         }  
         case 6: //add your code to do something 
         {
            ui_Robot_Mode_Index = 0; //  !!!!!!!  remove if using the case
            break;
         } 
      }

      // Update brightness of heartbeat display on SmartLED
      ul_Display_Time++;                                                      // Count milliseconds
      if(ul_Display_Time > ci_Display_Update)                                 // When display update period has passed
      {
         ul_Display_Time = 0;                                                 // Reset display counter
         LEDBrightnessIndex++;                                                // Shift to next brightness level
         if(LEDBrightnessIndex > sizeof(LEDBrightnessLevels))                 // If all defined levels have been used
         {
            LEDBrightnessIndex = 0;                                           // Reset to starting brightness
         }
         SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);    // Set brightness of heartbeat LED
         Indicator();                                                         // Update LED
      }
   }
}   

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator()
{
  SmartLEDs.setPixelColor(0, ui_Mode_Indicator[ui_Robot_Mode_Index]);         // Set pixel colors to = mode 
  SmartLEDs.show();                                                           // Send the updated pixel colors to the hardware
}






