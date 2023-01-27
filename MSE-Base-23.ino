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




//#include <Servo.h>
//#include <EEPROM.h>
//#include <uSTimer2.h>
#include <Adafruit_NeoPixel.h>
//#include <Wire.h>
//#include <I2CEncoder.h>

//Servo servo_RightMotor;
//Servo servo_LeftMotor;
//Servo servo_ArmMotor;    
//Servo servo_GripMotor;



// Uncomment keywords to enable debugging output
//#define DEBUG_MOTORS
//#define DEBUG_ENCODERS
//#define DEBUG_MOTOR_CALIBRATION


//port pin constants

#define MODE_BUTTON  0;      //GPIO0 pin 27 Push Button 1

#define LEFT_MOTOR_A 35    //GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B 36    //GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A 37    //GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B 38    //GPIO38 pin 31 (J38) Motor 2 B

#define SHOULDER_SERVO 41;    //GPIO41 pin 34 (J41) Servo 1
#define CLAW_SERVO 42;       //GPIO42 pin 35 (J42) Servo 2

#define MOTOR_ENABLE_SWITCH 3;     //DIP Switch S1-5 pulls Digital pin D3 to ground when ON; pin 15 GPIO3 (J3); When DIP Switch S1-5 is off can be used as analog AD1-2

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

//constants

// EEPROM addresses

const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;


const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;


const int ci_Claw_Servo_Open = 1650;         // Experiment to determine appropriate value
const int ci_Claw_Servo_Closed = 1880;        //  "
const int ci_Shoulder_Servo_Retracted = 690;      //  "
const int ci_Shoulder_Servo_Extended = 1300;      //  "

const int ci_Display_Time = 50;  //50 ms

const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;

//variables
boolean bt_Motors_Enabled = true;

byte b_LowByte;
byte b_HighByte;


unsigned int ui_Motors_Speed = 1900;        // Default run speed
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
long l_Left_Motor_Position;
long l_Right_Motor_Position;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

unsigned int uiMode_Debounce;

unsigned int ui_Cal_Count;
unsigned int ui_Cal_Cycle;


unsigned int  ui_Robot_Mode_Index = 0;
unsigned int  uiMode_PB_Debounce; 


boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;

// Declare our SK6812 SMART LED object:
Adafruit_NeoPixel SmartLEDs(BRDTST_LED_COUNT, BRDTST_SMART_LED, NEO_RGB + NEO_KHZ800);
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
unsigned char LEDMaxBrightness[] = {5,10,15,20,25,30,35,40,45,50,45,40,35,30,25,20,15,5};
//Smart LED mode chart
            //0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
  SmartLEDs.Color(5,0,0),  // Red - Stop
  SmartLEDs.Color(0,5,0),  // Green - Run
  SmartLEDs.Color(0,0,5))  // Blue - Calibrate motors
  
};

unsigned long ulPreviousMicrosCore;
unsigned long ulCurrentMicrosCore;

void setup()
 {

  Serial.begin(9600);

  SmartLEDs.begin(); // INITIALIZE SMART LEDs object (REQUIRED)
  SmartLEDs.clear();
  SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,0));// Set pixel colors to 'off'
  SmartLEDs.setBrightness(LEDMaxBrightness);
  SmartLEDs.show();   // Send the updated pixel colors to the hardware.

    // set up drive motors
  pinMode(LEFT_MOTOR_A, OUTPUT);
  pinMode(LEFT_MOTOR_B, OUTPUT);
  pinMode(RIGHT_MOTOR_A, OUTPUT);
  pinMode(RIGHT_MOTOR_B, OUTPUT);

  // set up arm motors
   ledcSetup(1, 50,14);// channel 1, 50 Hz, 14-bit width
   ledcAttachPin(SHOULDER_SERVO, 1); // assign servo pins to channels
   ledcSetup(2, 50,14);// channel 2, 50 Hz, 14-bit width
   ledcAttachPin(CLAW_SERVO, 2); // assign servo pins to channels
  

  // set up motor enable switch
  pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);

  // set up encoders. Must be initialized in order that they are chained together, 
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  pinMode(BRDTST_ENCODER_LEFT_DIR, INPUT_PULLUP);
  pinMode(BRDTST_ENCODER_LEFT_SPD, INPUT_PULLUP);

  pinMode(BRDTST_ENCODER_RIGHT_DIR, INPUT_PULLUP);
  pinMode(BRDTST_ENCODER_RIGHT_SPD, INPUT_PULLUP);

  //encoder_LeftMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  //encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  //encoder_RightMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);  
  //encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward

  // read saved values from EEPROM
 // ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
 // b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
 // b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
 // ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);

 uiMode_PB_Debounce = 0;

}

void loop()
{

  ulCurrentMicros = micros();
  if ((ulCurrentMicros - ulPreviousMicros) >= 1000)    //enter if 1ms has passed
  {
    ulCurrentMicros = ulPreviousMicros;
     
      ul_3_Second_timer = ul_3_Second_timer = 1;
      if(ul_3_Second_timer > 3000)
      {
        ul_3_Second_timer = 0;
        bt_3_S_Time_Up = true;
      }
  
  //Mode push button debounce and toggle
    if(!digitalRead(MODE_BUTTON))
    {
      //button is pressed
      if(uiMode_PB_Debounce <= 256)   //~25.6 mS debounce
      {
          uiMode_PB_Debounce = uiMode_PB_Debounce + 1;
          if(uiMode_PB_Debounce > 255)
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
      if(uiMode_PB_Debounce <= 256)
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
           // bt_Cal_Initialized = false;
           // ui_Cal_Cycle = 0;          
          }
      }
    }
  
  
  // check if drive motors should be powered
  bt_Motors_Enabled = !digitalRead(MOTOR_ENABLE_SWITCH); // if sw1-5 ia on then motors are enabled

  // modes 
  // 0 = default after power up/reset
  // 1 = Press mode button once to enter. Run robot.
  // 2 = Press mode button twice to enter. Calibrate line tracker light level.
  // 3 = Press mode button three times to enter. Calibrate line tracker dark level.
  // 4 = Press mode button four times to enter. Calibrate motor speeds to drive straight.
  switch(ui_Robot_Mode_Index)
  {
    case 0:    //Robot stopped
    {
     
     // encoder_LeftMotor.zero();
     // encoder_RightMotor.zero();
     
      break;
    } 
  
    case 1:    //Robot Run after 3 seconds
    {
      if(bt_3_S_Time_Up)
      {
       
#ifdef DEBUG_ENCODERS           
        l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
        l_Right_Motor_Position = encoder_RightMotor.getRawPosition();

        Serial.print("Encoders L: ");
        Serial.print(l_Left_Motor_Position);
        Serial.print(", R: ");
        Serial.println(l_Right_Motor_Position);
#endif

       // set motor speeds
        ui_Left_Motor_Speed = constrain(ui_Motors_Speed + ui_Left_Motor_Offset, 1600, 2100);
        ui_Right_Motor_Speed = constrain(ui_Motors_Speed + ui_Right_Motor_Offset, 1600, 2100);

       

        if(bt_Motors_Enabled)
        {
          servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
          servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
        }
        else
        {  
          servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop); 
          servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop); 
        }

#ifdef DEBUG_MOTORS
        Serial.print("Motors enabled: ");
        Serial.print(bt_Motors_Enabled);
        Serial.print(", Default: ");
        Serial.print(ui_Motors_Speed);
        Serial.print(", Left = ");
        Serial.print(ui_Left_Motor_Speed);
        Serial.print(", Right = ");
        Serial.println(ui_Right_Motor_Speed);
#endif    
        
      }
      break;
    } 
    
    case 2:    //Calibrate line tracker light levels after 3 seconds
    {
      if(bt_3_S_Time_Up)
      {
        if(!bt_Cal_Initialized)
        {
          bt_Cal_Initialized = true;
          ui_Left_Line_Tracker_Light = 0;
          ui_Middle_Line_Tracker_Light = 0;
          ui_Right_Line_Tracker_Light = 0;
          ul_Calibration_Time = millis();
          ui_Cal_Count = 0;
        }
        else if((millis() - ul_Calibration_Time) > ci_Line_Tracker_Calibration_Interval)
        {
          ul_Calibration_Time = millis();
          readLineTrackers();
          ui_Left_Line_Tracker_Light += ui_Left_Line_Tracker_Data;
          ui_Middle_Line_Tracker_Light += ui_Middle_Line_Tracker_Data;
          ui_Right_Line_Tracker_Light += ui_Right_Line_Tracker_Data;
          ui_Cal_Count++;
        }
        if(ui_Cal_Count == ci_Line_Tracker_Cal_Measures)
        {
          ui_Left_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
          ui_Middle_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
          ui_Right_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
          Serial.print("Light Levels: Left = ");
          Serial.print(ui_Left_Line_Tracker_Light,DEC);
          Serial.print(", Middle = ");
          Serial.print(ui_Middle_Line_Tracker_Light,DEC);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Line_Tracker_Light,DEC);
#endif           
          EEPROM.write(ci_Left_Line_Tracker_Light_Address_L, lowByte(ui_Left_Line_Tracker_Light));
          EEPROM.write(ci_Left_Line_Tracker_Light_Address_H, highByte(ui_Left_Line_Tracker_Light));
          EEPROM.write(ci_Middle_Line_Tracker_Light_Address_L, lowByte(ui_Middle_Line_Tracker_Light));
          EEPROM.write(ci_Middle_Line_Tracker_Light_Address_H, highByte(ui_Middle_Line_Tracker_Light));
          EEPROM.write(ci_Right_Line_Tracker_Light_Address_L, lowByte(ui_Right_Line_Tracker_Light));
          EEPROM.write(ci_Right_Line_Tracker_Light_Address_H, highByte(ui_Right_Line_Tracker_Light));
          ui_Robot_Mode_Index = 0;    // go back to Mode 0
        }
       
      }
      break;
    }
    
    case 3:    // Calibrate line tracker dark levels after 3 seconds
    {
      if(bt_3_S_Time_Up)
      {
        if(!bt_Cal_Initialized)
        {
          bt_Cal_Initialized = true;
          ui_Left_Line_Tracker_Dark = 0;
          ui_Middle_Line_Tracker_Dark = 0;
          ui_Right_Line_Tracker_Dark = 0;
          ul_Calibration_Time = millis();
          ui_Cal_Count = 0;
        }
        else if((millis() - ul_Calibration_Time) > ci_Line_Tracker_Calibration_Interval)
        {
          ul_Calibration_Time = millis();
          readLineTrackers();
          ui_Left_Line_Tracker_Dark += ui_Left_Line_Tracker_Data;
          ui_Middle_Line_Tracker_Dark += ui_Middle_Line_Tracker_Data;
          ui_Right_Line_Tracker_Dark += ui_Right_Line_Tracker_Data;
          ui_Cal_Count++;
        }
        if(ui_Cal_Count == ci_Line_Tracker_Cal_Measures)
        {
          ui_Left_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
          ui_Middle_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
          ui_Right_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
          Serial.print("Dark Levels: Left = ");
          Serial.print(ui_Left_Line_Tracker_Dark,DEC);
          Serial.print(", Middle = ");
          Serial.print(ui_Middle_Line_Tracker_Dark,DEC);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Line_Tracker_Dark,DEC);
#endif           
          EEPROM.write(ci_Left_Line_Tracker_Dark_Address_L, lowByte(ui_Left_Line_Tracker_Dark));
          EEPROM.write(ci_Left_Line_Tracker_Dark_Address_H, highByte(ui_Left_Line_Tracker_Dark));
          EEPROM.write(ci_Middle_Line_Tracker_Dark_Address_L, lowByte(ui_Middle_Line_Tracker_Dark));
          EEPROM.write(ci_Middle_Line_Tracker_Dark_Address_H, highByte(ui_Middle_Line_Tracker_Dark));
          EEPROM.write(ci_Right_Line_Tracker_Dark_Address_L, lowByte(ui_Right_Line_Tracker_Dark));
          EEPROM.write(ci_Right_Line_Tracker_Dark_Address_H, highByte(ui_Right_Line_Tracker_Dark));
          ui_Robot_Mode_Index = 0;    // go back to Mode 0
        }
        
      }
      break;
    }
   
    case 4:    //Calibrate motor straightness after 3 seconds.
    {
      if(bt_3_S_Time_Up)
      {
        if(!bt_Cal_Initialized)
        {
          bt_Cal_Initialized = true;
          encoder_LeftMotor.zero();
          encoder_RightMotor.zero();
          ul_Calibration_Time = millis();
          servo_LeftMotor.writeMicroseconds(ui_Motors_Speed);
          servo_RightMotor.writeMicroseconds(ui_Motors_Speed);
        }
        else if((millis() - ul_Calibration_Time) > ci_Motor_Calibration_Time) 
        {
          servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop); 
          servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop); 
          l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
          l_Right_Motor_Position = encoder_RightMotor.getRawPosition();
          if(l_Left_Motor_Position > l_Right_Motor_Position)
          {
           // May have to update this if different calibration time is used
            ui_Right_Motor_Offset = 0;
            ui_Left_Motor_Offset = (l_Left_Motor_Position - l_Right_Motor_Position) / 4;  
          }
          else
          {
           // May have to update this if different calibration time is used
            ui_Right_Motor_Offset = (l_Right_Motor_Position - l_Left_Motor_Position) / 4;
            ui_Left_Motor_Offset = 0;
          }
          
#ifdef DEBUG_MOTOR_CALIBRATION
          Serial.print("Motor Offsets: Left = ");
          Serial.print(ui_Left_Motor_Offset);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Motor_Offset);
#endif              
          EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_Right_Motor_Offset));
          EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_Right_Motor_Offset));
          EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_Left_Motor_Offset));
          EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_Left_Motor_Offset));
          
          ui_Robot_Mode_Index = 0;    // go back to Mode 0 
        }
#ifdef DEBUG_MOTOR_CALIBRATION           
          Serial.print("Encoders L: ");
          Serial.print(encoder_LeftMotor.getRawPosition());
          Serial.print(", R: ");
          Serial.println(encoder_RightMotor.getRawPosition());
#endif        
        
      } 
      break;
    }    
  }
  ul_Display_Time++;

  if(ul_Display_Time > ci_Display_Time)
  {
    ul_Display_Time = 0;

    //Change brightness to indicate heart beat 
    LEDBrightnessIndex++;
    if(LEDBrightnessIndex > 17)
    {
      LEDBrightnessIndex = 0;
    }
unsigned char LEDMaxBrightness[]
    SmartLEDs.setBrightness(LEDMaxBrightness);
    SmartLEDs.show();   // Send the updated pixel colors to the hardware.
    
    Indicator();
  }
} 

// set mode indicator LED state
void Indicator()
{
  //display routine, if true turn on led
  CharliePlexM::Write(ci_Indicator_LED,!(ui_Mode_Indicator[ui_Mode_Indicator_Index] & 
                      (iArray[iArrayIndex])));
  iArrayIndex++;
  iArrayIndex = iArrayIndex & 15;
}



// // measure distance to target using ultrasonic sensor  
// void Ping()
// {
//   //Ping Ultrasonic
//   //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
//   digitalWrite(ci_Ultrasonic_Ping, HIGH);
//   delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
//   digitalWrite(ci_Ultrasonic_Ping, LOW);
//   //use command pulseIn to listen to Ultrasonic_Data pin to record the
//   //time that it takes from when the Pin goes HIGH until it goes LOW 
//   ul_Echo_Time = pulseIn(ci_Ultrasonic_Data, HIGH, 10000);

//   // Print Sensor Readings
// #ifdef DEBUG_ULTRASONIC
//   Serial.print("Time (microseconds): ");
//   Serial.print(ul_Echo_Time, DEC);
//   Serial.print(", Inches: ");
//   Serial.print(ul_Echo_Time/148); //divide time by 148 to get distance in inches
//   Serial.print(", cm: ");
//   Serial.println(ul_Echo_Time/58); //divide time by 58 to get distance in cm 
// #endif
// }  





