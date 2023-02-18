# MSE-Base

This is the base code that is for used for the MSEbot that is constructed, tested, and programmed in MSE 2202 Labs 3 and 4. 

Note that this code is regularly updated to add and improve functionality. It is highly recommended that you clone the repository and regularly ensure that it is up to date via `git pull`. Your own code should be developed in a separate branch to simplify updates and merging.

The code depends on the [MSE2202_Lib](https://github.com/MSE2202/MSE2202_Lib) library. It is also recommended that it is cloned and installed via the Arduino IDE.

## Pin map

The code assumes that the MSEbot components are connected according to the following table. Note that for proper operation, all of the switches in the DIP switch S1, except for S1-6, must be in the ON position.

| GPIO | Jumper | Function | Note |
|------|------|------|------|
| 0    | Hardwired | Mode button | PB1 |
| 1    | J1  | Potentiometer on AD1-0 | S1-12 must be ON |
| 3    | J3  | Motor enable | S1-5 ON to enable drive motors |
| 9    | J9  | IR Receiver |  |
| 10   | J10 | Claw limit switch | |
| 11   | J11 | Right encoder A signal | S1-7 must be ON |
| 12   | J12 | Right encoder B signal | S1-8 must be ON |
| 13   | J13 | Right encoder direction signal | S1-9 must be ON |
| 14   | J14 | Right encoder speed signal | S1-10 must be ON |
| 15   | J15 | Left encoder A signal | S1-1 must be ON |
| 16   | J16 | Left encoder B signal | S1-2 must be ON |
| 17   | J17 | Left encoder direction signal | S1-3 must be ON |
| 18   | J18 | Left encoder speed signal | S1-4 must be ON |
| 21   | J21 | SmartLED | S1-11 must be ON |
| 35   | J35 | Motor 1 A | Connect to IN1 pin on MX1508 driver |
| 36   | J36 | Motor 1 B | Connect to IN2 pin on MX1508 driver |
| 37   | J37 | Motor 2 A | Connect to IN3 pin on MX1508 driver |
| 38   | J38 | Motor 2 B | Connect to IN4 pin on MX1508 driver |
| 39   | J39 | Stepper direction | Connect to DIRECTION pin on A4988 driver |
| 40   | J40 | Stepper clock/step | Connect to STEP pin on A4988 driver |
| 41   | J41 | Arm shoulder servo | Connect to signal wire of RC servo |
| 42   | J42 | Claw servo | Connect to signal wire of RC servo

## Run modes

The base code provides several predefined modes. These can be selected by pressing PB1 a number of times. The current mode is indicated by the colour of the Smart LED. The modes are as follows.

| Mode | Button presses | LED colour | Description | Note |
|------|------|------|------|------|
| 0 | — | Red | Robot stopped | Default after power up/reset |
| 1 | 1 | Green | Run robot | Robot will drive in a test sequence |
| 2 | 2 | Blue | Test stepper motor | Pot will control stepper/arm turret |
| 3 | 3 | Yellow | Test claw servo | Pot will control servo/claw |
| 4 | 4 | Magenta | Test shoulder servo | Pot will control servo/arm |
| 5 | 5 | Cyan | Test IR receiver | Received characters sent to serial port |
| 6 | 6 | Orange | Undefined | Can add additional mode here |

Note that all modes, except for 1 and 2 start immediately. The robot test sequence (Mode 1) will start after 5 seconds. It consists of the robot driving in the following sequence:

1. Forward for 2 seconds
2. Reverse for 2 seconds
3. Turn left (clockwise) for 2 seconds
4. Turn right (counterclockwise) for 2 seconds
5. Stop for 2 seconds

The sequence will continue until the mode is changed or the MSE-Duino is reset. The potentiometer is used to set the speed of the motors. The maximum motor speed is when the pot is turned fully counterclockwise. Performance at the lowest speed may be quite poor. If `DEBUG_DRIVE_SPEED` is defined, the pot value and motor speed are sent to the serial port. Similarly, if `DEBUG_ENCODER_COUNT` is defined, and the raw data from the motor encoders is sent to the serial port.

Mode 2 uses the potentiometer to set the desired number of steps from a centre position. If connected to the MSEbot, the zero position is when the arm is facing straight ahead. To prevent going past the end of the stepper rack, the potentiometer must be centred before motion will start. Output is sent to the serial port, which may be monitored to help find the appropriate starting position for the pot and monitor the step parameters during operation.

The range of the servos in Modes 3 and 4 is set by constants in the base code: `ci_Claw_Servo_Open`, `ci_Claw_Servo_Closed`, `ci_Shoulder_Servo_Retracted` and `ci_Shoulder_Servo_Extended`. These will likely need to be **carefully** adjusted for each robot to adjust the range of motion of the claw and arm, respectively. The raw pot values and servo position are sent to the serial port.
