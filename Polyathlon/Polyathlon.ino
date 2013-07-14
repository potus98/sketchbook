////////////////////////////////////////////////////////////////////////
// by John Christian
//    potus98@yahoo.com
//    potus98.com
//    @potus98 on twitter
//
// Code below is not entirely original. It may have snippets or entire
// sections from other examples or tutorials found online. Thank you
// wonderful Internet for the help. If you see something you feel should
// be specifically credited, please let me know! 
//
// Pololu is awesome. They have great customer service, great prices,
// and great prodoucts. Not affiliated in any way. Just a satisfied customer.
////////////////////////////////////////////////////////////////////////
// This example uses
//  - Arduino Mega 2560
//  - NXT Motor Shield
//  - Pololu QTR-8A IR Reflectance Sensor
//
////////////////////////////////////////////////////////////////////////
// Significant re-write from previous versions
//  - Removing long comments/explanations
//  - Leveraging NXTShield libraries


// The emitter control pin can optionally be connected to digital pin 2, or you can leave
// it disconnected and change the EMITTER_PIN #define below from 2 to QTR_NO_EMITTER_PIN.

////////////////////////////////////////////////////////////////////////
// Prepare NXT Motor Shield libraries
#include <Wire.h>
#include <NXTShield.h>
Motor1 leftMotor;
Motor2 rightMotor;
  
////////////////////////////////////////////////////////////////////////
// Prepare QTR-8A IR Reflectance Array
#include <QTRSensors.h>
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             27  // emitter is controlled by digital pin 27
// sensors 0 through 7 are connected to analog inputs 0 through 5, 7, and 8, respectively
//QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5, 7, 8}, 
QTRSensorsAnalog qtra((unsigned char[]) {8, 9, 10, 11, 12, 13, 14, 15}, // new wiring harness
NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

////////////////////////////////////////////////////////////////////////
// Prepare LEDs and pushbuttons
// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin (probably need to change, 13 is popular for other things)
// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

////////////////////////////////////////////////////////////////////////
// Prepare PID library
#include <PID_v1.h>   // include the PID library See: http://playground.arduino.cc/Code/PIDLibrary
// Initialize variables related to PID
double PIDsetpoint, PIDinput, PIDoutput;
//PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,2,5,1, DIRECT);  //2,5,1 is the default example in the library
//PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.1,0,0, DIRECT);  // works okay, or at least makes sense
//PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.025,0,0, DIRECT);  // works at first, then oscilates more and more
//PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.025,0,.1, DIRECT);  // oscilates sooner than prior
//PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.025,.1,0, DIRECT);  // better than previous, oscilates later
//PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.025,.05,.05, DIRECT);  // step backward
//PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.025,.025,0, DIRECT);  // good straight, once oscilates, falls apart fast.
                                                                     // oscilation due to go straight related to differing motor strength?
// added .93 motor power correction for motorA, repeating tests...
//PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.025,0,0, DIRECT);  // softer, farthest straight so far, but still ends in oscilation
//PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.075,0,0, DIRECT);  // increasing oscillation
//PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.5,0,0, DIRECT);  // increasing oscillation sooner than previous
//PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.025,.5,0, DIRECT);  // oscillation similar, harder
//PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.01,0,0, DIRECT);  // soft drifty, loses line, but re-finds smoothly if slowly
//PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.015,0,0, DIRECT);  // drifty at first, then starts oscillating
//PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.01,.5,0, DIRECT);  // better at first, starts oscillating
//PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.01,.1,0, DIRECT);  // a little better at first, then similar to previous
//PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.01,.05,0, DIRECT);  // best so far, last longest, does oscillate eventually

//resuming project on 7/11/2013 after PID Balance Beam project
PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.1,0,0, DIRECT);

// from br3ttb on arduino forums... Parameters and what they do (sort of)
// P_Param: the bigger the number the harder the controller pushes.
// I_Param: the SMALLER the number (except for 0, which turns it off,)  the more quickly the controller reacts to load changes, but the greater the risk of oscillations.
// D_Param: the bigger the number the more the controller dampens oscillations (to the point where performance can be hindered)

int PIDoutputMapped = 0;       // used when mapping PID output to safe servo range so servo doesn't twist demo rig apart
int PIDoutputConstrained = 0;  // extra safety against sending servo to positions that might break the demo rig
//const int pingPin = 7;         // defines signal pin used by ping sensor
//int timeToPing = 0;            // used during ping sensor operation
//int BallPosition = 0;          // calculated distance of ball from sensor
//int setPoint = 30;             // hardcoded target point (to-do: make adjustable via potentiometer)
//int servoPos = 90;             // servo position
int TestRuns = 0;              // counter used to limit total number of cycles to make data collection easier
int PIDoutputABS = 0;          // used when switching negatives to absolute values with abs() function

int speedMotorA = 0;
int speedMotorB = 0;
  
// wheelie popping might be complicating? it's tiny, but it does happen occassionally

////////////////////////////////////////////////////////////////////////
// Prepare 
////////////////////////////////////////////////////////////////////////
void setup()
{
  ////////////////////////////////////////////////////////////////////////
  // initialize serial communication
  Serial.begin(115200);

  ////////////////////////////////////////////////////////////////////////
  // initialize LED pin as an output
  pinMode(ledPin, OUTPUT);

  ////////////////////////////////////////////////////////////////////////
  // initialize pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  
  ////////////////////////////////////////////////////////////////////////
  // initialize maximum speed
  // Intent here is to have one place to set a relative max speed from which
  // other functions would calibrate.
  // Scale limit is arduino PWM of 0-255.
  // This may need to become a per-function setting
  int maxSpeed = 150;

  ////////////////////////////////////////////////////////////////////////
  // initialize PID variables we're linked to
  PIDinput = qtra.readLine(sensorValues); // 
  PIDsetpoint = 3500;                     // attempt to stay centered over line
  //myPID.SetOutputLimits(-90,90);              // set output limits
  myPID.SetOutputLimits(-255,255);
  myPID.SetMode(AUTOMATIC);               // turn the PID on

  ////////////////////////////////////////////////////////////////////////
  // calibrate IR array (might move this into a function later)
  delay(500);
  int i;
  
  Serial.println("Running calibration (see the blinky red light)?");
  for (i = 0; i < 200; i++)      // run calibration for a few seconds
  {
    digitalWrite(ledPin, HIGH);  // turn on LED (flicker LED during calibration)
    delay(20);
    qtra.calibrate();            // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
    digitalWrite(ledPin, LOW);   // turn off LED
    delay(20);
  }
  digitalWrite(ledPin, LOW);     // turn off LED to indicate we are through with calibration
  // print the calibration MINimum values measured when emitters were on
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  // print the calibration MAXimum values measured when emitters were on
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
  ////////////////////////////////////////////////////////////////////////
  // call pause function at end of setup to wait for button push
  pause(); 
} //end of void setup

////////////////////////////////////////////////////////////////////////
void loop()
////////////////////////////////////////////////////////////////////////
{
  //Serial.println ("Starting void loop");
  //navigationSensorTest(); // display reflectance sensor array reading
  //navigationLogicA();     // has 3 conditions: straight, turn left, turn right
  //navigationLogicB();     // has 7 states with varying motor responses
  ////navigationLogicC();       // uses PID library
  //navigationLogicD();     // lookup tables, basically
  //navigationLogicE();     // basic PID (non library) implementation
  navigationLogicF();     // uses PID library, after Balance Beam project
  //diagnosticDrive(5); 
  //PIDTestNoMotors();
} //end of void loop

////////////////////////////////////////////////////////////////////////
/* FUNCTIONS  */
////////////////////////////////////////////////////////////////////////

int pause(){
  // standby until momentary button is pressed
  // useful after calibration is complete, but before the event starts
  allStop(); // probably not necessary but may use in future
  buttonState = digitalRead(buttonPin);
  // buttsonState is HIGH while untouched, pressed button causes LOW
    while(buttonState == HIGH){
    buttonState = digitalRead(buttonPin);
    digitalWrite(ledPin, HIGH);    // turn on LED (slow blink waiting)
    delay(250);
    digitalWrite(ledPin, LOW);     // turn off LED
    delay(250); 
  }
  Serial.println("Button pressed! Start 5 second delay...");
  delay(5000);
  Serial.println("Time's up! Leaving pause loop.");
}


int allStop(){
  // turn off drive motors
  Serial.println("allStop - stopping both motors");
  leftMotor.stop();
  rightMotor.stop();
}
 

int driveForward(int speedMotorA, int speedMotorB){
  // based on testing data, motorA/right.motor is slightly more powerful
  // should shave about 7% off motorA's power
  //Serial.print("speedMotorA converted from ");
  //Serial.print(speedMotorA);
  //Serial.print(" to ");
  speedMotorA = speedMotorA * .93;
  //Serial.print(speedMotorA);
  //
  // Amping up the power a little without changing values in navigationLogicC
  speedMotorA = speedMotorA * 1.5;
  speedMotorB = speedMotorB * 1.5;
  //
  Serial.print("driveForward function: ");
  Serial.print(speedMotorA);
  Serial.print(" ");
  Serial.print(speedMotorB);
  // function accepts two arguments: speed for motorA, motorB
  // future args may include specific time to run
  // or distance to cover (if using wheel encoder)
  leftMotor.move(forward, speedMotorA);
  rightMotor.move(forward, speedMotorB);
}

/*
int driveForwardFake(int speedMotorA, int speedMotorB){  // fake driveForward for testing
  Serial.print("driveForwardFake function: ");
  Serial.print(speedMotorA);
  Serial.print(" ");
  Serial.print(speedMotorB);
}
*/

int driveBackward(int speedMotorA, int speedMotorB){
  // function accepts two args: speed for motorA, motorB
  // future args may include specific time to run
  // or distance to cover (for use with countable wheels)
  leftMotor.move(backward, speedMotorA);
  rightMotor.move(backward, speedMotorB);
}

/*
int pivotCounterClockwise(int speedMotor, int pivotTime){
  // function accepts two args: speed for motors and time to run
  // pivots counter clockwise
  // speedMotor adjusts how fast to pivot. >100 good to overcome standing resistance
  // pivotTime adjusts how long to pivot
  //
  // Based on tests with reasonably new batteries
  // 180 - 210 degrees = 110,1000
  //  40 -  80 degrees = 110, 400
  //  50 - 110 degrees = 100, 500
  // Bleh! Trying to use power and timing to perform semi-accurate turns doesn't seem feasible
  // with current setup. The resulting degrees of turn vary wildly. Will need to access the
  // wheel encoder information to make useful pivots.
  //
  int speedMotorA = speedMotor;
  int speedMotorB = speedMotor;
  Serial.print("pivotCounterClockwise function: ");
  Serial.print(speedMotorA);
  Serial.print(" ");
  Serial.print(speedMotorB);
  Serial.print(" Time: ");
  Serial.println(pivotTime);
  digitalWrite(dir_a, LOW);
  digitalWrite(dir_b, HIGH);
  analogWrite(pwm_a, speedMotorA);
  analogWrite(pwm_b, speedMotorB);
  delay(pivotTime);
}
*/

/*
int pivotClockwise(int speedMotor, int pivotTime){
  // function accepts two args: speed for motors and time to run
  // pivots counter clockwise
  // speedMotor adjusts how fast to pivot. >100 good to overcome standing resistance
  // pivotTime adjusts how long to pivot
  //
  // Based on tests with reasonably new batteries
  // 180 - 210 degrees = 110,1000
  //  40 -  80 degrees = 110, 400
  //  50 - 110 degrees = 100, 500
  // Bleh! Trying to use power and timing to perform semi-accurate turns doesn't seem feasible
  // with current setup. The resulting degrees of turn vary wildly. Will need to access the
  // wheel encoder information to make useful pivots.
  //
  int speedMotorA = speedMotor;
  int speedMotorB = speedMotor;
  Serial.print("pivotClockwise function: ");
  Serial.print(speedMotorA);
  Serial.print(" ");
  Serial.print(speedMotorB);
  Serial.print(" Time: ");
  Serial.println(pivotTime);
  digitalWrite(dir_a, HIGH);
  digitalWrite(dir_b, LOW);
  analogWrite(pwm_a, speedMotorA);
  analogWrite(pwm_b, speedMotorB);
  delay(pivotTime);
}
*/

/*
int setDirection(int positionToCheck){
  // function accepts the position value which should be between 1 and 6999 then
  // evaluate the position value and adjust motorA/B speeds to stay on course.
  // 
}
*/

int diagnosticDrive(int secondsToDrive){
  // drive forward for secondsToDrvie seconds
  //motorA is passenger side, motorB is driver side
  driveForward(135,150);
  delay(5000); // hardcoding 5 seconds for now
  allStop();
  delay(50000);
}

/* 
int BackupTurnFunction(){
  // This is a basic, pre-defined backup and turn routine. You could call this
  // when, say, a forward facing ping sensor detects an object in the bot's path
  digitalWrite(dir_a, HIGH);  // Set motor direction, 1 low, 2 high
  digitalWrite(dir_b, HIGH);  // Set motor direction, 3 high, 4 low
  analogWrite(pwm_a, 150);    //
  analogWrite(pwm_b, 75);     // running motors at different speed results in backup turn
  delay (3000);               // give the motor controller and motors a moment to come to a stop
  analogWrite(pwm_a, 0);      //
  analogWrite(pwm_b, 0);      //
  delay (2000);               // set both motors to off for a moment
  return 0;                   // may use return values in the future, but for now,
                              // this value is not used
}
*/

////////////////////////////////////////////////////////////////////////
int navigationSensorTest(){
////////////////////////////////////////////////////////////////////////
  // 
  // Show sensor readings
  //
  // read calibrated sensor values and obtain a measure of the line position from 0 to 7000.
  unsigned int position = qtra.readLine(sensorValues);
  unsigned char i;
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i] * 10 / 1001);
    Serial.print(' ');
  }
  Serial.print("  ");
  Serial.print(position);
  Serial.print("  ");
  delay(200); // Adjust for readability in Serial console
  unsigned int sensors[8];
  if (sensorValues[0] > 750 && sensorValues[1] > 750 && sensorValues[2] > 750 && sensorValues[3] > 750 && sensorValues[4] > 750 && sensorValues[5] > 750 && sensorValues[6] > 750 && sensorValues[7] > 750)
  {
    Serial.println(" Whoa! Everything looks black!");
  }
  if (sensorValues[0] < 80 && sensorValues[1] < 80 && sensorValues[2] < 80 && sensorValues[3] < 80 && sensorValues[4] < 80 && sensorValues[5] < 80 && sensorValues[6] < 80 && sensorValues[7] < 80)
  {
    Serial.println(" Whoa! Everything looks white!");
  } 
  
  // CHECK for a 90 degree intersection
  //
  
  
  // By this point, we're assuming no 90 degree intersection or 90 degree turns
  //
  // Determine drive action based on line position value between 0 and 7000.
  // This logic assumes a single line. No provision for intersections at this time.
  if (position >= 0 && position < 1000){
    Serial.println("          Right Strong");
    driveForward(90, 55);
  }
  else if (position >= 1000 && position < 2000){
    Serial.println("          Right Moderate");
    driveForward(90, 75);
  }
  else if (position >= 2000 && position < 3000){
    Serial.println("          Right Slight");
    driveForward(100, 85);
  }
  else if (position >= 3000 && position < 4000){
    Serial.println("          Straight Ahead");
    driveForward(100, 100);
  }
  else if (position >= 4000 && position < 5000){
    Serial.println("          Left Slight");
    driveForward(85, 100);
  }
  else if (position >= 5000 && position < 6000){
    Serial.println("          Left Moderate");
    driveForward(75, 90);
  }
  else if (position >= 6000 && position <= 7000){
    Serial.println("          Left Strong");
    driveForward(55, 90);
  }
}


////////////////////////////////////////////////////////////////////////
// int navigationLogicA(){ See LineFollower_v13 }
////////////////////////////////////////////////////////////////////////
// int navigationLogicB(){ See LineFollower_v13 }
////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////
int navigationLogicC(){
////////////////////////////////////////////////////////////////////////
  // Actual PID
  //
  int speedMotorA = 255;
  int speedMotorB = 255;
  // read calibrated sensor values and obtain a measure of the line position from 0 to 7000.
  unsigned int position = qtra.readLine(sensorValues);
  unsigned char i;
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i] * 10 / 1001);
    Serial.print(' ');
  }
  Serial.print("  ");
  Serial.print(position);
  Serial.print("  ");
  //delay(250); // Might consider removing this delay for actual competition
  ////////////////////////////////////////////////////////////////////////
  unsigned int sensors[8];
  // if all eight sensors see very low reflectance (black), take some appropriate action for this situation
  // might need to revisit hardcoding 750. should consider deriving from min/max during calibration sequence
  if (sensorValues[0] > 750 && sensorValues[1] > 750 && sensorValues[2] > 750 && sensorValues[3] > 750 && sensorValues[4] > 750 && sensorValues[5] > 750 && sensorValues[6] > 750 && sensorValues[7] > 750)
  {
    Serial.println(" Whoa! Everything looks black!");
    allStop();
    //delay(3000);
    // do something.  Maybe this means we're at the edge of a course or about to fall off a table,
    // in which case, we might want to stop moving, back up, and turn around.
    //return;
  }
  // if all eight sensors see very high reflectance (white), take some appropriate action for this situation
  // might need to revisit hardcoding 80. should consider deriving from min/max during calibration sequence
  if (sensorValues[0] < 80 && sensorValues[1] < 80 && sensorValues[2] < 80 && sensorValues[3] < 80 && sensorValues[4] < 80 && sensorValues[5] < 80 && sensorValues[6] < 80 && sensorValues[7] < 80)
  {
    Serial.println(" Whoa! Everything looks white!");
    allStop();
    // do something.  Maybe this means we're at the edge of a course or about to fall off a table,
    // in which case, we might want to stop moving, back up, and turn around.
    //return;
  } 
  // Determine drive action based on line position value between 0 and 7000.
  // This logic assumes a single line. No provision for intersections at this time.
  
  PIDinput = qtra.readLine(sensorValues);
  myPID.Compute();
  Serial.print("PIDoutput: ");
  Serial.println(PIDoutput);
  
  // Okay, so now that I have PIDoutput ranging from -255 to 255. How should I put it to use?
  // -255 (position is >3500) = need to turn left, reduce power to MotorB/left.motor
  // +255 (position is <3500) = need to turn right, reduce power to MotorA/right.motor
  
  // I don't really need the negative number, I just need to konw relative distance from setpoint of 3500
  int correctionAmount = abs(PIDoutput);
  
  if (position >= 3500){             // need to vere left, reduce power to MotorB/left.motor
    Serial.println("    vere Left");
    speedMotorA = 150 - correctionAmount;
    speedMotorA = speedMotorA * .93;
    speedMotorB = 150;
    leftMotor.move(forward, speedMotorA);
    rightMotor.move(forward, speedMotorB);
  }

  if (position < 3500){             // need to vere right, reduce power to MotorA/right.motor
    Serial.println("    vere Right");
    speedMotorA = 150;
    speedMotorA = speedMotorA * .93;
    speedMotorB = 150 - correctionAmount;
    leftMotor.move(forward, speedMotorA);
    rightMotor.move(forward, speedMotorB);
  } 
}



////////////////////////////////////////////////////////////////////////
int navigationLogicD(){
////////////////////////////////////////////////////////////////////////
Serial.println("Enterting navigationLogicD function");
  // Sort-of a poor man's PID
  //
  // Derived from navigationLogicB but adding 90 degree intersection and 90 degree turn handling
  //
  // read calibrated sensor values and obtain a measure of the line position from 0 to 7000.
  unsigned int position = qtra.readLine(sensorValues);
  unsigned char i;
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i] * 10 / 1001);
    Serial.print(' ');
  }
  Serial.print("  ");
  Serial.print(position);
  Serial.println("  ");
  ////////////////////////////////////////////////////////////////////////
  unsigned int sensors[8];
  // get calibrated sensor values returned in the sensors array, along with the line position
  // position will range from 0 to 7000, with 3000 corresponding to the line over sensor 4
  // Will be treating sensor 4 (array position 5) as middle sensor
  //int position = qtr.readLine(sensors);
  // if all eight sensors see very low reflectance (black), take some appropriate action for this situation
  // might need to revisit hardcoding 750. should consider deriving from min/max during calibration sequence
  
  // CHECK for all black (all sensors very low reflectance) won't need for line following loop course.
  if (sensorValues[0] > 750 && sensorValues[1] > 750 && sensorValues[2] > 750 && sensorValues[3] > 750 && sensorValues[4] > 750 && sensorValues[5] > 750 && sensorValues[6] > 750 && sensorValues[7] > 750)
  {
    Serial.println("    DETECTED: All Black");
  }
  if (sensorValues[0] < 80 && sensorValues[1] < 80 && sensorValues[2] < 80 && sensorValues[3] < 80 && sensorValues[4] < 80 && sensorValues[5] < 80 && sensorValues[6] < 80 && sensorValues[7] < 80)
  {
    Serial.println("    DETECTED: All White");
  }
  
  /*
  // CHECK for a 90 degree intersection (all sensors low reflectance) a more relaxed version of all black check
  if (sensorValues[0] > 400 && sensorValues[1] > 400 && sensorValues[2] > 400 && sensorValues[3] > 400 && sensorValues[4] > 400 && sensorValues[5] > 400 && sensorValues[6] > 400 && sensorValues[7] > 400)
  {
    Serial.print("    DETECTED: 90 degree intersection");
    // keep going straight
    driveForward(100,100);
    delay(100); // drive forward 1/10 second

  }

  // CHECK for a 90 degree LEFT
  if (sensorValues[0] > 400 && sensorValues[1] > 400 && sensorValues[2] > 400 && sensorValues[3] > 400)
  {
    Serial.print("    DETECTED: 90 degree Left");
    // perform 90 degree left turn  
  }

  // CHECK for a 90 degree RIGHT
  if (sensorValues[4] > 400 && sensorValues[5] > 400 && sensorValues[6] > 400 && sensorValues[7] > 400)
  {
    Serial.print("    DETECTED: 90 degree Right");
  }
  */
  
  // By this point, we're assuming no 90 degree intersection or 90 degree turns
  //
  // Determine drive action based on line position value between 0 and 7000.
  // This logic assumes a single line. No provision for intersections at this time.
  if (position == 0){
    Serial.println("    Left Pivot");
    //pivotCounterClockwise(80,0);  // Seems to pivot too fast.
    driveForward(75, 0);            // Try pivot on left wheel instead of center of vehicle.
  }
  else if (position > 0 && position < 500){
    driveForward(80, 55);
  }
  else if (position >= 500 && position < 1000){
    driveForward(90, 60);
  }
  else if (position >= 1000 && position < 1500){
    driveForward(90, 65);
  }
  else if (position >= 1500 && position < 2000){
    driveForward(100, 75);
  }
  else if (position >= 2000 && position < 2500){
    driveForward(100, 80);
  }
  else if (position >= 2500 && position < 3000){
    driveForward(100, 90);
  }
  else if (position >= 3000 && position < 3500){
    driveForward(100, 100);
  }
  else if (position >= 3500 && position < 4000){
    driveForward(100, 100);
  }
  else if (position >= 4000 && position < 4500){
    driveForward(90, 100);
  }
  else if (position >= 4500 && position < 5000){
    driveForward(80, 100);
  }
  else if (position >= 5000 && position < 5500){
    driveForward(75, 100);
  }
  else if (position >= 5500 && position < 6000){
    driveForward(65, 90);
  }
  else if (position >= 6000 && position < 6500){
    driveForward(60, 90);
  }
  else if (position >= 6500 && position < 7000){
    driveForward(55, 80);
  }
  else if (position >= 7000){
    Serial.println("    Right Pivot ");
    //pivotClockwise(80,0);        // Seems to pivot too fast.
    driveForward(0,75);            // Try pivot on left wheel instead of center of vehicle.
  }
Serial.println("end of navigationLogicD function");
//delay(500); // Remove this delay for final testing and actual competition
}

////////////////////////////////////////////////////////////////////////
int navigationLogicE(){
////////////////////////////////////////////////////////////////////////
//Serial.println("Enterting navigationLogicE function");
  // Attempting an actual PID like implementation based on
  // http://www.scribd.com/doc/49813253/Building-Autonomous-Line-Followers-using-Arduino-and-PID
  //
  //
  // read calibrated sensor values and obtain a measure of the line position from 0 to 7000.
  unsigned int position = qtra.readLine(sensorValues);
  unsigned char i;
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i] * 10 / 1001);
    Serial.print(' ');
  }
  Serial.print("  ");
  Serial.print(position);
  Serial.println("  ");
  ////////////////////////////////////////////////////////////////////////
  unsigned int sensors[8];  // not sure why this is here.
  
  // set point should be 3500. This is the target value of "position". The target this algorithm is trying to achieve.
  int setPoint = 3500;
  int Kp = .1;
  int Ki = 1;
  int Kd = 1;
  int proportional = 3500;
  int integral = 1;
  int derivative = 1;
  int lastProportional = 1;
  int errorValue = 1;
  
  // Calculate PID
  proportional = position - setPoint;
  integral = integral + proportional;
  derivative = proportional - lastProportional;
  lastProportional = proportional;
  errorValue = int(proportional * Kp + integral * Ki + derivative * Kd);
  
  Serial.print("                    proportional: ");
  Serial.print(proportional);
  Serial.print(" integral: ");
  Serial.print(integral);
  Serial.print(" derivative: ");
  Serial.print(derivative);
  Serial.print(" lastProportional: ");
  Serial.print(lastProportional);
  Serial.print(" errorValue: ");
  Serial.println(errorValue);

  // might need to revisit hardcoding 750. should consider deriving from min/max during calibration sequence
  // CHECK for all black (all sensors very low reflectance) won't need for line following loop course.
  if (sensorValues[0] > 750 && sensorValues[1] > 750 && sensorValues[2] > 750 && sensorValues[3] > 750 && sensorValues[4] > 750 && sensorValues[5] > 750 && sensorValues[6] > 750 && sensorValues[7] > 750)
  {
    Serial.println("    DETECTED: All Black");
  }
  if (sensorValues[0] < 80 && sensorValues[1] < 80 && sensorValues[2] < 80 && sensorValues[3] < 80 && sensorValues[4] < 80 && sensorValues[5] < 80 && sensorValues[6] < 80 && sensorValues[7] < 80)
  {
    Serial.println("    DETECTED: All White");
  }
  
  /*
  removed dedicated 90 degree intersection, 90 degree left, and 90 degree right checks
  */
  
  // Determine drive action based on line position value between 0 and 7000.
  // This logic assumes a single line. No provision for intersections at this time.
  
  // Following is a John hack. Very simple P implementation assuming full motor speed is 100.
  // Take the proportional value (position - setPoint) which will range from -3500 to 3500 and
  // multiply by .1 to convert, say, 3500 to 35, then multiply by 2
  // Subtract the resulting value from max speed of motor.
  
  if (proportional < 0)
  {
    Serial.println("                    proportional is less than 0");
    // Turn left
    proportional = abs(proportional);
    Serial.print("abs of proportional is: ");
    Serial.println(proportional);
    proportional = ((proportional * .01) *2);
    Serial.print("adjusted proportional is: ");
    Serial.println(proportional);
    proportional = (100 - proportional);
    Serial.print("adjusted speed is: ");
    Serial.println(proportional);
    driveForward(100, proportional);
  }
  
  if (proportional > 0)
  {
    Serial.println("                    proportional is less than 0");
    // Turn right
    proportional = abs(proportional);
    Serial.print("abs of proportional is: ");
    Serial.println(proportional);
    proportional = ((proportional * .01) *2);
    Serial.print("adjusted proportional is: ");
    Serial.println(proportional);
    proportional = (100 - proportional);
    Serial.print("adjusted speed is: ");
    Serial.println(proportional);
    driveForward(proportional, 100);
  }

//Serial.println("end of navigationLogicE function");
//delay(300); // Remove this delay for final testing and actual competition
}

////////////////////////////////////////////////////////////////////////
int PIDTestNoMotors(){
  
  if (TestRuns == 0){
    Serial.println("PID tunings: x,y,z");                                         //   <<< Change PID here 2/2 <<<<
    Serial.println("PIDsetpoint LinePosition PIDoutput PIDoutputMapped");
  }  
  PIDinput = qtra.readLine(sensorValues);
  myPID.Compute();
  PIDoutputMapped = map(PIDoutput, 0, 255, 0, 255);
  //PIDoutputConstrained = constrain (PIDoutputMapped, 0, 255); // constrain values so servo won't tear up the rig
  //myservo.write(PIDoutputConstrained);
  delay(25); // Provide time for screen scrolling
  Serial.print(PIDsetpoint);
  Serial.print(" ");
  Serial.print(PIDinput);
  Serial.print(" ");
  Serial.print(PIDoutput);
  Serial.print(" ");
  Serial.println(PIDoutputMapped);
  TestRuns++;
  if ( TestRuns > 2000 ){
    Serial.print(TestRuns);
    Serial.println(" TestRuns complete. Pausing for 2 minutes...");
    Serial.println(" ");
    delay(120000);
    TestRuns = 0;
  }
} // close PIDTestNoMotors function

////////////////////////////////////////////////////////////////////////
int navigationLogicF(){
  
  if (TestRuns == 0){
    Serial.println("PID tunings: .1-0-0");                                         //   <<< Change PID here 2/2 <<<<
    Serial.println("PIDsetpoint LinePosition PIDoutput speedMotorA speedMotorB");
  }  
  PIDinput = qtra.readLine(sensorValues);
  myPID.Compute();
  //PIDoutputMapped = map(PIDoutput, 0, 255, 0, 255);
  delay(25); // Provide time for screen scrolling
  Serial.print(PIDsetpoint);
  Serial.print(",");
  Serial.print(PIDinput);
  Serial.print(",");
  Serial.print(PIDoutput);
  Serial.print(",");
  //Serial.println(PIDoutputMapped);
  
  // Okay, so now that I have PIDoutput ranging from -255 to 255. How should I put it to use?
  // -255 (position is >3500) = need to turn left, reduce power to MotorB/left.motor
  // +255 (position is <3500) = need to turn right, reduce power to MotorA/right.motor
  
  // maybe there's a couple of ways to think about this. One, if a motor is on the good side of
  // of the line, it should be full on and you shoud dial down the other motor to bring the bot
  // back over the line. This is a subtract from full power method.
  //
  // But what if you thought about having a single pool of power to distribute between the two motors?
  // What if the PIDoutput range (-255 to 255) was used to distribute the power pool across both motors?
  // At PIDoutput 0, both motors would receive an equal amount of power. At PIDoutput 128, one motor
  // would recieve 75% of the power while the other would receive 25%. This wouldn't necessarily need
  // to be linear, it could be a curve that softens adjustments near center, but becomes stronger
  // when far off center. This is an allocate from a single power pool method.
  // This might allow us to separate speed from steering. You could calculate the power allocation first,
  // then apply a speed setting to amplify the value before setting the motor speeds.
  // 
  
  // assume position 3400 - 3600 is dead zone or sweet spot (implement later?)
  // assume forward operating range of motors is min=50 to max=150.
  //
  // if the PIDoutput is negative, need to turn left
  //   - Set MotorA/right.motor to max
  //   - Set MotorB/left.motor to map of -255/0 to min/max
  // if the PIDoutput is positive, need to turn right
  //   - Set MotorA/right.motor to map ot 0/255 to min/max
  //   - Set MotorB/left.motor to max

  if ( PIDoutput < 0 ){
    PIDoutputABS=abs(PIDoutput); // convert to absolute value
    speedMotorA = 150;
    //speedMotorA = speedMotorA * .93; // compensate for tested out of of tolerance
    speedMotorB = map(PIDoutputABS, 255, 0, 50, 150);
    Serial.print(speedMotorA);
    Serial.print(",");
    Serial.print(speedMotorB);
    //leftMotor.move(forward, speedMotorA);
    //rightMotor.move(forward, speedMotorB);
  }
  
    if ( PIDoutput > 0 ){
    speedMotorB = 150;
    speedMotorA = map(PIDoutput, 255, 0, 50, 150);
    //speedMotorA = speedMotorA * .93; // compensate for tested out of of tolerance
    Serial.print(speedMotorA);
    Serial.print(",");
    Serial.print(speedMotorB);
    //leftMotor.move(forward, speedMotorA);
    //rightMotor.move(forward, speedMotorB);
  }

  /*
  if (position >= 3500){             // need to vere left, reduce power to MotorB/left.motor
    Serial.println("    vere Left");
    speedMotorA = 150 - correctionAmount;
    speedMotorA = speedMotorA * .93;
    speedMotorB = 150;
    leftMotor.move(forward, speedMotorA);
    rightMotor.move(forward, speedMotorB);
  }

  if (position < 3500){             // need to vere right, reduce power to MotorA/right.motor
    Serial.println("    vere Right");
    speedMotorA = 150;
    speedMotorA = speedMotorA * .93;
    speedMotorB = 150 - correctionAmount;
    leftMotor.move(forward, speedMotorA);
    rightMotor.move(forward, speedMotorB);
  } 
  */

  Serial.println(" ");
  
  TestRuns++;
  if ( TestRuns > 2000 ){
    Serial.print(TestRuns);
    Serial.println(" TestRuns complete. Pausing for 2 minutes...");
    Serial.println(" ");
    delay(120000);
    TestRuns = 0;
  }
} // close PIDTestNoMotors function
