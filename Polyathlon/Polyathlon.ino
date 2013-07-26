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
const int ledPin =  13;      // the number of the LED pin (probably need to change, 13 is popular for other things)

const int buttonPin = 33;    // the number of the pushbutton pin
const int buttonPinB = 35;   // the number of the second pushbutton pin
const int ledPinBin0 = 37;   // LED pin for binary status array of four LEDs
const int ledPinBin1 = 39;   // LED pin for binary status array of four LEDs
const int ledPinBin2 = 41;   // LED pin for binary status array of four LEDs
const int ledPinBin3 = 43;   // LED pin for binary status array of four LEDs
const int ledPinFL = 45;     // LED pin for Front Left status LED
const int ledPinFR = 47;     // LED pin for Front Right status LED
const int ledPinRL = 49;     // LED pin for Rear Left status LED
const int ledPinRR = 51;     // LED pin for Rear Right status LED

const int pingPinA = 32;         // defines signal pin used by Front Left ping sensor
const int pingPinB = 34;         // defines signal pin used by Front Left ping sensor
const int pingPinC = 36;         // defines signal pin used by Front Left ping sensor

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int buttonStateB = 0;        // variable for reading the second pushbutton status
int var = 0;                 // variable for short loops

////////////////////////////////////////////////////////////////////////
// Prepare PID library
#include <PID_v1.h>   // include the PID library See: http://playground.arduino.cc/Code/PIDLibrary
// Initialize variables related to PID
double PIDsetpoint, PIDinput, PIDoutput;
//resuming project on 7/11/2013 after PID Balance Beam project
PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,.2,0,.05, DIRECT);

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

int pingArray[] = {0, 0, 0};

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
  pinMode(ledPinBin0, OUTPUT);
  pinMode(ledPinBin1, OUTPUT);
  pinMode(ledPinBin2, OUTPUT);
  pinMode(ledPinBin3, OUTPUT);
  pinMode(ledPinFL, OUTPUT);
  pinMode(ledPinFR, OUTPUT);
  pinMode(ledPinRL, OUTPUT);
  pinMode(ledPinRR, OUTPUT);

  ////////////////////////////////////////////////////////////////////////
  // initialize pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(buttonPinB, INPUT);
  
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
  /* moved to calibrateIRarray function
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
  */ //moved to calibrateIRarray function
  
} //end of void setup

////////////////////////////////////////////////////////////////////////
void loop()
////////////////////////////////////////////////////////////////////////
{
  /*
  /////Serial.println ("Starting void loop");
  /////navigationLogicA();     // has 3 conditions: straight, turn left, turn right
  /////navigationLogicB();     // has 7 states with varying motor responses
  /////navigationLogicC();       // uses PID library
  /////navigationLogicD();     // lookup tables, basically
  /////navigationLogicE();     // basic PID (non library) implementation
  /////diagnosticDrive(5); 
  /////PIDTestNoMotors();
  */
  
  // Polyathlon Events
  
  // A) Basic Line Follower
  // B) Advanced Line Follower
  //navigationLogicF();     // uses PID library, after Balance Beam project
  
  // C) Beacon Killer
  // D) Beacon Killer with obstacles
  // E) Navigation by dead reckoning
  //deadReckoning();
  // F) Bulldozer
  bulldozer();
  
  // Diagnostic Functions
  //navigationSensorTest(); // display reflectance sensor array reading
  //DIAGPushButtonsAndLEDs(); // cycle LEDs and check for button pushes
  
} //end of void loop

////////////////////////////////////////////////////////////////////////
/* FUNCTIONS  */
////////////////////////////////////////////////////////////////////////

int DIAGPushButtonsAndLEDs(){
  // flash all LEDs until a button is pushed, then walk left or right depending on button
  buttonState = digitalRead(buttonPin);
  buttonStateB = digitalRead(buttonPinB);
  while (buttonState == HIGH && buttonStateB == HIGH){
    buttonState = digitalRead(buttonPin);
    buttonStateB = digitalRead(buttonPinB);
    digitalWrite(ledPin, HIGH);    // turn on LED (slow blink waiting)
    delay(150);
    digitalWrite(ledPinBin0, HIGH);
    delay(150);
    digitalWrite(ledPinBin1, HIGH);
    delay(150);
    digitalWrite(ledPinBin2, HIGH);
    delay(150);
    digitalWrite(ledPinBin3, HIGH);
    delay(150);
    digitalWrite(ledPinFL, HIGH);
    delay(150);
    digitalWrite(ledPinFR, HIGH);
    delay(150);
    digitalWrite(ledPinRL, HIGH);
    delay(150);
    digitalWrite(ledPinRR, HIGH);
    delay(300);
    digitalWrite(ledPin, LOW);     // turn off LED
    digitalWrite(ledPinBin0, LOW);
    digitalWrite(ledPinBin1, LOW);
    digitalWrite(ledPinBin2, LOW);
    digitalWrite(ledPinBin3, LOW);
    digitalWrite(ledPinFL, LOW);
    digitalWrite(ledPinFR, LOW);
    digitalWrite(ledPinRL, LOW);
    digitalWrite(ledPinRR, LOW);
    delay(300);
  }
  delay(5000);   // a button was pressed to leave the blinking loop, so wait 5 seconds and resume
  
} // close DIAG-PushButtonsAndLEDs function

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

int driveBackward(int speedMotorA, int speedMotorB){
  // function accepts two args: speed for motorA, motorB
  // future args may include specific time to run
  // or distance to cover (for use with countable wheels)
  leftMotor.move(backward, speedMotorA);
  rightMotor.move(backward, speedMotorB);
}

int diagnosticDrive(int secondsToDrive){
  // drive forward for secondsToDrvie seconds
  //motorA is passenger side, motorB is driver side
  driveForward(135,150);
  delay(5000); // hardcoding 5 seconds for now
  allStop();
  delay(50000);
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
int calibrateIRarray(){
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
} // close calibrateIRarray function


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int navigationLogicF(){
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  if (TestRuns == 0){
    calibrateIRarray();
    Serial.println("PID tunings: .2-0-.05");                                         //   <<< Change PID here 2/2 <<<<
    Serial.println("PIDsetpoint LinePosition PIDoutput speedMotorA speedMotorB");
  }  
  
  PIDinput = qtra.readLine(sensorValues);
  myPID.Compute();
  //delay(25); // Provide time for screen scrolling
  Serial.print(PIDsetpoint);
  Serial.print(",");
  Serial.print(PIDinput);
  Serial.print(",");
  Serial.print(PIDoutput);
  Serial.print(",");
  //Serial.println(PIDoutputMapped);
  
  
  if ( PIDoutput < 0 ){
    PIDoutputABS=abs(PIDoutput); // convert to absolute value
    speedMotorB = 200;
    speedMotorA = map(PIDoutputABS, 255, 0, 50, 200);
    Serial.print(speedMotorA);
    Serial.print(",");
    Serial.print(speedMotorB);
    speedMotorB = speedMotorB * .93;
    leftMotor.move(forward, speedMotorA);
    rightMotor.move(forward, speedMotorB);
  }
    if ( PIDoutput > 0 ){
    speedMotorA = 200;
    speedMotorB = map(PIDoutput, 255, 0, 50, 200);
    Serial.print(speedMotorA);
    Serial.print(",");
    Serial.print(speedMotorB);
    speedMotorB = speedMotorB * .93;
    leftMotor.move(forward, speedMotorA);
    rightMotor.move(forward, speedMotorB);
  }
  
/*  
  // Checking for need of advanced navigation
  if (sensorValues[0] > 700 | sensorValues[7] > 700){  // If one of the outside sensors detects black, go into advanced navigation mode
    // proceed straight until all white or 2" -whichever comes first
    // if 2"     
    
  }
*/

  // check for all black
  if (sensorValues[0] > 700 && sensorValues[1] > 700 && sensorValues[2] > 700 && sensorValues[3] > 700 && sensorValues[4] > 700 && sensorValues[5] > 700 && sensorValues[6] > 700 && sensorValues[7] > 700)
  {
    Serial.println(" ");
    Serial.println(" Whoa! Everything looks black!");
    // assume an intersection and proceed for a brief moment. If still all black, stop.
    var=0;
    while(var < 150){
      Serial.println("going forward on black");
      var++;
    }
  }

  // check for sharp right
  if (sensorValues[0] > 700 && sensorValues[1] > 700) {
    Serial.println("probably sharp right");
    PIDinput = 0;
    // proceed 90 degrees (2")
    speedMotorA = 150;
    speedMotorB = 150;
    speedMotorB = speedMotorB * .93;
    leftMotor.move(forward, speedMotorA, 90, coast);
    rightMotor.move(forward, speedMotorB, 90, coast);
    // if all white, rotate right to 2000
    if (sensorValues[0] < 80 && sensorValues[1] < 80 && sensorValues[2] < 80 && sensorValues[3] < 80 && sensorValues[4] < 80 && sensorValues[5] < 80 && sensorValues[6] < 80 && sensorValues[7] < 80){
      while ( PIDinput < 3000 ){    // no need to rotate all the way back to 3500, just get to 2000 and resume PID. Actually, 3000 might be safer.
        speedMotorA = 130; // 120 worked well
        speedMotorB = 130;
        speedMotorB = speedMotorB * .93;
        leftMotor.move(forward, speedMotorA);
        rightMotor.move(backward, speedMotorB);
        PIDinput = qtra.readLine(sensorValues);
      }
    }
  }
    
    /*
    // if this is an intersection, we'll end up on the other side and resume PID
    // if this is a hard right or switchback, the right pivot should get us back on track (or exit quickly if on a good line)
    Serial.println("sharp turn noticed, punch through until white");
    var=0;
    //while(var < 500){  // punch through of 20 too low if battery is low and bot is really slow.
    while (sensorValues[0] > 700 | sensorValues[1] > 700 | sensorValues[2] > 700 | sensorValues[3] > 700 | sensorValues[4] > 700 | sensorValues[5] > 700 | sensorValues[6] > 700 | sensorValues[7] > 700){      Serial.println("punch");
      speedMotorA = 125;
      speedMotorB = 125;
      speedMotorB = speedMotorB * .93;
      leftMotor.move(forward, speedMotorA);
      rightMotor.move(forward, speedMotorB);
      //PIDinput = qtra.readLine(sensorValues); Since this punch was tripped by 3 right sensors, 
      //                                        don't want to continue updating PIDinput or a
      //                                        switchback corner under left sensors might be the last
      //                                        sensor read causing all white recovery to rotate
      //                                        in the wrong direction.
      qtra.readLine(sensorValues);  //read sensors here to update array
      PIDinput = 0; // seed PIDouput so all white will result in right turn      
      var++;
    }
    var=0;
    while(var < 25){
      Serial.println("punch a little more to fully clear the line");
      speedMotorA = 125;
      speedMotorB = 125;
      speedMotorB = speedMotorB * .93;
      leftMotor.move(forward, speedMotorA);
      rightMotor.move(forward, speedMotorB);
      var++;
    }

    // after punch through, start rotating right
    while ( PIDinput < 2000 ){
      speedMotorA = 110; // 120 worked well
      speedMotorB = 110;
      leftMotor.move(forward, speedMotorA);
      rightMotor.move(backward, speedMotorB);
      PIDinput = qtra.readLine(sensorValues);
    }
    */
  
  // check for sharp left
  if (sensorValues[7] > 700 && sensorValues[6] > 700) {
    Serial.println("probably sharp left");
    PIDinput = 7000;
    // proceed 90 degrees (2")
    speedMotorA = 150;
    speedMotorB = 150;
    speedMotorB = speedMotorB * .93;
    leftMotor.move(forward, speedMotorA, 90, coast);
    rightMotor.move(forward, speedMotorB, 90, coast);
    // if all white, rotate left to 5000
    if (sensorValues[0] < 80 && sensorValues[1] < 80 && sensorValues[2] < 80 && sensorValues[3] < 80 && sensorValues[4] < 80 && sensorValues[5] < 80 && sensorValues[6] < 80 && sensorValues[7] < 80){
      while ( PIDinput > 4000 ){    // no need to rotate all the way back to 3500, just get to 5000 and resume PID. Actually, 4000 might be safer.
        speedMotorA = 130; // 120 worked well
        speedMotorB = 130;
        speedMotorB = speedMotorB * .93;
        leftMotor.move(backward, speedMotorA);
        rightMotor.move(forward, speedMotorB);
        PIDinput = qtra.readLine(sensorValues);
      }
    }
  }
    // resume PID
    
    
    
    /*
    Serial.println("sharp left turn noticed, until all white, then punch through just a little more");
    var=0;
    //while(var < 500){ // punch through of 20 too low if battery is low and bot is really slow.   Why not punch through until all white? because the go until white hurts intersection performance
    // ...unless we could set a max punch of 2 inches, then 213
    while (sensorValues[0] > 700 | sensorValues[1] > 700 | sensorValues[2] > 700 | sensorValues[3] > 700 | sensorValues[4] > 700 | sensorValues[5] > 700 | sensorValues[6] > 700 | sensorValues[7] > 700){
      Serial.println("punch");
      speedMotorA = 125;
      speedMotorB = 125;
      speedMotorB = speedMotorB * .93;
      leftMotor.move(forward, speedMotorA);
      rightMotor.move(forward, speedMotorB);
      //PIDinput = qtra.readLine(sensorValues); Since this punch was tripped by 3 left sensors,
      //                                        don't want to continue updating PIDinput or a
      //                                        switchback corner under right sensors might be the last
      //                                        sensor read causing all white recovery to rotate
      //                                        in the wrong direction.
      qtra.readLine(sensorValues);  //read sensors here to update array for white checking
      PIDinput = 7000; // seed PIDouput so all white will result in left turn
      var++;
    }
    var=0;
    while(var < 25){
      Serial.println("punch a little more to fully clear the line before rotating");
      speedMotorA = 125;
      speedMotorB = 125;
      speedMotorB = speedMotorB * .93;
      leftMotor.move(forward, speedMotorA);
      rightMotor.move(forward, speedMotorB);
      var++;
    }
    // after punch through, start rotating left
    while ( PIDinput > 5000 ){
      speedMotorA = 110; // 120 worked well
      speedMotorB = 110;
      speedMotorB = speedMotorB * .93;
      leftMotor.move(backward, speedMotorA);
      rightMotor.move(forward, speedMotorB);
      PIDinput = qtra.readLine(sensorValues);
    }
    */
  
  // check for all white. If the robot is using this, it's a probably a bad sign
  if (sensorValues[0] < 80 && sensorValues[1] < 80 && sensorValues[2] < 80 && sensorValues[3] < 80 && sensorValues[4] < 80 && sensorValues[5] < 80 && sensorValues[6] < 80 && sensorValues[7] < 80)
  {
    Serial.println(" ");
    Serial.println(" Whoa! Everything looks white!");
    // we've lost the line. swing towards last known good location
    if ( PIDinput <= 3500 ) {
      Serial.println("Lost line off to our right. Swing right to find");
      // maybe add a while loop to swing right until PIDinput >5000 to ensure a long enough swing on switchbacks
      speedMotorA = 150; // 120 worked well
      speedMotorB = 150;
      leftMotor.move(forward, speedMotorA);
      rightMotor.move(backward, speedMotorB);
      PIDinput = qtra.readLine(sensorValues);
    }
    if (PIDinput > 3500) {
      Serial.println("Lost line off to our left. Swing left to find");
      speedMotorA = 150;
      speedMotorB = 150;
      leftMotor.move(backward, speedMotorA);
      rightMotor.move(forward, speedMotorB);
      PIDinput = qtra.readLine(sensorValues);
    }
  }
  
  // check for test run length
  TestRuns++;
  if ( TestRuns > 8000 ){
    allStop(); 
    Serial.print(TestRuns);
    Serial.println(" TestRuns complete. Pausing for 2 minutes...");
    Serial.println(" ");
    pause();
    TestRuns = 0;
  }
} // close PIDTestNoMotors function

////////////////////////////////////////////////////////////////////////
int deadReckoning(){
  Serial.println("Entering deadReckoning function");
  pause();
  // Goal: Drive clockwise around an equalateral triangle with 4' sides. Return to same starting point.
  // 360 degrees of wheel resolution should be approximately 7.75” of travel distance
  // 4' = 48”
  // 48” / 7.75” = 6.193548387
  // 6.193548387 * 360 = 2229.677419355
  // 2229.677419355 / 4 = 557.419354839 (break the leg into 4 smaller segments)
  
  // Leg 1
  var=0;
  while(var<1){
    speedMotorA = 150;
    speedMotorB = 150;
    speedMotorB = speedMotorB * .97; // .97 at 150 with new battery worked well
    leftMotor.move(forward, speedMotorA, 2220, brake);
    rightMotor.move(forward, speedMotorB, 2220, brake);
    delay(10000);
    var++;
  }
  Serial.println("pausing...");
  //pause();
  
  // Rotate right 120 degrees
  leftMotor.move(forward, speedMotorA, 230, brake);
  rightMotor.move(backward, speedMotorB, 230, brake);
  delay(3000);
  //pause();
  
  // Leg 2
  var=0;
  while(var<1){
    speedMotorA = 150;
    speedMotorB = 150;
    speedMotorB = speedMotorB * .97; // .97 at 150 with new battery worked well
    leftMotor.move(forward, speedMotorA, 2220, brake);
    rightMotor.move(forward, speedMotorB, 2220, brake);
    delay(10000);
    var++;
  }  
  pause();
  // Rotate right 120 degrees
  leftMotor.move(forward, speedMotorA, 230, brake);
  rightMotor.move(backward, speedMotorB, 230, brake);
  delay(3000);
  //pause();
  
  // Leg 3
  var=0;
  while(var<1){
    speedMotorA = 150;
    speedMotorB = 150;
    speedMotorB = speedMotorB * .97; // .97 at 150 with new battery worked well
    leftMotor.move(forward, speedMotorA, 2220, brake);
    rightMotor.move(forward, speedMotorB, 2220, brake);
    delay(10000);
    var++;
  }
  
  // Rotate right 120 degrees
  leftMotor.move(forward, speedMotorA, 230, brake);
  rightMotor.move(backward, speedMotorB, 230, brake);
  delay(3000);
  //pause();
  
} // close deadReckoning function

////////////////////////////////////////////////////////////////////////
int getPingDistances(){
  long duration, inches, cm;

  pinMode(pingPinA, OUTPUT);
  digitalWrite(pingPinA, LOW);
  delayMicroseconds(2);              // original value: 2
  digitalWrite(pingPinA, HIGH);
  delayMicroseconds(5);              // original value: 5
  digitalWrite(pingPinA, LOW);
  pinMode(pingPinA, INPUT);
  duration = pulseIn(pingPinA, HIGH);
  inches = microsecondsToInches(duration);
  delay(20);                         // Setting too low seemed to introduce issues. Too much ping noise in tube?
  Serial.print("pingPin[A,B,C] inches: ");
  Serial.print(inches);
  
  pinMode(pingPinB, OUTPUT);
  digitalWrite(pingPinB, LOW);
  delayMicroseconds(2);              // original value: 2
  digitalWrite(pingPinB, HIGH);
  delayMicroseconds(5);              // original value: 5
  digitalWrite(pingPinB, LOW);
  pinMode(pingPinB, INPUT);
  duration = pulseIn(pingPinB, HIGH);
  inches = microsecondsToInches(duration);
  delay(20);                         // Setting too low seemed to introduce issues. Too much ping noise in tube?
  Serial.print(" ");
  Serial.print(inches);
  
  pinMode(pingPinC, OUTPUT);
  digitalWrite(pingPinC, LOW);
  delayMicroseconds(2);              // original value: 2
  digitalWrite(pingPinC, HIGH);
  delayMicroseconds(5);              // original value: 5
  digitalWrite(pingPinC, LOW);
  pinMode(pingPinC, INPUT);
  duration = pulseIn(pingPinC, HIGH);
  inches = microsecondsToInches(duration);
  delay(20);                         // Setting too low seemed to introduce issues. Too much ping noise in tube?
  Serial.print(" ");
  Serial.println(inches);

  return inches;  
} // close getPingDistances function

////////////////////////////////////////////////////////////////////////
long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

////////////////////////////////////////////////////////////////////////
long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

int bulldozer(){
    getPingDistances();
} // close bulldozer function
