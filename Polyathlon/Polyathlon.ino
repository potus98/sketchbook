////////////////////////////////////////////////////////////////////////
// by John Christian
//    potus98@yahoo.com
//    potus98.com
//    @potus98 on twitter
//    https://github.com/potus98/sketchbook
//
// Code below is not entirely original. It may have snippets or entire
// sections from other examples or tutorials found online. Thank you
// wonderful Internet for the help. If you see something you feel should
// be specifically credited, please let me know! 
//
// Pololu is awesome. They have great customer service, great prices,
// and great prodoucts. Not affiliated in any way. Just a satisfied customer.
//
// This LineFollower code is finished enough for the stand-alone LineFollower
// version of code. There's still room for more improvements, but future
// development will be focused on the Polyathlon version of code:
// https://github.com/potus98/sketchbook/tree/master/Polyathlon
//
////////////////////////////////////////////////////////////////////////
//
// This example uses a Pololu QTR-8A IR Reflectance Sensor and an 
// Arduino Mega 2560
//
// Refatoring and rebuilding this code for new physical bot.
//
// Code below is broken up via functions in what might appear to be overly
// complex for a basic line follower. However, this code is intended to
// serve as the basis for a polyathlon robot so future flexability is
// important.
//
////////////////////////////////////////////////////////////////////////
//
// TODO
// variablize turning speed
// tighten turns when encountering a 90 degree turn (currently swings a little too wide)
// PD tuning for tighter following on center
// increase max speeds
// remove four way intersection pause
// add 5 second delay start button
// implement global #define debugprint Serial.print

////////////////////////////////////////////////////////////////////////
// Prepare NXTShield
#include <Wire.h>
#include <NXTShield.h>     // https://github.com/TKJElectronics/NXTShield
//Motor1 leftMotor;
//Motor2 rightMotor;

////////////////////////////////////////////////////////////////////////
// Prepare LEDs and pushbuttons
// constants won't change. They're used here to set pin numbers:
const int ledPin =  13;        // NXT shield already has an LED on pin 13, let's use it
const int buttonPinA = 33;     // outer pushbutton pin (pause/play)
const int buttonPinB = 35;     // inner pushbutton pin (mode changer)
// variables will change:
int buttonStateA = 0;          // variable for reading the pushbutton status
int buttonStateB = 0;          // variable for reading the second pushbutton status
//int var = 0;                 // variable for short loops TODO clean-up locally defined vars and uncomment this

////////////////////////////////////////////////////////////////////////
// Prepare Pololu QTR-8A IR Reflectance Array
#include <QTRSensors.h>    // https://github.com/pololu/qtr-sensors-arduino
#define NUM_SENSORS             8     // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4     // average 4 analog samples per sensor reading
#define EMITTER_PIN             27    // emitter is controlled by digital pin 27
QTRSensorsAnalog qtra((unsigned char[]) {8, 9, 10, 11, 12, 13, 14, 15},
NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
unsigned int sensorValuesB[NUM_SENSORS];  // second array use for doublecheck node readings

////////////////////////////////////////////////////////////////////////
// Prepare PD - alternative to using PID library
// http://letsmakerobots.com/node/38550
#define Kp .2 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd .8 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 190 // max speed of the robot
#define leftMaxSpeed 190 // max speed of the robot
#define rightBaseSpeed 170 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 170  // this is the speed at which the motors should spin when the robot is perfectly on the line
int lastError = 0;

////////////////////////////////////////////////////////////////////////
//Prepare Parralax BlueTooth Module RN-42
#include <SoftwareSerial.h>     // TODO is this lib even needed with Mega 2560?
//SoftwareSerial bluetooth(bluetoothTx, bluetoothRx); // uncomment this line for use with Arduino UNO, comment out for Mega 2560
int bluetoothTx = 14;           // TX-O pin of bluetooth (Mega 2560 pin 14)
int bluetoothRx = 15;           // RX-I pin of bluetooth (Mega 2560 pin 15)
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

////////////////////////////////////////////////////////////////////////
// Prepare misc variables
int TestRuns = 0;              // counter used to limit total number of cycles to make data collection easier
int speedMotorA = 0;
int speedMotorB = 0;
int nodeType = 0;
int IRwhite = 200;             // any IR value smaller than this is treated as white
int IRblack = 600;             // any IR value larger than this is treated as black
int turnSpeed = 110;           // running too fast sometimes results in missed IR scans as array sweeps too fast over a line
int pivotSpeed = 150;          // ...or overshoots a target position
int nodeCheckSpeed = 100;      // ...or advances farther than necessary for a second reading
int nodeBump = 125;             // degrees of extra nudge forward after detecting a node to improve node-to-node measurements
int pivotBump = 0;             // was uTurnBump, still needed?
int nodeCheckDelay = 600;      // time to pause and allow bot to stop moving before evaluating a found node

char headingCurrent = 'n';             // direction bot is facing. n, s, e, w (North, South, East, West)
int reconMode = 0;             // maze recon mode 0 = on, 1 = off
int locx = 0;                  // location on x axis
int locy = 0;                  // location on y axis

int leftDegreesCumulative = 0;
int rightDegreesCumulative = 0;
int leftDegreesPrevious = 0;
int rightDegreesPrevious = 0;
int leftDegreesTraveled = 0;
int rightDegreesTraveled = 0;
int avgDegreesTraveled = 0;

float inches = 0;
int wheelCirc = 8;   // wheel circumference in inches

float legPreviousDegrees = 0;
float legPreviousInches = 0;

int nodeLocCurrent[2] = {0, 0};
int nodeLocPrevious[2] = {0, 0};

int degreesToRotate = 0;

////////////////////////////////////////////////////////////////////////
// Setup
void setup()
{
  //Serial.begin(115200);          // initialize serial communication over USB
  //Serial3.begin(115200);           // Begin the serial monitor over BlueTooth. Use with Arduino Mega 2560 w/ pins 14,15
                                // seems to only support 9600 baud rate (??), 19200 is garbled, 115200 is nothing
                                 // TODO baud rates supported by Mega 2560?
  
  Serial.begin(9600);
  bluetooth.begin(115200);
  delay(320);
  bluetooth.print("$");
  bluetooth.print("$");
  bluetooth.print("$");
  delay(100);
  bluetooth.println("U,9600,N");
  bluetooth.begin(9600);

  
  pinMode(ledPin, OUTPUT);       // initialize the digital pin as an output

} // close void setup()


////////////////////////////////////////////////////////////////////////
// Main loop
void loop()
{
  if (TestRuns == 0){
    delay(3000);
    calibrateIRarray();
    pause();
  }  

  //testNXTShield();             // basic test/demo of NXTShield driving two NXT servos
  //testBlueToothSerial();       // basic test/demo of Parallax BlueTooth Module RN-42
  //calibrateIRarray();
  //lineFollowerMode();          // bot acts like a line follower
  //mazeSolverModeRHS();         // bot acts like a maze solver using Right Hand Side algorithm
  //mazeSolverModeLHS();         // bot acts like a maze solver using Left Hand Side algorithm
  //testNXTEncoders();
  //getDegrees();
  /////mazeSolverModeRHSPruning();    // bot acts like a maze solver using Right Hand Side algorithm with loop pruning
  //mazeSolverModeSolved();

  pivot(90,0);
  delay(5000);
  pivot(180,1);
  delay(5000);
  
  //currentLoc();
  //delay(1000);

  // check for test run length
  TestRuns++;
  if ( TestRuns > 80000 ){       // Usually 4000 for testing. Change to big number before competition !!!
    allStop(); 
    delay(5000);
    TestRuns = 0;
  }

} // close void loop()



////////////////////////////////////////////////////////////////////////
/* FUNCTIONS  */
////////////////////////////////////////////////////////////////////////


int testNXTShield(){

  // BACKWARD at full speed for 2 seconds
  Serial.println("BACKWARD for 2 seconds");
  Motor1.move(BACKWARD, 255);
  Motor2.move(BACKWARD, 255);
  delay(2000);

  // FORWARD at full speed for 2 seconds
  Serial.println("FORWARD for 2 seconds");
  Motor1.move(FORWARD, 255);
  Motor2.move(FORWARD, 255);
  delay(2000);

  // Stop both motors for 2 seconds
  Serial.println("Stop for 2 seconds");
  Motor1.stop();
  Motor2.stop();
  delay(2000);

  // Rotate 360 degrees (one resolution) and coast motors
  Serial.println("Coast");
  Motor1.move(BACKWARD, 255, 360, COAST);
  Motor2.move(BACKWARD, 255, 360, COAST);
  while (Motor1.isTurning() || Motor2.isTurning()); // Wait until it has reached the position
  delay(2000);

  // Rotate 360 degrees (one resolution) and brake motors
  Serial.println("Brake");
  Motor1.move(FORWARD, 255, 360, BRAKE);
  Motor2.move(FORWARD, 255, 360, BRAKE);
  while (Motor1.isTurning() || Motor2.isTurning()); // Wait until it has reached the position
  delay(2000);
  
}  // close testNXTShield function

int testNXTEncoders(){
  Serial.print(Motor1.readPosition());
  Serial.print('\t');
  Serial.println(Motor2.readPosition());
  delay(100);  // is this delay necessary to read the positions?
}


////////////////////////////////////////////////////////////////////////
int testIRarray(){
  // read in IR sensor values and print the values on a line
  int i;
  qtra.readLine(sensorValues);
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print(' ');
  }
  Serial.println(' ');
}  // close testIRarray

////////////////////////////////////////////////////////////////////////
int printIRarray(){
  // print current IR values on a line
  int i;
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial3.print(sensorValues[i]);
    Serial.print(' ');
    Serial3.print(' ');
  }
  Serial.println(' ');
  Serial3.println(' ');
}  // close testIRarray

////////////////////////////////////////////////////////////////////////
int testBlueToothSerial(){
  Serial.println("Entering testBlueToothSerial function");
  Serial3.println("Entering testBlueToothSerial function");
  Serial.println("TX test: 0123456789");
  Serial3.println("TX test: 0123456789");
  delay(2000);
}  // close testBlueToothSerial

////////////////////////////////////////////////////////////////////////
int calibrateIRarray(){
    int i;
  
    //Serial.println("Running calibration (see the blinky red light)?");
    bluetooth.println("Running calibration (see the blinky red light)?");
    for (i = 0; i < 100; i++)      // run calibration for a few seconds (reduced from 200 to 100)
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
      bluetooth.print(qtra.calibratedMinimumOn[i]);
      Serial.print(' ');
      bluetooth.print(' ');
    }
    Serial.println();
    bluetooth.println();
    // print the calibration MAXimum values measured when emitters were on
    for (i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtra.calibratedMaximumOn[i]);
      bluetooth.print(qtra.calibratedMaximumOn[i]);
      Serial.print(' ');
      bluetooth.print(' ');
    }
    Serial.println("Leaving calibrateIRarray function");
    bluetooth.println("Leaving calibrateIRarray function");
} // close calibrateIRarray function


////////////////////////////////////////////////////////////////////////
int pause(){
  Serial.println("entering pause function");
  bluetooth.println("entering pause function");
  // standby until momentary button is pressed
  // useful after calibration is complete, but before the event starts
  allStop();
  buttonStateA = digitalRead(buttonPinA);
  // buttsonState is HIGH while untouched, pressed button causes LOW
  while(buttonStateA == HIGH){
    buttonStateA = digitalRead(buttonPinA);
    digitalWrite(ledPin, HIGH);    // turn on LED (slow blink waiting)
    delay(250);
    digitalWrite(ledPin, LOW);     // turn off LED
    delay(250); 
  }
  //Serial.println("Button A pressed! Start 5 second delay...");
  //Serial3.println("Button A pressed! Start 5 second delay...");
  bluetooth.println("Button A pressed! Start 5 second delay...");
  delay(5000);
  //Serial.println("Time's up! Leaving pause loop.");
  //Serial3.println("Time's up! Leaving pause loop.");
  bluetooth.println("Time's up! Leaving pause loop.");
  //Serial.println(" ");
  //Serial3.println(" ");
  bluetooth.println(" ");
}


////////////////////////////////////////////////////////////////////////
int allStop(){                                  // turn off drive motors
  Serial.println("allStop - stopping both motors");
  Motor1.stop();
  Motor2.stop();
}

////////////////////////////////////////////////////////////////////////
int getDegrees(){      // TODO rename to getEncoderTics or something more accurate, meaningful
                        // determine distance from last time this fucntion was called until this time
                        // 'distance' is currently measured in wheel encoder tics (not inches or cm)
                        //
                        // TODO will bot need to track distances on final run? Or just know sequence of valid nodes and run those?
  bluetooth.println("Entering getDegrees function");
  allStop();
  delay(400);
  leftDegreesCumulative = Motor1.readPosition();                          // obtain encoder reading
  rightDegreesCumulative = Motor2.readPosition();                         // obtain encoder reading
  delay(10);                                                              // TODO is this delay necessary to read the positions?
  leftDegreesTraveled = leftDegreesCumulative - leftDegreesPrevious;    // calculate distance traveled
  rightDegreesTraveled = rightDegreesCumulative - rightDegreesPrevious;  // calculate distance traveled
  if (leftDegreesTraveled > 0 && rightDegreesTraveled > 0){
    avgDegreesTraveled = (leftDegreesTraveled + rightDegreesTraveled) / 2;
  }
  if (leftDegreesTraveled == 0 && rightDegreesTraveled == 0){
    avgDegreesTraveled = 0;
  }

  /*
  Serial3.print("Total: ");
  Serial3.print(leftDegreesCumulative);
  Serial3.print(" ");
  Serial3.println(rightDegreesCumulative);
  Serial3.print("Prev: ");
  Serial3.print(leftDegreesPrevious);
  Serial3.print(" ");
  Serial3.println(rightDegreesPrevious);
  Serial3.print("This leg: ");
  Serial3.print(leftDegreesTraveled);
  Serial3.print(" ");
  Serial3.println(rightDegreesTraveled);
  */
  
  bluetooth.print("avgDegreesTraveled: ");  // really, it's avg encoder tics accumulated
  bluetooth.println(avgDegreesTraveled);
  bluetooth.println(" ");
    
  leftDegreesPrevious = leftDegreesCumulative;           // reset starting point for next leg
  rightDegreesPrevious = rightDegreesCumulative;         // reset starting point for next leg
  
  delay(200); // TODO remove debugging delay
  
  return avgDegreesTraveled;
  
} // close getDegrees function

////////////////////////////////////////////////////////////////////////
int lineFollowerMode(){                  // bot acts like a line follower
  
  nodeType = checkForNode();
  
  switch (nodeType) {
    case 3:
      followLine();      // already passed 4-way intersection by this point, so keep going
      break;
    default:
      followLine();
  } // close switch statement
  
} // close lineFollowerMode


////////////////////////////////////////////////////////////////////////
int mazeSolverModeRHS(){      // bot acts like a maze solver using Right Hand Side algorithm
                              // TODO fix this function since turns removed from checkForNode()
  nodeType = checkForNode();

  switch (nodeType) {
    case 0:                   // 0  - no node detected
      followLine();
      break;

    case 1:                   // 1  - four way intersection (bot can turn left, right, or continue straight)
      turnRight(turnSpeed);         //      100 motor speed (130 seemed to be too fast for IR sensors to catch the line)
      break;                  //      270 degrees of rotation (6 inches of wheel travel) would be
                              //      theoretically ideal 90 degree right turn, but reduced to 250 due to extra wheel momentum
    
    case 2:                   // 2  - black square (finish box for maze solving)
      pause();
      break;
    
    case 3:                   // 3  - dead end T intersection (bot can turn left or right)
      turnRight(turnSpeed);         //      100 motor speed (130 seemed to be too fast for IR sensors to catch the line)
      break;                  //      270 degrees of rotation (6 inches of wheel travel) would be
                              //      theoretically ideal, but reduced here due to extra wheel momentum
    
    case 4:                   // 4  - right hand T intersection (bot can turn right or continue straight)
      turnRight(turnSpeed);         //      100 motor speed (130 seemed to be too fast for IR sensors to catch the line)
      break;
    
    case 5:                   // 5  - left hand T intersection (bot can turn left or continue straight)
      followLine();
      break;
    
    case 6:                   // 6  - dead end line (bot must stop or complete a U-turn)
      followLine();           //      bot has already turned by this point
      break;
      
    case 9:                   // 9  - 90 degree right turn (bot can only turn right) need to identify such nodes for building a coordinate grid of a maze
      followLine();           //      bot has already turned by this point
      break;
    
    case 10:                  // 10 - 90 degree left turn (bot can only turn left) need to identify such nodes for building a coordinate grid of a maze
      followLine();           //      bot has already turned by this point
      break;
    
    default:
      followLine();
    
  } // close switch statement
} // close mazeSolverModeRHS

////////////////////////////////////////////////////////////////////////
int mazeSolverModeRHSPruning(){      // bot acts like a maze solver using Right Hand Side algorithm with loop pruning
  
  nodeType = checkForNode();

  switch (nodeType) {
    case 0:                   // 0  - no node detected
      //followLine();
      break;

    case 1:                   // 1  - four way intersection (bot can turn left, right, or continue straight)
      getDegrees();          //      obtain distance reading from previous node
      //turnRight(turnSpeed);
      pivotRight(pivotSpeed);
      updateHeading(0); //      assuming all lines are 90 degrees
      getDegrees();          //      reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      break;
    
    case 2:                   // 2  - black square (finish box for maze solving)
      pause();
      break;
    
    case 3:                   // 3  - dead end T intersection (bot can turn left or right)
      getDegrees();          //      obtain distance reading from previous node
      //turnRight(turnSpeed);
      pivotRight(pivotSpeed);
      updateHeading(0);
      getDegrees();          //      reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      break;

    
    case 4:                   // 4  - right hand T intersection (bot can turn right or continue straight)
      getDegrees();          //      obtain distance reading from previous node
      //turnRight(turnSpeed);
      pivotRight(pivotSpeed);
      updateHeading(0);
      getDegrees();          //      reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      break;
    
    case 5:                   // 5  - left hand T intersection (bot can turn left or continue straight)
      getDegrees();          //      obtain distance reading from previous node
      // RHS so just go straight
      break;
    
    case 6:                   // 6  - dead end line (bot must stop or complete a U-turn)
      getDegrees();          //      obtain distance reading from previous node
      pivotRight(pivotSpeed);
      updateHeading(2);
      getDegrees();          //      reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      break;
      
    case 9:                   // 9  - 90 degree right turn (bot can only turn right) need to identify such nodes for building a coordinate grid of a maze
      getDegrees();          //      obtain distance reading prior to resuming (negate encoder tics accumulated during the turning process
      //turnRight(turnSpeed);
      pivotRight(pivotSpeed);
      updateHeading(0);
      getDegrees();          //      reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      break;
    
    case 10:                  // 10 - 90 degree left turn (bot can only turn left) need to identify such nodes for building a coordinate grid of a maze
      //getDegrees();          //      obtain distance reading prior to resuming (negate encoder tics accumulated during the turning process
      //turnLeft(turnSpeed);

      pause();
      
      /*             FIX this back to basic RHS
      bump();
      delay(250);
      legPreviousInches = degreesToInches(getDegrees());          //  distance measured at this point 
      bluetooth.print("Distance of previous leg: ");
      bluetooth.print(legPreviousInches);
      bluetooth.println(" inches");
      delay(5000);
      pivotLeft(pivotSpeed);
      delay(3000);
      
      updateHeading(1);
      getDegrees();  // reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      */
      break;
    
    default:
      //followLine();
      break;
    
  } // close switch statement
  
  followLine();
  
} // close mazeSolverModeRHSPruning


////////////////////////////////////////////////////////////////////////
int mazeSolverModeSolved(){      // bot acts like a maze solver using Right Hand Side algorithm with loop pruning
  
  nodeType = checkForNode();

  switch (nodeType) {
    case 0:                   // 0  - no node detected
      //followLine();
      break;

    case 1:                   // 1  - four way intersection (bot can turn left, right, or continue straight)
      getDegrees();          //      obtain distance reading from previous node
      //turnRight(turnSpeed);
      pivotRight(pivotSpeed);
      updateHeading(0); //      assuming all lines are 90 degrees
      getDegrees();          //      reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      break;
    
    case 2:                   // 2  - black square (finish box for maze solving)
      pause();
      break;
    
    case 3:                   // 3  - dead end T intersection (bot can turn left or right)
      getDegrees();          //      obtain distance reading from previous node
      //turnRight(turnSpeed);
      pivotRight(pivotSpeed);
      updateHeading(0);
      getDegrees();          //      reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      break;

    
    case 4:                   // 4  - right hand T intersection (bot can turn right or continue straight)
      getDegrees();          //      obtain distance reading from previous node
      //turnRight(turnSpeed);
      pivotRight(pivotSpeed);
      updateHeading(0);
      getDegrees();          //      reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      break;
    
    case 5:                   // 5  - left hand T intersection (bot can turn left or continue straight)
      getDegrees();          //      obtain distance reading from previous node
      // RHS so just go straight
      break;
    
    case 6:                   // 6  - dead end line (bot must stop or complete a U-turn)
      getDegrees();          //      obtain distance reading from previous node
      pivotRight(pivotSpeed);
      updateHeading(2);
      getDegrees();          //      reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      break;
      
    case 9:                   // 9  - 90 degree right turn (bot can only turn right) need to identify such nodes for building a coordinate grid of a maze
      getDegrees();          //      obtain distance reading prior to resuming (negate encoder tics accumulated during the turning process
      //turnRight(turnSpeed);
      pivotRight(pivotSpeed);
      updateHeading(0);
      getDegrees();          //      reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      break;
    
    case 10:                  // 10 - 90 degree left turn (bot can only turn left) need to identify such nodes for building a coordinate grid of a maze
      
      //turnLeft(turnSpeed);
      bump();
      delay(250);
      legPreviousInches = degreesToInches(getDegrees());          //  distance measured at this point 
      currentLoc();
      delay(5000);
      pivotLeft(pivotSpeed);
      getDegrees();          //      obtain distance reading prior to resuming (negate encoder tics accumulated during the turning process
      delay(3000);
      
      updateHeading(1);
      getDegrees();  // reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      break;
    
    default:
      //followLine();
      break;
    
  } // close switch statement
  
  followLine();
  
} // close mazeSolverModeRHSPruning



////////////////////////////////////////////////////////////////////////
int mazeSolverModeLHS(){      // bot acts like a maze solver using Left Hand Side algorithm
                              // TODO fix this function since turns removed from checkForNode()
  nodeType = checkForNode();

  switch (nodeType) {
    case 0:                   // 0  - no node detected
      followLine();
      break;

    case 1:                   // 1  - four way intersection (bot can turn left, right, or continue straight)
      turnLeft(turnSpeed);         //      100 motor speed (130 seemed to be too fast for IR sensors to catch the line)
      break;                  //      270 degrees of rotation (6 inches of wheel travel) would be
                              //      theoretically ideal 90 degree right turn, but reduced to 250 due to extra wheel momentum
    
    case 2:                   // 2  - black square (finish box for maze solving)
      pause();
      break;
    
    case 3:                   // 3  - dead end T intersection (bot can turn left or right)
      turnLeft(turnSpeed);         //      100 motor speed (130 seemed to be too fast for IR sensors to catch the line)
      break;                  //      270 degrees of rotation (6 inches of wheel travel) would be
                              //      theoretically ideal, but reduced here due to extra wheel momentum
    
    case 4:                   // 4  - right hand T intersection (bot can turn right or continue straight)
      followLine();
      break;
    
    case 5:                   // 5  - left hand T intersection (bot can turn left or continue straight)
      turnLeft(turnSpeed);
      break;
    
    case 6:                   // 6  - dead end line (bot must stop or complete a U-turn)
      followLine();           //      bot has already turned by this point
      break;
      
    case 9:                   // 9  - 90 degree right turn (bot can only turn right) need to identify such nodes for building a coordinate grid of a maze
      followLine();           //      bot has already turned by this point
      break;
    
    case 10:                  // 10 - 90 degree left turn (bot can only turn left) need to identify such nodes for building a coordinate grid of a maze
      followLine();           //      bot has already turned by this point
      break;
    
    default:
      followLine();
    
  } // close switch statement
} // close mazeSolverModeRHS


////////////////////////////////////////////////////////////////////////
int followLine(){                  // Using PD calculations instead of PID library
  
  //unsigned int sensors[8]; // delete after confirmed working
  int position = qtra.readLine(sensorValues); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = position - 3500;

  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  
  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive


  speedMotorA = leftMotorSpeed;
  speedMotorB = rightMotorSpeed;
  Motor1.move(BACKWARD, speedMotorA);
  Motor2.move(BACKWARD, speedMotorB);

} // close followLine


////////////////////////////////////////////////////////////////////////
int checkForNode(){
  
  // Evaluate IR sensor array readings to identify potential nodes or intersections.
  // If sensor readings suggest bot has encountered a node, this function may perform
  // manual maneuvers to obtain more information about the current line configuration.
  //
  //
  // This function returns a value to indicate type of node detected.
  // 
  // Returned values and their meaning
  //
  //      ---- need these for line following ----
  //   0  no node detected
  //   1  four way intersection (bot can turn left, right, or continue straight)
  //
  //      ---- need these for maze solving ----
  //   2  black square (finish box for maze solving)
  //   3  dead end T intersection (bot can turn left or right)
  //   4  right hand T intersection (bot can turn right or continue straight)
  //   5  left hand T intersection (bot can turn left or continue straight)
  //   6  dead end line (bot must stop or complete a U-turn)
  //
  //      ---- need these for advanced line following ----
  //   7  TODO  right hand switchback
  //   8  TODO  left hand switchback
  //
  //      ---- need these for more efficient line following (tighter turns),          ----
  //      ---- and more efficient maze solving (tighter turns AND coordinate capture) ----
  //   9  90 degree right turn (bot can only turn right) need to identify such nodes for building a coordinate grid of a maze
  //  10  90 degree left turn (bot can only turn left) need to identify such nodes for building a coordinate grid of a maze
  //
  //
  //   TODO return net x,y changes so dead reckoning algorithms know what this function did to the bot's position
  // 
  //   notes:
  //   only middle four sensors seemed to get tripped by a 90 degree turn when bot was already leaning into the turn
  
  int position = qtra.readLine(sensorValues); // initalize position in this scope
  delay(10);  // allow time to collect readings TODO variablize?

  ////////////////////////////////////////////////
  // check for intersections 1 four way, 2 finish square, or 3 T intersection
  if (sensorValues[1] > IRblack && sensorValues[2] > IRblack && sensorValues[3] > IRblack && sensorValues[4] > IRblack && sensorValues[5] > IRblack && sensorValues[6] > IRblack)
  {
    allStop();
    delay(nodeCheckDelay);  // give bot a moment to come to a stop before taking a second reading
    
    //if (reconMode == 0){
    //  getDegrees();
    //}
    
    qtra.readLine(sensorValuesB);                      // obtain reading after just passing over a line
    delay(10);   //give sensors a moment to read
    
    if ((sensorValuesB[0] < IRwhite && sensorValuesB[7] < IRwhite) && (sensorValuesB[3] > IRblack || sensorValuesB[4] > IRblack)){
      // outer sensors see white and at least one of the middle sensors sees black
        //Serial3.println("ret 1");
        return 1;  
    }
    
    if (sensorValuesB[2] > IRblack && sensorValuesB[3] > IRblack && sensorValuesB[4] > IRblack && sensorValuesB[5] > IRblack) // reduced to middle four sensors
    {      
      reconMode = 1;
      //Serial3.println("ret 2");
      return 2;
    }

    if (sensorValuesB[0] < IRwhite && sensorValuesB[1] < IRwhite && sensorValuesB[2] < IRwhite && sensorValuesB[3] < IRwhite && sensorValuesB[4] < IRwhite && sensorValuesB[5] < IRwhite && sensorValuesB[6] < IRwhite && sensorValuesB[7] < IRwhite)
    {
      //Serial3.println("ret 3");
      return 3;
    }
  } //end of check for intersections 1, 2, and 3

  ////////////////////////////////////////////////
  // check for node 9 and 4 (9 is 90 degree right turn, 4 is right hand T)
  if ((sensorValues[0] > IRblack && sensorValues[1] > IRblack && sensorValues[2] > IRblack) && (sensorValues[6] < IRwhite && sensorValues[7] < IRwhite)) {
    allStop();
    delay(nodeCheckDelay);  // give bot a moment to come to a stop before taking a second reading
    qtra.readLine(sensorValuesB); // take a second reading, but don't overwrite the orignal checkForNode reading
    delay(10);

    // check for all white
    if (sensorValuesB[0] < IRwhite && sensorValuesB[1] < IRwhite && sensorValuesB[2] < IRwhite && sensorValuesB[3] < IRwhite && sensorValuesB[4] < IRwhite && sensorValuesB[5] < IRwhite && sensorValuesB[6] < IRwhite && sensorValuesB[7] < IRwhite){      
      //Serial3.println("ret 9");
      return 9;
    }
    
    // check for 3 way intersection (bot can turn right, or continue straight)
    if ((sensorValuesB[0] < IRwhite && sensorValuesB[7] < IRwhite) && (sensorValuesB[3] > IRblack || sensorValuesB[4] > IRblack)){
      // outer sensors see white and at least one of the middle sensors sees black        
      //Serial3.println("ret 4");
      return 4;
    }
  }  // close check for node 9 and 4
  
  ////////////////////////////////////////////////
  // check for node 10 and 5 (10 is a 90 degree left turn, 5 is right left hand T)
  if ((sensorValues[0] < IRwhite && sensorValues[1] < IRwhite) && (sensorValues[5] > IRblack && sensorValues[6] > IRblack && sensorValues[7] > IRblack)) {
    allStop();
    delay(nodeCheckDelay);  // give bot a moment to come to a stop before taking a second reading
    qtra.readLine(sensorValuesB);
    delay(10);
    
    // check for all white
    if (sensorValuesB[0] < IRwhite && sensorValuesB[1] < IRwhite && sensorValuesB[2] < IRwhite && sensorValuesB[3] < IRwhite && sensorValuesB[4] < IRwhite && sensorValuesB[5] < IRwhite && sensorValuesB[6] < IRwhite && sensorValuesB[7] < IRwhite){
      //Serial3.println("ret 10");
      return 10;
    }
    
    // check for 3 way intersection (bot can turn left, or continue straight)
    if ((sensorValuesB[0] < IRwhite && sensorValuesB[7] < IRwhite) && (sensorValuesB[3] > IRblack || sensorValuesB[4] > IRblack)){
      // outer sensors see white and at least one of the middle sensors sees black
      //Serial3.println("ret 5");
      return 5;
    }
  }  // close check for node 10 and 5

  ////////////////////////////////////////////////
  // check for node 6 (dead end)
  // I was reading a line and now *BAM* no line whatsoever. Must be a dead end.
  if (sensorValues[0] < IRwhite && sensorValues[1] < IRwhite && sensorValues[2] < IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite && sensorValues[5] < IRwhite && sensorValues[6] < IRwhite && sensorValues[7] < IRwhite){    
    allStop();
    delay(nodeCheckDelay);  // give bot a moment to come to a stop before taking a second reading
    //Serial3.println("ret 6");
    return 6;
  }

  // no known nodes detected
  return 0;
} // close checkForNode() function


////////////////////////////////////////////////////////////////////////
int turnRight(int speed){
  //Serial3.println("Entering turnRight function");
  // robot turns right by pivoting on the right wheel (only running left wheel)

  // bot might be on a current line so get off the current line
  while (sensorValues[4] > IRblack || sensorValues[5] > IRblack || sensorValues[6] > IRblack || sensorValues[7] > IRblack){
    Motor1.move(BACKWARD, speed);
    qtra.readLine(sensorValues);
    delay(10);  // give a chance for sensors to finish reading?
  }
  
  // ...then keep turning right while all sensors see white until a line is detected
  while (sensorValues[0] < IRwhite && sensorValues[1] < IRwhite && sensorValues[2] < IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite && sensorValues[5] < IRwhite && sensorValues[6] < IRwhite && sensorValues[7] < IRwhite){
    Motor1.move(BACKWARD, speed);
    qtra.readLine(sensorValues);
    delay(10);  // give a chance for sensors to finish reading?
  }
} // close turnRight function


////////////////////////////////////////////////////////////////////////
int turnLeft(int speed){               // robot turns left by pivoting on the left wheel (only running right wheel)
  //Serial3.println("Entering turnLeft function");
  // robot turns left by pivoting on the left wheel (only running right wheel)

  // bot might be on a current line so get off the current line
  while (sensorValues[0] > IRblack || sensorValues[1] > IRblack || sensorValues[2] > IRblack || sensorValues[3] > IRblack){
    Motor2.move(BACKWARD, speed);
    qtra.readLine(sensorValues);
    delay(10);  // give a chance for sensors to finish reading
  }
  
  // ...then keep turning left while all sensors see white until a line is detected
  while (sensorValues[0] < IRwhite && sensorValues[1] < IRwhite && sensorValues[2] < IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite && sensorValues[5] < IRwhite && sensorValues[6] < IRwhite && sensorValues[7] < IRwhite){
    Motor2.move(BACKWARD, speed);
    qtra.readLine(sensorValues);
    delay(10);  // give a chance for sensors to finish reading
  }
} // close turnLeft function


////////////////////////////////////////////////////////////////////////
int pivotRight(int speed){                         // robot rotates by pivoting in place (turning both wheels in opposite directions)

  //Serial3.println("pivot right");
  bluetooth.println("pivot right");
  // move straight forward a bit to provide more room to resume line following after completing the U-Turn manuever
  pivotBump = (nodeBump - 20); // seems bot runs a little farther while evaluating if statements above before reaching this point (???) TODO remove this?
  Motor1.move(BACKWARD, nodeCheckSpeed, pivotBump, BRAKE);  // TODO move nodeBump to separate function since maze solver will use, but line follower may not
  Motor2.move(BACKWARD, nodeCheckSpeed, pivotBump, BRAKE);
  delay(nodeCheckDelay); // allow previous maneuver to complete

  // bot might be on a current line so get off the current line
  while (sensorValues[2] > IRblack || sensorValues[3] > IRblack || sensorValues[4] > IRblack || sensorValues[5] > IRblack || sensorValues[6] > IRblack || sensorValues[7] > IRblack){
    Motor1.move(BACKWARD, speed);
    Motor2.move(FORWARD, speed);
    qtra.readLine(sensorValues);
    delay(10);  // give a chance for sensors to finish reading?
  }
  
  // continue rotating until line reached
  //while (sensorValues[0] < IRwhite && sensorValues[1] < IRwhite && sensorValues[2] < IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite && sensorValues[5] < IRwhite && sensorValues[6] < IRwhite && sensorValues[7] < IRwhite){
  //while (sensorValues[1] < IRwhite && sensorValues[2] < IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite && sensorValues[5] < IRwhite && sensorValues[6] < IRwhite){
  while (IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite && sensorValues[5] < IRwhite && sensorValues[6] < IRwhite){
    Motor1.move(BACKWARD, speed);
    Motor2.move(FORWARD, speed);
    qtra.readLine(sensorValues);
    delay(10);  // give a chance for sensors to finish reading?
  }
} // close pivotRight function

////////////////////////////////////////////////////////////////////////
int pivotLeft(int speed){                         // robot rotates by pivoting in place (turning both wheels in opposite directions)

  bluetooth.println("pivot left");

  // bot might be on a current line so get off the current line
  while (sensorValues[0] > IRblack || sensorValues[1] > IRblack || sensorValues[2] > IRblack || sensorValues[3] > IRblack || sensorValues[4] > IRblack || sensorValues[5] > IRblack){
    Motor1.move(FORWARD, speed);
    Motor2.move(BACKWARD, speed);
    qtra.readLine(sensorValues);
    delay(10);  // give a chance for sensors to finish reading?
  }
  
  // continue rotating until line reached
  //while (sensorValues[0] < IRwhite && sensorValues[1] < IRwhite && sensorValues[2] < IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite && sensorValues[5] < IRwhite && sensorValues[6] < IRwhite && sensorValues[7] < IRwhite){
  //while (sensorValues[1] < IRwhite && sensorValues[2] < IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite && sensorValues[5] < IRwhite && sensorValues[6] < IRwhite){
  while (sensorValues[1] < IRwhite && sensorValues[2] < IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite){
    Motor1.move(FORWARD, speed);
    Motor2.move(BACKWARD, speed);
    qtra.readLine(sensorValues);
    delay(10);  // give a chance for sensors to finish reading?
  }
} // close pivotLeft function

////////////////////////////////////////////////////////////////////////
/// TODO probably don't need this function
int pivot(int pivotDegrees, int pivotDirection){
  allStop();
  bluetooth.println("entering pivotDegrees function");
  bluetooth.print("pivotDegrees ");
  bluetooth.println(pivotDegrees);
  bluetooth.print("pivotDirection ");
  bluetooth.println(pivotDirection);  

  // robot rotates by pivoting in place (turning both wheels in opposite directions)
  // robot should rotate in place counter clockwise the specified number of degrees
  //
  // pivotDegrees is an integer between 0 and 360
  // pivotDirection is either 0 (right) or 1 (left)
  // 
  // wheelbase on my bot is 3.5 inches from centerline of one wheel to the other.
  // ...which means a rotating circumference of about 11 inches.
  // ...my wheels have an 8 inch circumference, or 45 degrees per inch
  // ...so both wheels will need to rotate (45 * 11) 495 degrees for the bot itself to pivot 360
  // ...in theory.

  // convert requested degrees to rotate into degrees of rotation needed by the motors

  //degreesToRotate = map(pivotDegrees, 0, 360, 0, 495);
  // reducing re-map a little to compensate for momentum but allow turns to be specified 
  // at logical (to the user) values. Re-map value arrived at via trial-and-error
  degreesToRotate = map(pivotDegrees, 0, 360, 0, 425);

  bluetooth.print("remapped degreesToRotate ");
  bluetooth.println(degreesToRotate);

  if (pivotDirection == 0)
  {
    Motor1.move(BACKWARD, pivotSpeed, degreesToRotate, BRAKE);
    Motor2.move(FORWARD, pivotSpeed, degreesToRotate, BRAKE);
  }
  if (pivotDirection == 1)
  {
    Motor1.move(FORWARD, pivotSpeed, degreesToRotate, BRAKE);
    Motor2.move(BACKWARD, pivotSpeed, degreesToRotate, BRAKE);  
  }
  
  delay(3000); // TODO remove this delay?
 
} // close pivotLeft function


int bump(){
  // Compensate for bot stopping when line detected.
  // We want to measure distances between node at a point beneath the rear axel / pivot point.
  // But the bot stops immedately upon detecting a line (which is good).
  // This bump is intended to nudge the robot forward so the node is directly beneath the rear axel.
  
  // move straight forward a bit to improve node-to-node measurments...
  // and provide more room to resume line following after completing the U-Turn manuever
  //pivotBump = (nodeBump - 20); // seems bot runs a little farther while evaluating if statements above before reaching this point (???)
  Motor1.move(BACKWARD, nodeCheckSpeed, nodeBump, BRAKE);  // TODO move nodeBump to separate function since maze solver will use, but line follower may not
  Motor2.move(BACKWARD, nodeCheckSpeed, nodeBump, BRAKE);
  delay(nodeCheckDelay); // allow previous maneuver to complete
}

////////////////////////////////////////////////////////////////////////
int updateHeading(int pivotType){                      //  TODO probably won's use this approach/function
  // pivotType can be one of "right" "left" or "uTurn"
  switch (pivotType) {
    case 0:
      bluetooth.println("change heading right");
      delay(10);
      break;

    case 1:
      bluetooth.println("change heading left");
      delay(10);
      break;

    case 2:
      bluetooth.println("change heading uTurn");
      delay(10);
      break;
  }
}  // close updateHeading function

float degreesToInches(int degrees){
  bluetooth.print("degrees coming into degreesToInches: ");
  bluetooth.println(degrees);
  bluetooth.print("wheelCirc: ");
  bluetooth.println(wheelCirc);
  // convert wheel encoder tics to inches based on 8 inch circumference wheel
  inches = (degrees / 360.0) * wheelCirc;
  bluetooth.print("inches: ");
  bluetooth.println(inches);
  return inches;
}  // close degreesToInches

int currentLoc(){
  bluetooth.println("entering currentLoc function");
  bluetooth.print("nodeLocPrevious ");
  int i; for (i = 0; i < 2; i++) { bluetooth.print(nodeLocPrevious[i]);bluetooth.print(" ");} bluetooth.println(" ");
  bluetooth.print("legPreviousInches: ");
  bluetooth.println(legPreviousInches, 4);
  bluetooth.print("heading: ");
  bluetooth.println(headingCurrent);
  
}  // close currentLoc

////////////////////////////////////////////////////////////////////////
int checkForSwitchback(){
  // 
}

////////////////////////////////////////////////////////////////////////
int checkForEdge(){
  // check for edge of table
}



