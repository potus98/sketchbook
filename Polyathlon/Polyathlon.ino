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
#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 170 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 170  // this is the speed at which the motors should spin when the robot is perfectly on the line
int lastError = 0;

////////////////////////////////////////////////////////////////////////
//Prepare Parralax BlueTooth Module RN-42
#include <SoftwareSerial.h>     // TODO is this lib even needed with Mega 2560?
//SoftwareSerial bluetooth(bluetoothTx, bluetoothRx); // uncomment this line for use with Arduino UNO, comment out for Mega 2560
int bluetoothTx = 14;           // TX-O pin of bluetooth (Mega 2560 pin 14)
int bluetoothRx = 15;           // RX-I pin of bluetooth (Mega 2560 pin 15)

////////////////////////////////////////////////////////////////////////
// Prepare misc variables
int TestRuns = 0;              // counter used to limit total number of cycles to make data collection easier
int speedMotorA = 0;
int speedMotorB = 0;
int nodeType = 0;
int IRwhite = 200;             // any IR value smaller than this is treated as white
int IRblack = 600;             // any IR value larger than this is treated as black
int turnSpeed = 100;           // running too fast sometimes results in missed IR scans as array sweeps too fast over a line
int pivotSpeed = 100;          // ...or overshoots a target position
int nodeCheckSpeed = 100;      // ...or advances farther than necessary for a second reading
char facing = 'N';             // direction bot is facing. N, S, E, W (North, South, East, West)
int leftDistanceCumulative = 0;
int rightDistanceCumulative = 0;
int leftDistancePrevious = 0;
int rightDistancePrevious = 0;
int leftDistanceTraveled = 0;
int rightDistanceTraveled = 0;
int avgDistanceTraveled = 0;
int reconMode = 0;             // maze recon mode 0 = on, 1 = off
int nodeBump = 120;             // extra nudge forward after detecting a node to improve node-to-node measurements
int uTurnBump = 0;

////////////////////////////////////////////////////////////////////////
// Setup
void setup()
{
  //Serial.begin(115200);          // initialize serial communication over USB
  Serial3.begin(115200);           // Begin the serial monitor over BlueTooth. Use with Arduino Mega 2560 w/ pins 14,15
                                 // seems to only support 9600 baud rate (??), 19200 is garbled, 115200 is nothing
                                 // TODO baud rates supported by Mega 2560?
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
  mazeSolverModeRHSPruning();    // bot acts like a maze solver using Right Hand Side algorithm with loop pruning
  //testNXTEncoders();
  //getDistance();

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
  
    Serial.println("Running calibration (see the blinky red light)?");
    Serial3.println("Running calibration (see the blinky red light)?");
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
      Serial3.print(qtra.calibratedMinimumOn[i]);
      Serial.print(' ');
      Serial3.print(' ');
    }
    Serial.println();
    Serial3.println();
    // print the calibration MAXimum values measured when emitters were on
    for (i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtra.calibratedMaximumOn[i]);
      Serial3.print(qtra.calibratedMaximumOn[i]);
      Serial.print(' ');
      Serial3.print(' ');
    }
    Serial.println("Leaving calibrateIRarray function");
    Serial3.println("Leaving calibrateIRarray function");
} // close calibrateIRarray function


////////////////////////////////////////////////////////////////////////
int pause(){
  Serial.println("entering pause function");
  Serial3.println("entering pause function");
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
  Serial.println("Button A pressed! Start 5 second delay...");
  Serial3.println("Button A pressed! Start 5 second delay...");
  delay(5000);
  Serial.println("Time's up! Leaving pause loop.");
  Serial3.println("Time's up! Leaving pause loop.");
  Serial.println(" ");
  Serial3.println(" ");
}


////////////////////////////////////////////////////////////////////////
int allStop(){                                  // turn off drive motors
  Serial.println("allStop - stopping both motors");
  Motor1.stop();
  Motor2.stop();
}

////////////////////////////////////////////////////////////////////////
int getDistance(){      // determine distance from last time this fucntion was called until this time
                        // 'distance' is currently measured in wheel encoder tics (not inches or cm)
                        //
                        // TODO will bot need to track distances on final run? Or just know sequence of valid nodes and run those?

  allStop();
  delay(500);
  leftDistanceCumulative = Motor1.readPosition();                          // obtain encoder reading
  rightDistanceCumulative = Motor2.readPosition();                         // obtain encoder reading
  delay(100);                                                              // TODO is this delay necessary to read the positions?
  leftDistanceTraveled = leftDistanceCumulative - leftDistancePrevious;    // calculate distance traveled
  rightDistanceTraveled = rightDistanceCumulative - rightDistancePrevious;  // calculate distance traveled
  if (leftDistanceTraveled > 0 && rightDistanceTraveled > 0){
    avgDistanceTraveled = (leftDistanceTraveled + rightDistanceTraveled) / 2;
  }
  if (leftDistanceTraveled == 0 && rightDistanceTraveled == 0){
    avgDistanceTraveled = 0;
  }

  /*
  Serial3.print("Total: ");
  Serial3.print(leftDistanceCumulative);
  Serial3.print(" ");
  Serial3.println(rightDistanceCumulative);
  Serial3.print("Prev: ");
  Serial3.print(leftDistancePrevious);
  Serial3.print(" ");
  Serial3.println(rightDistancePrevious);
  Serial3.print("This leg: ");
  Serial3.print(leftDistanceTraveled);
  Serial3.print(" ");
  Serial3.println(rightDistanceTraveled);
  */
  
  Serial3.print("Distance: ");
  Serial3.println(avgDistanceTraveled);
  Serial3.println(" ");
    
  leftDistancePrevious = leftDistanceCumulative;           // reset starting point for next leg
  rightDistancePrevious = rightDistanceCumulative;         // reset starting point for next leg
  
  delay(1000); // TODO remove debugging delay
  
  return avgDistanceTraveled;
  
} // close getDistance function

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
      getDistance();          //      obtain distance reading from previous node
      //turnRight(turnSpeed);
      pivotRight(pivotSpeed);
      getDistance();          //      reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      break;
    
    case 2:                   // 2  - black square (finish box for maze solving)
      pause();
      break;
    
    case 3:                   // 3  - dead end T intersection (bot can turn left or right)
      getDistance();          //      obtain distance reading from previous node
      //turnRight(turnSpeed);
      pivotRight(pivotSpeed);
      getDistance();          //      reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      break;

    
    case 4:                   // 4  - right hand T intersection (bot can turn right or continue straight)
      getDistance();          //      obtain distance reading from previous node
      //turnRight(turnSpeed);
      pivotRight(pivotSpeed);
      getDistance();          //      reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      break;
    
    case 5:                   // 5  - left hand T intersection (bot can turn left or continue straight)
      getDistance();          //      obtain distance reading from previous node
      // RHS so just go straight
      break;
    
    case 6:                   // 6  - dead end line (bot must stop or complete a U-turn)
      getDistance();          //      obtain distance reading from previous node
      pivotRight(pivotSpeed);
      getDistance();          //      reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      break;
      
    case 9:                   // 9  - 90 degree right turn (bot can only turn right) need to identify such nodes for building a coordinate grid of a maze
      getDistance();          //      obtain distance reading prior to resuming (negate encoder tics accumulated during the turning process
      //turnRight(turnSpeed);
      pivotRight(pivotSpeed);
      getDistance();          //      reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
      break;
    
    case 10:                  // 10 - 90 degree left turn (bot can only turn left) need to identify such nodes for building a coordinate grid of a maze
      getDistance();          //      obtain distance reading prior to resuming (negate encoder tics accumulated during the turning process
      //turnLeft(turnSpeed);
      pivotLeft(pivotSpeed);
      getDistance();          //      reset measurements for distance measurement to next node (negate the encoder tics accumulated during the turning process)
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
  //Serial3.println("checkForNode");  // TODO remove temp debugging
  
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

  // take an IR reading
  //Serial.println("taking an IR reading");
  //qtra.readLine(sensorValues);
  //Serial.println("Entering checkForNode function");
  //Serial3.println("Entering checkForNode function");
  
  
  int position = qtra.readLine(sensorValues); // initalize position in this scope
  delay(10);  // allow time to collect readings TODO variablize?


  // print the IR values
  //printIRarray();

  ////////////////////////////////////////////////
  // check for intersections 1, 2, or 3
  if (sensorValues[1] > IRblack && sensorValues[2] > IRblack && sensorValues[3] > IRblack && sensorValues[4] > IRblack && sensorValues[5] > IRblack && sensorValues[6] > IRblack)
  {
    // If reading black across the 6 middle QTR-8A sensors, we might be facing one of three things
    // four way intersection (return 1)
    // black finish square (return 2)
    // dead end T intersection (return 3)
    //

    // TODO actually, don't think this extra bump is necessary, not sure, keep the nudge for now
    //Motor1.move(BACKWARD, nodeCheckSpeed, 25, BRAKE);
    //Motor2.move(BACKWARD, nodeCheckSpeed, 25, BRAKE);
    //delay(350);  // give previous movement a chance to finish
    
    // This implementation is basic since it requires careful control of distance moved between each IR reading
    // TODO find a method that can track IR readings across time (actually, across distances via wheel encoders)
    // in order to avoid having to stop the bot to identify intersection types.
    //
    //Serial.println("Intersection?");
    //Serial3.println("Intersection?");
    
    allStop();
    //delay(250);
    
    // if in recon mode, stop and take a good distance measurement before any more wheel movement
    //if (reconMode == 0){
    //  getDistance();
    //}

    
    // proceed 1" (8" wheel circumference = 45 degrees per inch) TODO - Actually, don't think this is necessary:
    // ...by the time the bot stops above, it has already advanced 1 to 1.5 inches beyond the original line that tripped these checks.
    //Motor1.move(BACKWARD, 100, 35, BRAKE);
    //Motor2.move(BACKWARD, 100, 35, BRAKE);

    
    //while (Motor1.isTurning() || Motor2.isTurning()); // Wait until it has reached the position
    delay(1000);  // give bot a moment to come to a stop before taking a second reading
    qtra.readLine(sensorValuesB);                      // obtain reading after just passing over a line
    delay(20);   //give sensors a moment to read
    
    //Serial.println(" - check for 4-way intersection");
    //Serial3.println(" - check for 4-way intersection");
    if ((sensorValuesB[0] < IRwhite && sensorValuesB[7] < IRwhite) && (sensorValuesB[3] > IRblack || sensorValuesB[4] > IRblack)){
      // outer sensors see white and at least one of the middle sensors sees black
        //Serial.println(" - Yep, it's a 4-way intersection");
        //Serial3.println(" - Yep, it's a 4-way intersection");

        /*
        // TODO actually, not sure, keep the nudge for now for better node-to-node measurements
        Motor1.move(BACKWARD, nodeCheckSpeed, nodeBump, BRAKE);                                              // TODO variablize nudge distance (degrees of rotation)
        Motor2.move(BACKWARD, nodeCheckSpeed, nodeBump, BRAKE);
        delay(500);  // give previous movement a chance to finish
        */

        Serial3.println("ret 1");
        return 1;  
    }
    //Serial.println(" - Well, it's not a 4-way intersection, check for finish square");
    //Serial3.println(" - Well, it's not a 4-way intersection, check for finish square");
    //if (sensorValues[1] > IRblack && sensorValues[2] > IRblack && sensorValues[3] > IRblack && sensorValues[4] > IRblack && sensorValues[5] > IRblack && sensorValues[6] > IRblack)
    if (sensorValuesB[2] > IRblack && sensorValuesB[3] > IRblack && sensorValuesB[4] > IRblack && sensorValuesB[5] > IRblack) // reduced to middle four sensors
    {
      //Serial.println(" - Yep, it's a finish square");
      //Serial3.println(" - Yep, it's a finish square");
      //pause();  // TODO just return 2 and let the maze function do the pause()
      
      reconMode = 1;
      Serial3.println("ret 2");
      return 2;
    }

    //Serial.println(" - check for dead end T intersection");
    //Serial3.println(" - check for dead end T intersection");
    if (sensorValuesB[0] < IRwhite && sensorValuesB[1] < IRwhite && sensorValuesB[2] < IRwhite && sensorValuesB[3] < IRwhite && sensorValuesB[4] < IRwhite && sensorValuesB[5] < IRwhite && sensorValuesB[6] < IRwhite && sensorValuesB[7] < IRwhite)
    {
      //Serial.println(" - Yep, it's a dead end T intersection");
      //Serial3.println(" - Yep, it's a dead end T intersection");

      /*
      // TODO actually, not sure, keep the nudge for now for better node-to-node measurements
      Motor1.move(BACKWARD, nodeCheckSpeed, nodeBump, BRAKE);
      Motor2.move(BACKWARD, nodeCheckSpeed, nodeBump, BRAKE);
      delay(500);  // give previous movement a chance to finish
      */
      Serial3.println("ret 3");
      return 3;
    }

  } //end of check for intersections 1, 2, and 3

  ////////////////////////////////////////////////
  // check for node 9 and 4 (9 is 90 degree right turn, 4 is right hand T)
  //if ((sensorValues[0] > IRblack && sensorValues[1] > IRblack) && (sensorValues[6] < IRwhite && sensorValues[7] < IRwhite)) {
  if ((sensorValues[0] > IRblack && sensorValues[1] > IRblack && sensorValues[2] > IRblack) && (sensorValues[6] < IRwhite && sensorValues[7] < IRwhite)) {

    allStop();
    
    // sharp right, or 3 way right?
    //Serial.println("Sharp right or 3 way right?");
    //Serial3.println("Sharp right or 3 way right?");
    // proceed about an inch

    /*
    // TODO actually not sure, keep the nudge for now
    Motor1.move(BACKWARD, nodeCheckSpeed, nodeBump, BRAKE);
    Motor2.move(BACKWARD, nodeCheckSpeed, nodeBump, BRAKE);
    delay(500);  // give previous movement a chance to finish
    */
    
    //Serial.println(" - check again for sharp right or 3 way right");
    //Serial3.println(" - check again for sharp right or 3 way right");
    //position = qtra.readLine(sensorValuesB); // take a second reading, but don't overwrite the orignal checkForNode reading
    qtra.readLine(sensorValuesB); // take a second reading, but don't overwrite the orignal checkForNode reading
    delay(20);

    
    // check for all white
    if (sensorValuesB[0] < IRwhite && sensorValuesB[1] < IRwhite && sensorValuesB[2] < IRwhite && sensorValuesB[3] < IRwhite && sensorValuesB[4] < IRwhite && sensorValuesB[5] < IRwhite && sensorValuesB[6] < IRwhite && sensorValuesB[7] < IRwhite){
      // if all white here, rotate right until seeing line again and return 9
      //Serial.println(" - Yep, it's a sharp right. Rotate right and find line");
      //Serial3.println(" - Yep, it's a sharp right. Rotate right and find line");
      
      /*
      if (reconMode == 0){  // not sure if this should go before or after the extra inch bump
        getDistance();
      }
      */

      /*
      while ( position < 1000 ){    // no need to rotate all the way back to 3500, just get to 1000 and resume PD
        Motor1.move(BACKWARD, turnSpeed);
        //Motor2.move(FORWARD, 130); // comment out to just pivot, need to coordinate with "proceed 2 inches" distance above
        position = qtra.readLine(sensorValues);
      }      
      allStop();    // maybe keep allstop to reduce overshoot
      //delay(2000);  // debugging, remove later
      */
      
      Serial3.println("ret 9");
      return 9;
    }
    
    // check for 3 way intersection (bot can turn right, or continue straight)
    if ((sensorValuesB[0] < IRwhite && sensorValuesB[7] < IRwhite) && (sensorValuesB[3] > IRblack || sensorValuesB[4] > IRblack)){
      // outer sensors see white and at least one of the middle sensors sees black
        //Serial.println(" - Right-handed 3-way intersection");
        //Serial3.println(" - Right-handed 3-way intersection");

        /*
        // TODO actually not sure, keep the nudge for now
        Motor1.move(BACKWARD, nodeCheckSpeed, nodeBump, BRAKE);
        Motor2.move(BACKWARD, nodeCheckSpeed, nodeBump, BRAKE);
        delay(500);  // give previous movement a chance to finish
        */
        
        Serial3.println("ret 4");
        return 4;
    }
  }  // close check for node 9 and 4
  
  ////////////////////////////////////////////////
  // check for node 10 and 5
  //if ((sensorValues[0] < IRwhite && sensorValues[1] < IRwhite) && (sensorValues[6] > IRblack && sensorValues[7] > IRblack)) {
  if ((sensorValues[0] < IRwhite && sensorValues[1] < IRwhite) && (sensorValues[5] > IRblack && sensorValues[6] > IRblack && sensorValues[7] > IRblack)) {

    allStop();
    
    //Serial.println("Sharp left or 3 way left?");
    //Serial3.println("Sharp left or 3 way left?");
    // proceed 90 degrees (2")

    /*
    // TODO actually not sure, keep the nudge for now
    Motor1.move(BACKWARD, nodeCheckSpeed, nodeBump, BRAKE);
    Motor2.move(BACKWARD, nodeCheckSpeed, nodeBump, BRAKE);
    delay(500);  // give previous movement a chance to finish
    */
    
    //Serial.println(" - check again for sharp left or 3 way left");
    //Serial3.println(" - check again for sharp left or 3 way left");
      
    //position = qtra.readLine(sensorValues);
    qtra.readLine(sensorValuesB);
    delay(20);
    
    // check for all white
    if (sensorValuesB[0] < IRwhite && sensorValuesB[1] < IRwhite && sensorValuesB[2] < IRwhite && sensorValuesB[3] < IRwhite && sensorValuesB[4] < IRwhite && sensorValuesB[5] < IRwhite && sensorValuesB[6] < IRwhite && sensorValuesB[7] < IRwhite){
      // if all white here, rotate left until seeing line again and return 10
      //Serial.println(" - Yep, it's a sharp left. Rotate left and find line");
      //Serial3.println(" - Yep, it's a sharp left. Rotate left and find line");

      /*
      while ( position > 5000 ){    // no need to rotate all the way back to 3500, just get to 5000 and resume PD
        //Motor1.move(FORWARD, 130); // comment out to just pivot, need to coordinate with "proceed 2 inches" distance above
        Motor2.move(BACKWARD, turnSpeed);
        position = qtra.readLine(sensorValues);
      }
      allStop();    // maybe keep allstop to reduce overshoot
      */
      
      //delay(2000);  // debugging, remove later
      Serial3.println("ret 10");
      return 10;
    }
    
    // check for 3 way intersection (bot can turn left, or continue straight)
    if ((sensorValuesB[0] < IRwhite && sensorValuesB[7] < IRwhite) && (sensorValuesB[3] > IRblack || sensorValuesB[4] > IRblack)){
      // outer sensors see white and at least one of the middle sensors sees black
        //Serial.println(" - Left-handed 3-way intersection");
        //Serial3.println(" - Left-handed 3-way intersection");
        Serial3.println("ret 5");
        return 5;
    }
  }  // close check for node 10 and 5

  ////////////////////////////////////////////////
  // check for node 6 (dead end)
  // I was reading a line and now *BAM* no line whatsoever. Must be a dead end.
  if (sensorValues[0] < IRwhite && sensorValues[1] < IRwhite && sensorValues[2] < IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite && sensorValues[5] < IRwhite && sensorValues[6] < IRwhite && sensorValues[7] < IRwhite){
    
    allStop();

    /*
    uTurn(uTurnSpeed);  // TODO variablize turn speeds, 130 seemed to be too fast for IR sensors to catch the line
    */
    Serial3.println("ret 6");
    return 6;
  }

  // no known nodes detected
  //Serial.println(" No nodes detected, returning 0 (zero)");
  //Serial3.println(" No nodes detected, returning 0 (zero)");
  //Serial3.println("ret 0");
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
    delay(100);  // give a chance for sensors to finish reading?
  }
  
  // ...then keep turning right while all sensors see white until a line is detected
  while (sensorValues[0] < IRwhite && sensorValues[1] < IRwhite && sensorValues[2] < IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite && sensorValues[5] < IRwhite && sensorValues[6] < IRwhite && sensorValues[7] < IRwhite){
    Motor1.move(BACKWARD, speed);
    qtra.readLine(sensorValues);
    delay(100);  // give a chance for sensors to finish reading?
  }
  
  /* original turnRight code that rotated the outside wheel a certain number of degrees, then stopped.  
  //Serial.println("Entering turnRight function");
  //Serial3.println("Entering turnRight function");
  Motor1.move(BACKWARD, speed, degrees, BRAKE);
  delay(1500);            //      give bot a moment to complete the turn
  // TODO change this function to accept degrees as intended rotation of bot itself
  // and calculate the wheel movement necessary to accomplish. ie: "I want my bot
  // to turn 90 degrees to the right" is more intuitive than "I want my left wheel
  // to rotate 270-ish degrees to accomplish a right hand turn."
  //
  */ 
} // close turnRight function



////////////////////////////////////////////////////////////////////////
int turnLeft(int speed){               // robot turns left by pivoting on the left wheel (only running right wheel)
  //Serial3.println("Entering turnLeft function");
  // robot turns left by pivoting on the left wheel (only running right wheel)

  // bot might be on a current line so get off the current line
  while (sensorValues[0] > IRblack || sensorValues[1] > IRblack || sensorValues[2] > IRblack || sensorValues[3] > IRblack){
    Motor2.move(BACKWARD, speed);
    qtra.readLine(sensorValues);
    delay(100);  // give a chance for sensors to finish reading
  }
  
  // ...then keep turning left while all sensors see white until a line is detected
  while (sensorValues[0] < IRwhite && sensorValues[1] < IRwhite && sensorValues[2] < IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite && sensorValues[5] < IRwhite && sensorValues[6] < IRwhite && sensorValues[7] < IRwhite){
    Motor2.move(BACKWARD, speed);
    qtra.readLine(sensorValues);
    delay(100);  // give a chance for sensors to finish reading
  }

  /*
  //Serial.println("Entering turnLeft function");
  //Serial3.println("Entering turnLeft function");
  Motor2.move(BACKWARD, speed, degrees, BRAKE);
  delay(1500);            //      give bot a moment to complete the turn
  // TODO change this function to accept degrees as intended rotation of bot itself
  // and calculate the wheel movement necessary to accomplish. ie: "I want my bot
  // to turn 90 degrees to the left" is more intuitive than "I want my right wheel
  // to rotate 270-ish degrees to accomplish a left hand turn."
  */
  
} // close turnLeft function

int pivotRight(int speed){                         // robot rotates by pivoting in place (turning both wheels in opposite directions)

  Serial3.println("pivot right");
  // move straight forward a bit to provide more room to resume line following after completing the U-Turn manuever
  uTurnBump = (nodeBump - 20); // seems bot runs a little farther while evaluating if statements above before reaching this point (???)
  Motor1.move(BACKWARD, nodeCheckSpeed, uTurnBump, BRAKE);  // TODO move nodeBump to separate function since maze solver will use, but line follower may not
  Motor2.move(BACKWARD, nodeCheckSpeed, uTurnBump, BRAKE);
  delay(500); // allow previous maneuver to complete

  // bot might be on a current line so get off the current line
  while (sensorValues[4] > IRblack || sensorValues[5] > IRblack || sensorValues[6] > IRblack || sensorValues[7] > IRblack){
    Motor1.move(BACKWARD, speed);
    Motor2.move(FORWARD, speed);
    qtra.readLine(sensorValues);
    delay(20);  // give a chance for sensors to finish reading?
  }
  
  // continue rotating until line reached
  //while (sensorValues[0] < IRwhite && sensorValues[1] < IRwhite && sensorValues[2] < IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite && sensorValues[5] < IRwhite && sensorValues[6] < IRwhite && sensorValues[7] < IRwhite){
  while (sensorValues[1] < IRwhite && sensorValues[2] < IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite && sensorValues[5] < IRwhite && sensorValues[6] < IRwhite){
    Motor1.move(BACKWARD, speed);
    Motor2.move(FORWARD, speed);
    qtra.readLine(sensorValues);
    delay(20);  // give a chance for sensors to finish reading?
  }
} // close pivotRight function

int pivotLeft(int speed){                         // robot rotates by pivoting in place (turning both wheels in opposite directions)

  Serial3.println("pivot left");
  // move straight forward a bit to improve node-to-node measurments...
  // and provide more room to resume line following after completing the U-Turn manuever
  uTurnBump = (nodeBump - 20); // seems bot runs a little farther while evaluating if statements above before reaching this point (???)
  Motor1.move(BACKWARD, nodeCheckSpeed, uTurnBump, BRAKE);  // TODO move nodeBump to separate function since maze solver will use, but line follower may not
  Motor2.move(BACKWARD, nodeCheckSpeed, uTurnBump, BRAKE);
  delay(500); // allow previous maneuver to complete

  // bot might be on a current line so get off the current line
  while (sensorValues[0] > IRblack || sensorValues[1] > IRblack || sensorValues[2] > IRblack || sensorValues[3] > IRblack){
    Motor1.move(FORWARD, speed);
    Motor2.move(BACKWARD, speed);
    qtra.readLine(sensorValues);
    delay(20);  // give a chance for sensors to finish reading?
  }
  
  // continue rotating until line reached
  //while (sensorValues[0] < IRwhite && sensorValues[1] < IRwhite && sensorValues[2] < IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite && sensorValues[5] < IRwhite && sensorValues[6] < IRwhite && sensorValues[7] < IRwhite){
  while (sensorValues[1] < IRwhite && sensorValues[2] < IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite && sensorValues[5] < IRwhite && sensorValues[6] < IRwhite){
    Motor1.move(FORWARD, speed);
    Motor2.move(BACKWARD, speed);
    qtra.readLine(sensorValues);
    delay(20);  // give a chance for sensors to finish reading?
  }
} // close pivotLeft function

////////////////////////////////////////////////////////////////////////
int checkForSwitchback(){
  // 
}

////////////////////////////////////////////////////////////////////////
int checkForEdge(){
  // check for edge of table
}


