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
const int ledPin =  13;     // NXT shield already has an LED on pin 13, let's use it

////////////////////////////////////////////////////////////////////////
// Prepare Pololu QTR-8A IR Reflectance Array
#include <QTRSensors.h>    // https://github.com/pololu/qtr-sensors-arduino
#define NUM_SENSORS             8     // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4     // average 4 analog samples per sensor reading
#define EMITTER_PIN             27    // emitter is controlled by digital pin 27
QTRSensorsAnalog qtra((unsigned char[]) {8, 9, 10, 11, 12, 13, 14, 15},
NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

////////////////////////////////////////////////////////////////////////
// Prepare PD - alternative to using PID library
#define Kp .2 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd .8 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150  // this is the speed at which the motors should spin when the robot is perfectly on the line
int lastError = 0;

////////////////////////////////////////////////////////////////////////
//Prepare Parralax BlueTooth Module RN-42
#include <SoftwareSerial.h>
//SoftwareSerial bluetooth(bluetoothTx, bluetoothRx); // uncomment this line for use with Arduino UNO, comment out for Mega 2560
int bluetoothTx = 14;           // TX-O pin of bluetooth (Mega 2560 pin 14)
int bluetoothRx = 15;           // RX-I pin of bluetooth (Mega 2560 pin 15)

////////////////////////////////////////////////////////////////////////
// Prepare misc variables
int TestRuns = 0;              // counter used to limit total number of cycles to make data collection easier                               
int speedMotorA = 0;
int speedMotorB = 0;
int nodeType = 0;
int IRwhite = 200; // any IR value smaller than this is treated as white
int IRblack = 600; // any IR value larger than this is treated as black

////////////////////////////////////////////////////////////////////////
// Setup
void setup()
{
  Serial.begin(115200);          // initialize serial communication over USB
  Serial3.begin(9600);         // Begin the serial monitor over BlueTooth. Use with Arduino Mega 2560 w/ pins 14,15
  pinMode(ledPin, OUTPUT);       // initialize the digital pin as an output

} // close void setup()



////////////////////////////////////////////////////////////////////////
// Main loop
void loop()
{
  if (TestRuns == 0){
    delay(3000);
    calibrateIRarray();  // TODO do I need to calibrate array every time?
  }  

  //testNXTShield();          // basic test/demo of NXTShield driving two NXT servos
  testBlueToothSerial();       // basic test/demo of Parallax BlueTooth Module RN-42
  //calibrateIRarray();
  //lineFollowerMode();

  // check for test run length
  TestRuns++;
  if ( TestRuns > 4000 ){            // Change to 200000 (or remove TestRun checking) before competition !!!
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
int testBlueToothSerial(){
  Serial.println("Entering testBlueToothSerial function");
  Serial3.println("TX test: 0123456789");
  delay(2000);
}  // close testBlueToothSerial


////////////////////////////////////////////////////////////////////////
int calibrateIRarray(){
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
    // call pause function at end of calibration to wait for button push
    //pause();
    Serial.println("Leaving calibrateIRarray function");
} // close calibrateIRarray function


////////////////////////////////////////////////////////////////////////
int allStop(){                                  // turn off drive motors
  Serial.println("allStop - stopping both motors");
  Motor1.stop();
  Motor2.stop();
}

////////////////////////////////////////////////////////////////////////
int lineFollowerMode(){                  // bot acts like a line follower
  
  nodeType = checkForNode();
  
  switch (nodeType) {
    case 3:
      // TODO move following to separate function
      // proceed 1" (8" wheel circumference = 45 degrees per inch)
      //Motor1.move(BACKWARD, 100, 45, COAST);
      //Motor2.move(BACKWARD, 100, 45, COAST);
      Motor1.move(BACKWARD, 100, 45, BRAKE);
      Motor2.move(BACKWARD, 100, 45, BRAKE);
      //while (Motor1.isTurning() || Motor2.isTurning()); // Wait until it has reached the position
      //delay(3000); // this delay for testing purposes. TODO remove it later
      break;
    default:
      followLine();
  }
  
} // close lineFollowerMode


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

  Serial.println("Entering checkForNode function");

  // Evaluate IR sensor array readings to identify potential nodes or intersections.
  // If sensor readings suggest bot has encountered a node, this function may perform
  // manual maneuvers to obtain more information about the current line configuration.
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
  //   7  right hand switchback
  //   8  left hand switchback

  //      ---- need these for more efficient maze solving ----
  //   9  90 degree right turn (bot can only turn right) need to identify such nodes for building a coordinate grid of a maze
  //  10  90 degree left turn (bot can only turn left) need to identify such nodes for building a coordinate grid of a maze
  //
  //   TODO variable-ize the 700 and 80 reflectance readings for easier adjustment (or better, derive them from calibration test)
  //   TODO return net x,y changes so dead reckoning algorithms know what this function did to the bot's position
   
  //   notes:
  //   only middle four sensors seemed to get tripped by a 90 degree turn when bot was already leaning into the turn

  // take an IR reading
  Serial.println("taking an IR reading");
  qtra.readLine(sensorValues);

  // print the IR values
  int i;
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print(' ');
  }
  Serial.println(' ');

  // check IR values for an intersection
  if (sensorValues[1] > IRblack && sensorValues[2] > IRblack && sensorValues[3] > IRblack && sensorValues[4] > IRblack && sensorValues[5] > IRblack && sensorValues[6] > IRblack)
  {
    // If reading black across the 6 middle QTR-8A sensors, we might be facing one of three things
    // four way intersection (return 1)
    // black finish square (return 2)
    // dead end T intersection (return 3)
    //
    // This implementation is basic since it requires careful control of distance moved between each IR reading
    // TODO find a method that can track IR readings across time (actually, across distances via wheel encoders)
    // in order to avoid having to stop the bot to identify intersection types.
    //
    Serial.println(" Sensors 1-6 seeing black...");
    allStop();
    delay(500);
    // proceed 1" (8" wheel circumference = 45 degrees per inch)
    Serial.println(" Proceeding 1 inch...");
    Motor1.move(BACKWARD, 100, 45, BRAKE);
    Motor2.move(BACKWARD, 100, 45, BRAKE);
    //while (Motor1.isTurning() || Motor2.isTurning()); // Wait until it has reached the position
    //delay(2000);
    Serial.println(" Taking another reflectance reading");
    qtra.readLine(sensorValues);

    if (sensorValues[0] < IRwhite && sensorValues[7] < IRwhite) // outer sensors see white
    {
      Serial.println(" End sensors are both seeing white. Let's check if either of the middle sensors are seeing a line");
      if (sensorValues[3] > IRblack || sensorValues[4] > IRblack) // at least one of the middle sensors sees black
      {
        Serial.println(" Looks like bot is on top of a four way intersection, returning 1");
        return 1;
      }
      //Serial.println(" Saw a black line, moved 1 inch, outer sensors are seeing white, but middle sensors don't see black. Did I lose the line, or hit a Y fork?");
      //Serial.println(" ...not sure what to do, so returning 0");
      //return 0;
      // meh, see epiphany below
    }
    
    if (sensorValues[1] > IRblack && sensorValues[2] > IRblack && sensorValues[3] > IRblack && sensorValues[4] > IRblack && sensorValues[5] > IRblack && sensorValues[6] > IRblack)
    {
      Serial.println(" Maybe this is the finish square");
      // let's proceed a little more just to be sure
      // proceed 1" (8" wheel circumference = 45 degrees per inch)
      Motor1.move(BACKWARD, 100, 45, BRAKE);
      Motor2.move(BACKWARD, 100, 45, BRAKE);
      //while (Motor1.isTurning() || Motor2.isTurning()); // Wait until it has reached the position
      delay(1000);
      Serial.println(" Taking another reflectance reading to check for finish square");
      qtra.readLine(sensorValues);
      if (sensorValues[1] > IRblack && sensorValues[2] > IRblack && sensorValues[3] > IRblack && sensorValues[4] > IRblack && sensorValues[5] > IRblack && sensorValues[6] > IRblack)
      {
        Serial.println(" It's all black after a second reading. Must be the finish square! Returning 2");
        delay(10000); // TODO remove this debugging delay
        return 2;
        // TODO I guess bot should check for continuing line (we just crossed a thick cross street?) or all white and recover
      }
      Serial.println(" I thought I might be in the finish square, but maybe I re-read the cross line too fast?");
      // Epiphiny, this where I realized I'll either need to stop and take very careful IR readings while advancing specific known distances
      // OR, remember multiple IR array readings over time (encoder ticks) and compare.
      return 1;
    }
    
    if (sensorValues[0] < IRwhite && sensorValues[1] < IRwhite && sensorValues[2] < IRwhite && sensorValues[3] < IRwhite && sensorValues[4] < IRwhite && sensorValues[5] < IRwhite && sensorValues[6] < IRwhite && sensorValues[7] < IRwhite)
    {
      Serial.println(" It's all white which suggests a dead end T intersection. Returning 3");
      return 3;
    }

  } //end of check for intersections 1, 2, and 3

  // no nodes detected
  Serial.println(" No nodes detected, returning 0 (zero)");
  return 0;
} // close checkForNode() function


////////////////////////////////////////////////////////////////////////
int checkForSwitchback(){
  // 
}

////////////////////////////////////////////////////////////////////////
int checkForEdge(){
  // check for edge of table
}




