////////////////////////////////////////////////////////////////////////
// PID Demo Sketch for use with PID Balance Beam rig. See: http://potus98.com
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

#include <Servo.h>  // include the Servo library
Servo myservo;      // create servo object to control a servo 
int val;            // variable to read the value from the analog pin 

#include <PID_v1.h>  // include the PID library See: http://playground.arduino.cc/Code/PIDLibrary
double PIDsetpoint, PIDinput, PIDoutput;                       // define varibles PID will be connecting to

PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,7,.6,3, DIRECT);  // specify PID links and initial tuning parameters
//                                              ^
//                                              |
//                                              `<<< Change PID here 1/2 <<<<


                                                               // 2,5,1 was the initial starting point
// from br3ttb on arduino forums... Parameters and what they do (sort of)
// P_Param:  the bigger the number the harder the controller pushes.
// I_Param:  the SMALLER the number (except for 0, which turns it off,)  the more quickly the controller reacts to load changes, but the greater the risk of oscillations.
// D_Param: the bigger the number  the more the controller dampens oscillations (to the point where performance can be hindered)

int PIDoutputMapped = 0;       // used when mapping PID output to safe servo range so servo doesn't twist demo rig apart
int PIDoutputConstrained = 0;  // extra safety against sending servo to positions that might break the demo rig
const int pingPin = 7;         // defines signal pin used by ping sensor
int timeToPing = 0;            // used during ping sensor operation
int BallPosition = 0;          // calculated distance of ball from sensor
int setPoint = 24;             // hardcoded target point (to-do: make adjustable via potentiometer)
int servoPos = 90;             // servo position
int TestRuns = 0;              // counter used to limit total number of cycles to make data collection easier

////////////////////////////////////////////////////////////////////////
void setup() 
{ 
  Serial.begin(115200);
  myservo.attach(9);    // attaches the servo on pin 9 to the servo object 
  myservo.write(90);
  Serial.println ("Level tube now...");
  delay(10000);
  Serial.println ("Tube should be level by now!");
  delay(5000);
  PIDinput = getBallPosition(); // initialize PID variable
  PIDsetpoint = 24;             // set the target ball position (to-do: make adjustable via potentiometer)
  myPID.SetMode(AUTOMATIC);     // turn the PID on
}

////////////////////////////////////////////////////////////////////////
void loop()
{
  
 //decideMovement();         // Simple function to exercise rig mechanics
 decideMovementWithPID();    // The PID version you're probably most interested in using.

}   // end of void loop

/////////////////////////////* FUNCTIONS  */////////////////////////////

////////////////////////////////////////////////////////////////////////

int decideMovement(){

  if (TestRuns == 0){
    Serial.println("No PID");
    Serial.println("setpoint BallPosition servoPosition");
  }  
  BallPosition=getBallPosition();
  servoPos = myservo.read();
  if ( servoPos > 30 && BallPosition < setPoint ){
    servoPos--;
    myservo.write(servoPos);
  }
  if ( servoPos <= 150 && BallPosition > setPoint ){
    servoPos++;
    myservo.write(servoPos);
  }
  delay(25); // Provide time for servo to reach destination.
  
  Serial.print (setPoint);
  Serial.print(" ");
  Serial.print (BallPosition);
  Serial.print(" ");
  Serial.println (servoPos);  
  TestRuns++;
  if (TestRuns > 2000){
    Serial.print(TestRuns);
    Serial.println(" TestRuns complete. Pausing for 2 minutes...");
    Serial.println(" ");
    delay(120000);
    TestRuns = 0;
  }
} // close decideMovement function

////////////////////////////////////////////////////////////////////////
int decideMovementWithPID(){
  
  if (TestRuns == 0){
    Serial.println("PID tunings: 7,.6,3");                                         //   <<< Change PID here 2/2 <<<<
    Serial.println("PIDsetpoint BallPosition PIDoutput PIDoutputMapped");
  }  
  PIDinput = getBallPosition();
  myPID.Compute();
  PIDoutputMapped = map(PIDoutput, 0, 255, 150, 30);
  PIDoutputConstrained = constrain (PIDoutputMapped, 30, 150); // constrain values so servo won't tear up the rig
  myservo.write(PIDoutputConstrained);
  delay(25); // Provide time for servo to reach destination.
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
} // close decideMovementWithPID function

////////////////////////////////////////////////////////////////////////
int getBallPosition(){
  long duration, inches, cm;
                                    // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
                                    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);              // original value: 2
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);              // original value: 5
  digitalWrite(pingPin, LOW);
                                     // This is where the ping distance is read in...
                                     // The same pin is used to read the signal from the PING))): a HIGH
                                     // pulse whose duration is the time (in microseconds) from the sending
                                     // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  inches = microsecondsToInches(duration);
  delay(15);                         // Setting too low seemed to introduce issues. Too much ping noise in tube?
  return inches;  
} // close getBallPosition function

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
