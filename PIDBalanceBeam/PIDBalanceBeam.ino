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

#include <Servo.h>
Servo myservo;  // create servo object to control a servo 
int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin 

#include <PID_v1.h>
double PIDsetpoint, PIDinput, PIDoutput; // define varibles PID will be connecting to
PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint,2,5,1, DIRECT);  // specify PID links and initial tuning parameters
int PIDoutputMapped = 0;
int PIDoutputConstrained = 0;

// this constant won't change.  It's the pin number
// of the sensor's output:
const int pingPin = 7;

int obstacle = 200; // arbitrarally init this variable to 200 inches.
                    // ie: assume it's all clear in front of you.
                    // (Not necessary for this example, but will be
                    // used later for obstacle avoidance bot.)
                    
int timeToPing = 0;
int BallPosition = 0;
int TubeAngle = 90;
int setPoint = 24;
int servoPos = 90;

void setup() 
{ 
  Serial.begin(115200);
  myservo.attach(9); // attaches the servo on pin 9 to the servo object 
  myservo.write(90);
  Serial.println ("Level tube now...");
  delay(10000);
  Serial.println ("Tube should be level by now!");
  delay(5000);
  
  PIDinput = getBallPosition(); // initialize PID variables
  PIDsetpoint = 22;             // set the targe ball position
  myPID.SetMode(AUTOMATIC);     // turn the PID on
}

////////////////////////////////////////////////////////////////////////
void loop()

{
  //BallPosition = getBallPosition();
  PIDinput = getBallPosition();
  Serial.print("                            PIDinput: ");
  Serial.print(PIDinput);
  myPID.Compute();
  Serial.print(" PIDoutput: ");
  Serial.print(PIDoutput); // the PIDoutput ranges from 0 - 255. Probably need to map this to servo 50-130 range
                             // 0 should lift up since ball far from sensor
                             // 255 should tilt down since ball is close to sensor
  //PIDoutputMapped = map(PIDoutput, 0, 255, 50, 130); // re-map 0-255 PIDoutput values to servo's 50-130 position range
  PIDoutputMapped = map(PIDoutput, 0, 255, 130, 50); // need to reverse (?)
  Serial.print(" PIDoutputMapped: ");
  Serial.print(PIDoutputMapped);
  PIDoutputConstrained = constrain (PIDoutputMapped, 50, 130); // constrain values so servo won't tear up the rig
  Serial.print(" PIDoutputConstrained: ");
  Serial.println(PIDoutputConstrained);
  myservo.write(PIDoutputConstrained); // The big test! (may want to move PID-related lines above into a function)
  //decideMovement();
  //decideMovementWithPID();
  //Serial.println("delay in void loop");
  delay(10); // preferred setting
  //delay(60); // slowing to read screen
  
} 

////////////////////////////////////////////////////////////////////////
/* FUNCTIONS  */
////////////////////////////////////////////////////////////////////////

int getBallPosition(){
  //Serial.println("entering getBallPosition");
  long duration, inches, cm;
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  //Serial.println("pingPin LOW");
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2); // original value: 2
  //Serial.println("pingPin HIGH");
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5); // original value: 5
  //Serial.println("pingPin LOW");
  digitalWrite(pingPin, LOW);
  // This is where the ping distance is read in...
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  //Serial.println("calc duration");
  duration = pulseIn(pingPin, HIGH);
  inches = microsecondsToInches(duration);
  //Serial.println("delay 10 in getBallPosition");
  delay(10); // dialed down to 2 seemed to cause spurious readings.
            // 5 a little flakey?
            // 8 seemed pretty okay
            // 10 consistent, good
  return inches;
  
  /*
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  //Serial.println(val);  
  val = map(val, 50, 300, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
  myservo.write(val);                  // sets the servo position according to the scaled value 
  //delay(15);                           // waits for the servo to get there 
  delay(15);                           // waits for the servo to get there. original value 15
  timeToPing++;
  Serial.println(timeToPing);
  // Set the obstacle variable to the results of the pingChecker function
  if (timeToPing > 30) {
    obstacle = pingChecker(1);    // call the pingChecker function with argument of 5
    timeToPing = 0;
  }
  */
  
} // close getBallPosition function

////////////////////////////////////////////////////////////////////////
int decideMovement(){
  
  servoPos = myservo.read();
  
  Serial.print ("                                BallPosition: ");
  Serial.print (BallPosition);
  Serial.print (" servoPos: ");
  Serial.print (servoPos);  
   
  if ( servoPos > 50 && BallPosition < setPoint ){
    servoPos--;
    Serial.print (" about to move servo to pos ");
    Serial.print (servoPos);
    myservo.write(servoPos);
  }
  if ( servoPos <= 130 && BallPosition > setPoint ){
    servoPos++;
    Serial.print (" about to move servo to pos ");
    Serial.print (servoPos);
    myservo.write(servoPos);
  }
  Serial.println (" ");
  
} // close decideMovement function

////////////////////////////////////////////////////////////////////////
int decideMovementWithPID(){
  
  
} // close decideMovementWithPID function

////////////////////////////////////////////////////////////////////////
int pingChecker(int pingCount){
  ////Serial.print("Entering pingChecker function with pingCount of ");
  ////Serial.println(pingCount);
  // This function will accept one integer argument
  //    - number of pings to execute
  // Then return the averaged result of all pings performed

  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, inches, cm;
  long totalDuration = 0;

  for (int x = 0; x < pingCount; x++){
    ////Serial.print("Entering for loop with x at ");
    ////Serial.println(x);

    // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2); // original value: 2
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5); // original value: 5
    digitalWrite(pingPin, LOW);
    // This is where the ping distance is read in...
    // The same pin is used to read the signal from the PING))): a HIGH
    // pulse whose duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(pingPin, INPUT);
    duration = pulseIn(pingPin, HIGH);
    ////Serial.print("duration: ");
    ////Serial.println(duration);
    totalDuration = totalDuration + duration;
  }
  ////Serial.print("totalDuration: ");
  ////Serial.println(totalDuration);

  // Compute the average duration of all the for loop passes

  long averageDuration = totalDuration / pingCount;
  ////Serial.print("averageDuration: ");
  ////Serial.println(averageDuration);

  // ...Then the ping distance is converted to inches and cm
  // convert the time into a distance
  //cm = microsecondsToCentimeters(averageDuration);
  inches = microsecondsToInches(averageDuration);

  // This section simply prints the values to serial out
  // for easy monitoring or troubleshooting 
  Serial.print(inches);
  Serial.println(" in");
  //Serial.print(cm);
  //Serial.print("cm");
  //Serial.println();

  //  delay(100); original of 100
  delay(10); // dialed down to 2 seemed to cause spurious readings.
            // 5 a little flakey?
            // 8 seemed pretty okay
            // 10 consistent, good

  return inches;
  // close pingChecker function 
}

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
