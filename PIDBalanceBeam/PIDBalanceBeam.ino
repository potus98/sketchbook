// Based on File > Examples > Servo > Knob
// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h> 

Servo myservo;  // create servo object to control a servo 
int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin 

// this constant won't change.  It's the pin number
// of the sensor's output:
const int pingPin = 7;

int obstacle = 200; // arbitrarally init this variable to 200 inches.
                    // ie: assume it's all clear in front of you.
                    // (Not necessary for this example, but will be
                    // used later for obstacle avoidance bot.)


void setup() 
{ 
  Serial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  
  // SETUP serial port monitor
  // initialize serial communication
  Serial.begin(115200);
} 

void loop() 

{ 
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  Serial.println(val);  
  val = map(val, 50, 300, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
  myservo.write(val);                  // sets the servo position according to the scaled value 
  delay(15);                           // waits for the servo to get there 
  
  // Set the obstacle variable to the results of the pingChecker function
  obstacle = pingChecker(5);    // call the pingChecker function with argument of 5
  // Adjust the sensitivity of the bot here.
  // pingChecker argument = number of measurements to collect.
  // Some sensors don't read closer than a certain distance
  // Sparkfun ultrasonic distance sensor EZ1 doesn't seem to read <5" very well
  // pingChecker function will use the arg of 5 to collect 5 distance
  // measurements and average them together to help smooth out spurious
  // readings.
  delay(50);    // pause for a moment to allow easier reading of Serial Monitor
  
} 

////////////////////////////////////////////////////////////////////////
/* FUNCTIONS  */
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
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);
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
  Serial.println("in");
  //Serial.print(cm);
  //Serial.print("cm");
  //Serial.println();

  //  delay(100); original of 100
  delay(50);

  return inches;
  // close pingChecker function 
}

long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
