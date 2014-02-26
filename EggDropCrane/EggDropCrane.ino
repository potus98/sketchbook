/*
  by John Christian
     potus98@yahoo.com
     potus98.com
     @potus98
  
  This code was used with an arduino based egg dropping robot. The robot
  hangs from the ceiling by a strong fishing line. It uses a hoist to raise
  and lower itself to pre-defined heights. Distance measurements are obtained
  by two sonar sensors so robot knows distance from floor and distance from
  ceiling. A servo controls release of egg basket which drops an egg.
  Commands are issued to the robot via bluetooth serial connection. I used
  BlueTerm on android to interface with robot.
  
  Code below is not entirely original. It may have snippets or entire
  sections from other examples or tutorials found online. Thank you
  wonderful Internet for the help. If you see something you feel should
  be specifically credited, please let me know!

  Pololu is awesome. They have great customer service, great prices,
  and great prodoucts. Not affiliated in any way. Just a satisfied customer.

 */

#include <SoftwareSerial.h>
//SoftwareSerial bluetooth(bluetoothTx, bluetoothRx); // use with Arduino UNO, comment out for Mega 2560
#include <Servo.h>              // Servo library
Servo myservo;                  // create servo object to control a servo
#include <Wire.h>               // NXT Motor Shield libraries
#include <NXTShield.h>          // NXT Motor Shield libraries
Motor1 rightMotor;              // define hoist
Motor2 leftMotor;               // define hoist
int bluetoothTx = 14;           // TX-O pin of bluetooth (Mega 2560 pin 14)
int bluetoothRx = 15;           // RX-I pin of bluetooth (Mega 2560 pin 15)
int servoPos = 170;             // initial servo position
const int pingPinUp = 52;       // defines signal pin used by ping sensor
const int pingPinDown = 53;     // defines signal pin used by ping sensor
int led = 13;                   // Pin 13 has an LED connected on most Arduino boards
char character = 'a';           // initialize variable
char command = 'h';             // initialize create variable
char newcommand = 'h';          // initialize create variable
int headroom = 0;               // clearance above robot
int footroom = 0;               // clearance below robot
int loadHeight = 15;            // target height for egg loading
int dropHeight = 30;            // target height for egg dropping
int targetHeight = loadHeight;  // target height in inches, init with loadHeight
int upperSetPoint = 6;          // create safe zone to reduce thrashing without PID (inches)
int lowerSetPoint = 6;          // create safe zone to reduce thrashing without PID (inches)


// Command list
// l (lower case L)    Load     Causes robot to lower itself for reloading
// p                   Prepare  Causes robot to seek and maintain target height
// d                   Drop     Causes robot to drop the egg
// t                   Test     
// h                   Hold     Motors stop, ping sensors slow interval
// 1 (number one)      24"
// 2                   48"
// 3                   72"
// 4                   120"
// u                   Up        go up for 3 seconds
// g                   Ground    go down for 3 seconds

////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);        // Begin the serial monitor at 9600bps   Used for USB serial output to PC
  Serial3.begin(9600);       // Begin the serial monitor at 9600bps   Use with Arduino Mega 2560 w/ pins 14,15
  pinMode(led, OUTPUT);      // initialize the digital pin as an output.
  
  myservo.attach(51);        // attaches the servo on pin 51 to the servo object
  myservo.write(170);        // send servo to horizontal (hold the egg basket) position
}

////////////////////////////////////////////////////////////////////////////
void loop()
{
  newcommand=getCommand();   // check bluetooth serial connection for valid new command
  if (newcommand == '1' || newcommand == '2' ||newcommand == '3' ||newcommand == '4' || newcommand == 'l' || newcommand == 'p' || newcommand == 'd' || newcommand == 't' || newcommand == 'h' || newcommand == 'u' || newcommand == 'g' ){    // ...but don't change status unless a valid command
    command = newcommand;
  }
  
  Serial3.print("cmd status: ");    // print current command mode to BlueTerm
  Serial3.print(command);
  Serial3.print(" ");
  //headroom=getDistanceAbove();    // obtain overhead clearance // comment out for troubleshooting zero readings
  headroom=12;                      // temp hardcoded headroom due to suspect ping interference
  footroom=getDistanceBelow();      // obtain ground clearance
  
  if (headroom < 12 ){              // safety check for obstacles above robot
    Serial3.print("Safety Tolerance Exceeded");
    Serial3.print(" headroom: ");
    Serial3.print(headroom);
    Serial3.print(" footroom: ");
    Serial3.println(footroom);
    leftMotor.move(backward, 150);   //
    rightMotor.move(backward, 150);  // move down for 3 seconds
    delay(3000);                     //
    command = 'h';
    hold();
  }

  if (command == '1'){     // if command equals number 1
    moveRobot(24);         // robot to seek and maintain height of 24"
  }
  
  if (command == '2'){     // if command equals number 2
    moveRobot(48);         // robot to seek and maintain height of 48"
  }
  
  if (command == '3'){     // if command equals number 3
    moveRobot(96);         // robot to seek and maintain height of 96"
  }
  
  if (command == '4'){     // if command equals number 4
    moveRobot(120);        // robot to seek and maintain height of 120"
  }
    
  if (command == 'l'){     // if command equals letter l
    moveRobot(loadHeight); // robot to seek and maintain height defined by loadHeight variable
  }
  
  if (command == 'p'){     // if command equals letter p
    moveRobot(dropHeight); // robot to seek and maintain height defined by dropHeight variable
  }
  
  if (command == 'd'){
    dropEgg();
  }
  
  if (command == 'h'){
    hold();
  }
  
  if (command == 'u'){      // provide command for manual height adjustments
    Serial3.println("                              up for 3 secs");
    leftMotor.move(forward, 180);
    rightMotor.move(forward, 180);
    delay(3000);
    command = 'h';                           // hold after going up for 3 seconds
  } 
  
  if (command == 'g'){      // provide command for manual height adjustments
    Serial3.println("                              down for 3 secs");
    leftMotor.move(backward, 180);
    rightMotor.move(backward, 180);
    delay(3000);
    command = 'h';                           // hold after going down for 3 seconds
  } 
}

////////////////////////////////////////////////////////////////////////////
/* FUNCTIONS */
////////////////////////////////////////////////////////////////////////////

char getCommand(){
  // get serial commands from bluetooth receiver
  if(Serial3.available())                    // If the bluetooth sent any characters
  {
    character = int(Serial3.read());         // read in next character received via bluetooth
    Serial.print(character);                 // print the character to Serial port on computer
    Serial3.println(character);              // echo the character back to the BlueTerm terminal
    
    if (character == 'l'){
      return('l');
    }
    else if (character == 'p'){
      return('p');
    }
    else if (character == 'd'){
      return('d');
    }
    else if (character == 't'){
      return('t');
    }
    else if (character == 'h'){
      return('h');
    }
    else if (character == '1'){
      return('1');
    }
    else if (character == '2'){
      return('2');
    }
    else if (character == '3'){
      return('3');
    }
    else if (character == '4'){
      return('4');
    }
    else if (character == 'u'){
      return('u');
    }
    else if (character == 'g'){
      return('g');
    }
  }
}

////////////////////////////////////////////////////////////////////////////
int getDistanceAbove(){
  long duration, inches, cm;
                                     // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
                                     // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPinUp, OUTPUT);
  digitalWrite(pingPinUp, LOW);
  delayMicroseconds(2);              // original value: 2
  digitalWrite(pingPinUp, HIGH);
  delayMicroseconds(5);              // original value: 5
  digitalWrite(pingPinUp, LOW);
                                     // This is where the ping distance is read in...
                                     // The same pin is used to read the signal from the PING))): a HIGH
                                     // pulse whose duration is the time (in microseconds) from the sending
                                     // of the ping to the reception of its echo off of an object.
  pinMode(pingPinUp, INPUT);
  duration = pulseIn(pingPinUp, HIGH);
  inches = microsecondsToInches(duration);
  delay(25);
  return inches;
}

////////////////////////////////////////////////////////////////////////////
int getDistanceBelow(){
  long duration, inches, cm;
                                     // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
                                     // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPinDown, OUTPUT);
  digitalWrite(pingPinDown, LOW);
  delayMicroseconds(2);              // original value: 2
  digitalWrite(pingPinDown, HIGH);
  delayMicroseconds(5);              // original value: 5
  digitalWrite(pingPinDown, LOW);
                                     // This is where the ping distance is read in...
                                     // The same pin is used to read the signal from the PING))): a HIGH
                                     // pulse whose duration is the time (in microseconds) from the sending
                                     // of the ping to the reception of its echo off of an object.
  pinMode(pingPinDown, INPUT);
  duration = pulseIn(pingPinDown, HIGH);
  inches = microsecondsToInches(duration);
  delay(25);
  return inches;
}

////////////////////////////////////////////////////////////////////////////
int moveRobot(int setPoint){         // move to the target height
  Serial3.print("head: ");           // send telemetry data to serial connection
  Serial3.print(headroom);
  Serial3.print(" foot: ");
  Serial3.print(footroom);
  Serial3.print(" sP: ");
  Serial3.println(setPoint);
  upperSetPoint = setPoint + 2;      // check for being in target zone
  lowerSetPoint = setPoint - 2;
  if ( footroom <= upperSetPoint && footroom >= lowerSetPoint ){   // if we're in the butter zone
    Serial3.println("Stop Hoist");
    leftMotor.stop();
    rightMotor.stop();
    return(0);
  }
  
  if ( footroom < setPoint ){
    // keep hoisting up
    Serial3.println("                                         up");
    leftMotor.move(forward, 180);
    rightMotor.move(forward, 180);
  }
  
  if ( footroom > setPoint ){
    // keep hoisting down
    Serial3.println("                                         down");
    leftMotor.move(backward, 150);
    rightMotor.move(backward, 150);
  }
}

////////////////////////////////////////////////////////////////////////
int dropEgg(){
  // drop the egg
  Serial3.println("                                         DROP!");
  delay(2000);
  myservo.write(50);     // turn servo horn to release egg basket
  delay(2000);           // use gravity to remove egg from robot
  myservo.write(170);    // return servo horn to holding position
  command = 'h';         // hold after dropping egg
}

////////////////////////////////////////////////////////////////////////
int hold(){
  Serial3.println("                                       HOLDING");
  leftMotor.stop();
  rightMotor.stop();
  delay(2000);
}

////////////////////////////////////////////////////////////////////////
long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second). This gives the distance travelled by the ping, outbound
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

