// 2008:  #include <PololuQTRSensors.h>
// 2010:  #include <PololuQTRSensors.h>
// 2012:  #include <QTRSensors.h>

// add a line here to test out git
// adding a second line here to test out git
// adding a third line here

// testing with 2008 (had to move 2010 out of libraries folder due to overlapping names)
#include <QTRSensors.h>

// Line Following Robot
// by John Christian
//    potus98@yahoo.com
//    potus98.wikidot.com
//    @potus98 on twitter
//
// Code below is not entirely original. It may have snippets or entire
// sections from other examples or tutorials found online. Thank you
// wonderful Internet for the help. If you see something you feel should
// be specifically credited, please let me know! 
////////////////////////////////////////////////////////////////////////
//
// Now using Arduino Mega 2560
// This code should work on the Arduino Uno. I switched to the Mega
// because this bot might evolve to compete in a polyathlon and mini-sumo
// contests so I may need more digital and analog pins.
//
// QTR-8A pins
//   GND
//   Vcc
//   LED On
//   1
//   2
//   3
//   4
//   5
//   6
//   7
//   8
//
// Initial setup for reflectance sensors
// Import Pololu libraries for reflectance sensors
//#include <PololuQTRSensors.h>
// Newer Pololu libraries seem to have dropped the Pololu text
//#include <QTRSensors.h>
 
#define NUM_SENSORS             6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // avg 4 samples per sensor reading
#define EMITTER_PIN             8  // emitter controlled by digital pin 8
// sensors 0 through 7 (labeled pins 1-8 on QTR array) are connected to 
// analog inputs 0 through 7, respectively
//PololuQTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3}, 
// Newer Pololu libraries seem to have dropped the Pololu text
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5}, 
///QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5, 6, 7}, 
NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
 
////////////////////////////////////////////////////////////////////////
 
void setup() 
{ 
  // SETUP serial port monitor
  // initialize serial communication
  Serial.begin(9600);
 
  // SETUP reflectance sensor calibration
  delay(5000);   // give me a few seconds to open the serial monitor
  int i;   // initialize a basic integer variable to use for counting
  int k;
  Serial.println("reflectance sensor calibrating for about 10 seconds...");
  Serial.println("(Hint: slide array over light/dark surfaces now.)");
  for (i = 0; i < 800; i++)  // make the calibration take about 10 seconds
  {
    // reads all sensors 10 times at 2.5 ms per eight sensors
    // (i.e. ~25 ms per call)
    qtra.calibrate();
  }
  Serial.println("...reflectance calibration complete.");
  // print the calibration minimum values measured when emitters were on
  Serial.println("Sensors 1   2   3   4   5   6   7   8");
  Serial.print("Min     ");
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.print("Max     ");
  // print the calibration maximum values measured when emitters were on
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
 
// END of void setup()  
} 
 
////////////////////////////////////////////////////////////////////////
/* Main loop */
////////////////////////////////////////////////////////////////////////
 
void loop() 
 
{ 
  // Run the checkLine function. Breaking this out as a function in this
  // demo is a little overkill, but I plan to use this as part of a bigger
  // polyathlon robot program later.
  checkLine(1);
  delay(500);  // slow down the loop a little to make it easier to watch serial port
  // Now that I've collected info about my environment, let's evaluate it...
  //evalLineQuantity; // Not sure how to do this yet. Assume one line for now
  evalLineToCenter;
// END OF MAIN VOID LOOP
}
 
////////////////////////////////////////////////////////////////////////
/* FUNCTIONS  */
////////////////////////////////////////////////////////////////////////
 
int checkLine(int color){
  // Function receives an argument of 0 (for white line) 1 (for black
  // line -default) code below pulled from black line detection example,
  // so actually looking for black on white.
  // ** arg to change color not implemented yet **
 
  // read calibrated sensor values and obtain a measure of the line
  // position from 0 to 7000, where 0 means directly under sensor 0 or
  // the line was lost past sensor 0, 1000 means directly under sensor 1,
  // 2000 means directly under sensor 2, etc.
  //
  // Note: the values returned will be incorrect if the sensors have not
  // been properly calibrated during the calibration phase.  To get raw
  // sensor values, call:
  //    qtra.read(sensorValues);
  unsigned int position = qtra.readLine(sensorValues);
 
  // print the sensor values as numbers from 0 to 9, where 0 means
  // maximum reflectance and 9 means minimum reflectance, followed by the
  // line position. Output will look similar to this EXAMPLE...
  //
  //    0 0 0 8 7 0 0 0    3455
  //
  // The first eight positions are the eight sensors. 0 is high
  // reflectance (white) while 9 is low reflectance (black). In the
  // example above, the array is centered over a black electrical tape
  // line on a white posterboard.
  //
  // The "3455" number to the right is on a scale of 0 to 7000 with 0
  // representing one end of the array and 7000 representing the other\
  // end of the array. Instead of worrying about the exact values of each
  // sensor individually, you could write your line following code to
  // simply keep the scaled number near a target value of 3500.
  //
  //
  unsigned char i;
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i] * 10 / 1001);
    Serial.print(' ');
  }
  Serial.print("    ");
  Serial.println(position);
 
  // following lines may be used in the future  
  int sensor1 = sensorValues[0] * 10 / 1001;
  int sensor2 = sensorValues[1] * 10 / 1001;
  int sensor3 = sensorValues[2] * 10 / 1001;
  int sensor4 = sensorValues[3] * 10 / 1001;
  int sensor5 = sensorValues[4] * 10 / 1001;
  int sensor6 = sensorValues[5] * 10 / 1001;
  int sensor7 = sensorValues[6] * 10 / 1001;
  int sensor8 = sensorValues[7] * 10 / 1001;
 
// END of checkLine function 
}

int evalLineToCenter(){
 Serial.println("Entered evalLineToCenter function...");
 delay(500);
 Serial.print("sensorValue stored in position 8 is the big number?");
 Serial.println(sensorValues[8]);
}
