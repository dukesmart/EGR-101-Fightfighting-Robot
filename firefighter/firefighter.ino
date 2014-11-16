#include <QTRSensors.h>
#include <Servo.h>

/*
EGR 101 Firefighting Challenge
Dustin Steadman, Matthew Gileau, Sydney O'Neal
17 November 2014

*/

// Original example source made available through Professor Brian Butka (?)

// This example is designed for use with eight QTR-1RC sensors or the eight sensors of a
// QTR-8RC module.  These reflectance sensors should be connected to digital inputs 3 to 10.
// The QTR-8RC's emitter control pin (LEDON) can optionally be connected to digital pin 2, 
// or you can leave it disconnected and change the EMITTER_PIN #define below from 2 to 
// QTR_NO_EMITTER_PIN.

// The main loop of the example reads the raw sensor values (uncalibrated).
// You can test this by taping a piece of 3/4" black electrical tape to a piece of white 
// paper and sliding the sensor across it.  It prints the sensor values to the serial 
// monitor as numbers from 0 (maximum reflectance) to 2500 (minimum reflectance).


#define NUM_SENSORS   3     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   255

// sensors 0 through 2 are connected to digital pins 8 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) { 
  11, 9, 10}
,
NUM_SENSORS, TIMEOUT,EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];
// Set the thresholds you may need different values for each sensor
unsigned int left_thresh = 600;
unsigned int center_thresh = 600;
unsigned int right_thresh = 600;
// Declare some toggles
boolean toggleFan = false;
boolean toggleMove = true;
boolean moveUsingDistance = false;

// Set pins for hardware
int analogFirePin = A0;
int digitalFirePin = 5;
int fireSensorValue = 0;
int ledin = 7;
int ledout = 8;
int left = 0;
int center = 0;
int right = 0;
Servo leftWheel;
Servo rightWheel;

void setup()
{
  // Configure fan
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  // Configure LED
  pinMode(ledin, OUTPUT);
  pinMode(ledout, OUTPUT);
  // Turn fan off (just in case)
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  // Configure motors
  leftWheel.attach(13);
  rightWheel.attach(12);
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(1000);
}


void loop()
{
  // Read sensors
  int fireSensorValue = digitalRead(digitalFirePin);
  float distance = analogRead(1);
  // Project to serial monitor
  Serial.print("Distance: \t");
  Serial.println(distance);
  Serial.print("Fire: \t");
  Serial.println(fireSensorValue);
  
  digitalWrite(ledin, LOW); // Turn off LED (just in case)
  
  // Check for fires
  if(fireSensorValue != 1){
    toggleFan = false;
  }
  else {
    toggleFan = true;
  }

  if(toggleFan != true){ // If there's a fire...
    leftWheel.writeMicroseconds(1500); // Stop,
    rightWheel.writeMicroseconds(1500);
    digitalWrite(3, LOW); // Then use the fan to extinguish it,
    digitalWrite(4, HIGH);
    digitalWrite(ledin, HIGH); // And finally, use the LED to make sure observers know what's going on
    delay(2500); // Wait for the fire to be extinguished
    toggleMove = false; // Prevent further movement
  }
  else if(toggleMove != true){ // Prevent further movement if the fire has been extinguished
    leftWheel.writeMicroseconds(1500); // Stop
    rightWheel.writeMicroseconds(1500);
    digitalWrite(3, LOW); // Ensure fan is off
    digitalWrite(4, LOW);
    digitalWrite(ledin, LOW); // Turn LED off
  }
  else if (distance > 300){
    leftWheel.writeMicroseconds(1500); // Stop, you're about to hit an obstacle
    rightWheel.writeMicroseconds(1500);
    digitalWrite(3, LOW); // Turn the fan off (just in case)
    digitalWrite(4, LOW);
    digitalWrite(ledin, HIGH); // Turn on the LED to let observers know something is happening
    delay(100);
  }    
  else
  {
    digitalWrite(3, LOW); // Turn the fan off (just in case)
    digitalWrite(4, LOW);
    digitalWrite(ledin, LOW); // Turn the LED off (just in case)
    // read raw sensor values
    qtrrc.read(sensorValues);

    // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
    // 1023 means minimum reflectance
    if(sensorValues[0]>left_thresh){
      left=1;
    }
    else {
      left=0;
    }
    Serial.print(left);  
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor 

    if(sensorValues[2]>center_thresh){
      center=1;
    }
    else {
      center=0;
    }
    Serial.print(center);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor

    if(sensorValues[1]>right_thresh){
      right=1;
    }
    else {
      right=0;
    }
    Serial.print(right); 
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
    Serial.println();
    if(center || left || right){
      moveUsingDistance = false;
    }
    if ((center && !left && !right) || (center && left && right)){ //TAPE IS IN THE CENTER BUT NOT ON THE SIDES  GO STRAIGHT

      leftWheel.writeMicroseconds(1450);
      rightWheel.writeMicroseconds(1550);
      Serial.println("In the center");
      delay(10);
    }
    else if (left && center && !right){//TAPE IS IN THE LEFT AND CENTER BUT NOT ON THE RIGHT   TURN LEFT 
      leftWheel.writeMicroseconds(1450);
      rightWheel.writeMicroseconds(1505);
      Serial.println("Too far right");
      delay(15);
    }
    else if (right && center && !left){//TAPE IS IN THE RIGHT AND CENTER BUT NOT ON THE RIGHT   TURN RIGHT
      leftWheel.writeMicroseconds(1495);
      rightWheel.writeMicroseconds(1550);
      Serial.println("Too far left");
      delay(15); 
    }
    else if (left && !center && !right){ // TOO FAR LEFT
      leftWheel.writeMicroseconds(1450);
      rightWheel.writeMicroseconds(1505);
      Serial.println("Much too far right");
      delay(15);
    }
    else if (!left && !center && right){ // TOO FAR RIGHT
      leftWheel.writeMicroseconds(1495);
      rightWheel.writeMicroseconds(1550);
      Serial.println("Much too far left");
      delay(15);
    }
    else if ( left && !center && right){
      leftWheel.writeMicroseconds(1450);
      rightWheel.writeMicroseconds(1550);
      Serial.println("Something weird is happening, but moving forward anyway");
      delay(05);
    }
    else {                         //TAPE IS NOT DOING WHAT IS EXPECTED     DO SOMETHING
      // Spin until you can find tape again
      leftWheel.writeMicroseconds(1490);
      rightWheel.writeMicroseconds(1490);
      // Activate the search for the tape
      // Activate motion based on distance sensor values
      moveUsingDistance = true;
      delay(250);
      Serial.println("No tape found!");
    }
  }
}

