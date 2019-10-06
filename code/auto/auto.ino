#include <MPU6050_tockn.h> // Library for Gyroscope
#include <Wire.h> // Prerequisite for Gyroscope 
/*
   Autonomous tracked vehicle "Larry"
   Version 1.4
   Made by:
   David Schmidt, Max Grüning, Steven Bolln, Pascal Harders
   Feb-Mar 2019
*/

// DC Motor 1 (right side)
byte DCM1 = 10;
byte in1 = 9;
byte in2 = 8;

// DC Motor 2 (left side)
byte DCM2 = 5;
byte in3 = 7;
byte in4 = 6;

// Motor speeds, vary on different ground
int motorvarR = 160; // right motor (160)
int motorvarL = 215; // left motor (210/207)

// Sharp infraredsensors , ir_dist may vary depending on your wallcolor
int ir_dist = 209;  // Value shown by sensor (if printed in serial monitor) (209)
byte ir1 = 14; // Sharp IR sensor on the front
byte ir2 = 15; // Sharp IR sensor on the left
byte ir3 = 17; // Sharp IR sensor on the right
int ir_delay = 50; // Waiting time between the readout attempts of the IRSensor

// Gyroscope
MPU6050 mpu6050(Wire); // Gyro is works with the Wire library
int degreecounter = 0; // Variable while turning
bool gyro = 0; // Return of the function gyro_sensor
int gyro_delay = 50; // Waiting time between the readout attempts of the gyroscope

// Endstops / bumper (Failsafe, normally not used)
byte bumpPin1 = 3; // front

// Time relevant
const unsigned short sec = 1000;  // 1 second

// ## Function prototypes ##

// Sensorfunctions
int ir_sensor(int); // Input: sensor number | Return: distance too small (1) or distance OK (0)
bool gyro_sensor(signed short int angle, int degree); // Input: Current angle, number of degrees to rotate | Return: Rotation completed (1) or not(0)

// Endstopfunction
bool bumpers(); // Input: Nothing | Return: pressed (1) or unpressed (0)

// turnfunction
void turn(int, int); // Input: Number of degrees, correction without rotation | Return: Nothing

// Wait function (replacement for delay();)
void wait(unsigned short); // Input: Duration | Return: Nothing

// ######################## SETUP ########################
// runs once
void setup()
{
  Serial.begin(9600); // Starts the serial interface
  Serial.println("##PROGRAMMSTART##");

  // Pin-definition
  pinMode(DCM1, OUTPUT); // DC motor 1
  pinMode(DCM2, OUTPUT); // DC motor 2

  pinMode(in1, OUTPUT); // DC motor 1 input1
  pinMode(in2, OUTPUT); // DCM1 input2

  pinMode(in3, OUTPUT); // DCM2 input1
  pinMode(in4, OUTPUT); // DCM2 input2

  pinMode(bumpPin1, INPUT_PULLUP); // endstop 1, for safety's sake including pullup

  pinMode (ir1, INPUT); // Sharp IR sensor on the front
  pinMode (ir2, INPUT); // Sharp IR sensor on the left
  pinMode (ir3, INPUT); // Sharp IR sensor on the right

  // GYROSCOPE
  Serial.println("##GYRO...##");
  Wire.begin(); // Initialization of the Wire library
  mpu6050.begin(); // Initialization of the Gyroscope library
  mpu6050.calcGyroOffsets(true); // The gyroscope is to be calibrated.
  Serial.println("\n##CALIBRATED!##");

}
// ######################## LOOP ########################
// runs as a loop:
void loop()
{
  int sensorvar = 0;
  int sensorvar2 = 0;
  int leftvar = 0;
  int rightvar = 0;

  // Define motor speed
  analogWrite(DCM1, motorvarR); // right motor
  analogWrite(DCM2, motorvarL); // left motor

  Serial.println("##LOOPSTART##");

  // start driving forwards
  // Motor 1 vorwärts
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // Motor 2 vorwärts
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Runs as long as neither the front IR sensor nor the bumpers have detected an obstacle.
  while ((sensorvar == 0) && (sensorvar2 == 0)) {
    sensorvar = ir_sensor(0); // Sensor 0 = front

    // Check distance left
    leftvar = ir_sensor(1);

    // If distance left too small
    while (leftvar == 1) {
      digitalWrite(12, HIGH); // LED red on
      // Correcting..
      turn(25, 1); // 25°
      digitalWrite(12, LOW); // LED red off
      leftvar = 0;
    }

    // Checke distance right
    rightvar = ir_sensor(2);
    // If distance right too small
    while (rightvar == 1) {
      digitalWrite(11, HIGH); // LED green on
      // Correcting..
      turn(-35, 2); // -35°
      digitalWrite(11, LOW); // LED green off
      rightvar = 0;
    }
    // If distance is large enough
    // Reset engine speed
    analogWrite(DCM1, motorvarR); // right motor
    analogWrite(DCM2, motorvarL); // left motor
    sensorvar2 = bumpers(); // Check front endstop
  }

  // Stop
  // Motor 1 off
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  // Motor 2 off
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  wait(50);

  // Differentiate between 90° and 180° rotation
  // Angles don't seem to be accurate on our sensor, so 90° in code is different than 90° in reality
  switch (degreecounter) {
    case 0:
      // turn 90 degrees
      digitalWrite(11, HIGH); // LED green on
      digitalWrite(12, LOW); // LED red off

      turn(170, 0); // 90° -> 170

      degreecounter++; // Set turn variable to 1 (add 1)
      digitalWrite(11, LOW); //LED green off
      break;
    case 1:
      // turn 180 degrees
      digitalWrite(11, LOW); // LED green off
      digitalWrite(12, HIGH); // LED red on

      turn(320, 0); // 180° -> 360-320

      degreecounter--; // set turn variable to 0 (subtract 1)
      digitalWrite(11, LOW); // LED green off
      digitalWrite(12, LOW); // LED red off
      break;
    default:
      // no reaction
      break;
  }
}
// ######################## IR_Sensor ########################
int ir_sensor(int sensornr) {

  int irtemp = 0; // Temporary sensor number
  int distancetemp = 0; // Temporary distance value
  int sumtemp = 0; // Temporary sum of distance values

  int repeat = 4; // Number of multiple readouts

  int entfrange = 70; // Distance range for readings

  // "Auswahl" des Sensors
  switch (sensornr)
  {
    // Vorne
    case 0:
      irtemp = ir1;
      // Less sensitive on the front, because otherwise with a slower motor on the left, it will trigger the front sensor first instead of the left side
      entfrange = 50;
      break;
    // left
    case 1:
      irtemp = ir2;
      break;
    // right
    case 2:
      irtemp = ir3;
      break;
    default:
      // not(hing) happening
      break;
  }
  distancetemp = analogRead(irtemp); // Transfer distance value of the sensor

  // Bypass error readings
  if ((distancetemp == 0) || (distancetemp == -1)) {
    distancetemp = 100;
  }
  // Multiple distance readings
  for (int i = 0; i < repeat; i++)
  {
    // Read distance
    distancetemp = analogRead(irtemp); // Transfer distance value of the sensor
    sumtemp += distancetemp; // Add distance value to total
    wait(ir_delay);
  }
  sumtemp = sumtemp / repeat; // Calculate mean value
  //distance < ca. 20cm +- X? (Inverted logic!)
  if ((ir_dist - entfrange < sumtemp) && (ir_dist + entfrange > sumtemp))
  {
    sumtemp = 0; // Reset total for next calculation
    distancetemp = analogRead(irtemp); // Transfer distance value of the sensor
    // Multiple reading of the distance
    for (int i = 0; i < repeat; i++)
    {
      distancetemp = analogRead(irtemp); // Transfer distance value of the sensor
      sumtemp = sumtemp + distancetemp; // Add distance value to total
      // Bypass error readings again
      if ((distancetemp == 0) || (distancetemp == -1)) {
        distancetemp = 100;
      }
      wait(ir_delay);
    }
    sumtemp = sumtemp / repeat; // Calculate mean value
    sumtemp = sumtemp + 50; // Bypass small sensor errors

    // If the mean value is smaller than the ir_dist... (Inverted logic!)
    if (sumtemp > ir_dist)
    {
      // Distance too close
      return 1;
    }
  }
  else {
    // Distance still OK
    return 0;
  }
}
// ######################## BUMPER ########################
bool bumpers() {
  int value = digitalRead(bumpPin1); // Reading the endstops
  int i = 0;

  if (value == 0) {

    // Hit something
    for (i = 0; i < 10; i++) {
      // Motor 1 off
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 off
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      return 1;
    }
  }
  else {
    // No contact (yet)
    return 0;
  }
}
// ######################## TURN ########################
void turn(int degree, int correct) {
  mpu6050.update(); // Update sensor values
  signed short int cur_angle = mpu6050.getAngleZ(); // Angle when starting the function

  // Define engine speed
  analogWrite(DCM1, motorvarR); // Right motor
  analogWrite(DCM2, motorvarL); // Left motor

  // Selection whether correction or "planned" rotation
  switch (correct)
  {
    case 0:
      // Motor 1 backwards
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      // Motor 2 backwards
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);

      delay(2000); // "Drive for 2 seconds"

      // Waiting time function spontaneously functionless shortly before deadline, therefore delay() above, instead.
      // Might fix this somewhen
      // For other functions there is no real difference without waiting time(), so it was omitted there
      //wait(2000);

      // Motor 1 off
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 off
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);

      // Clockwise rotation
      // Motor 1 backwards
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      // Motor 2 backwards
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);

      gyro = 0;
      // As long as (according to the gyroscope) the rotation is not yet complete
      while (gyro == 0) {
        gyro = gyro_sensor(cur_angle, degree);
      }

      // Stop
      // Motor 1 off
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 off
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);

      wait(500);
      break;
    case 1:
      // Left Sensor
      // Motor 1 off
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 off
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);

      // Set motor speed
      analogWrite(DCM1, motorvarR); // Right motor
      analogWrite(DCM2, motorvarL); // Left motor

      // Clockwise rotation
      // Motor 1 vforwards
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      // Motor 2 backwards
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);

      gyro = 0;
      // As long as (according to the gyroscope) the rotation is not yet complete
      while (gyro == 0) {
        gyro = gyro_sensor(cur_angle, degree);
      }
      // Stop
      // Motor 1 off
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 off
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      // Motor 1 forwards
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      // Motor 2 forwards
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      break;
    case 2:
      // Right sensor
      // Motor 1 off
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 off
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);

      // Set motor speed
      analogWrite(DCM1, motorvarR); // Right motor
      analogWrite(DCM2, motorvarL); // Left motor

      // Counter-clockwise turn
      // Motor 1 forwards
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      // Motor 2 backwards
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);

      gyro = 0;
      // As long as (according to the gyroscope) the rotation is not yet complete
      while (gyro == 0) {
        gyro = gyro_sensor(cur_angle, degree);
      }
      // Stop
      // Motor 1 off
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      // Motor 2 off
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);

      // Motor 1 forwards
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      // Motor 2 forwards
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      break;
    default:
      // Not(hing) happening
      break;
  }
  gyro = 0;
}
// ######################## GYRO ########################
// Check if rotation is complete, if yes return "1"
bool gyro_sensor(signed short int angle, int degree) {
  // Angle = Angle when starting the function
  // degree = Angle to be achieved (number of degrees)
  signed short int cur_angle = angle; // Current value

  if (degree < 0) {

    // Left rotation (lets the gyroscope output negative values)
    // Wait until the gyroscope has detected a complete X° rotation (<-180)
    while (cur_angle <= angle + degree) {
      mpu6050.update(); // Update sensor values
      cur_angle = mpu6050.getAngleZ(); // Transfer Z axis values (rotation angle)
      wait(gyro_delay);
    }
  }
  else {
    // Clockwise rotation (allows the gyroscope to output positive values)
    // Wait until the gyroscope has detected a complete X° rotation (<-180)
    while (cur_angle >= angle - degree) {
      mpu6050.update(); // Update sensor values
      cur_angle = mpu6050.getAngleZ(); // Transfer Z axis values (rotation angle)
      wait(gyro_delay);
    }
  }
  // Turn(ing) finished
  return 1;
}
// ######################## WAIT ########################
// Replaces delay() with millis()
void wait(unsigned short duration) {
  unsigned short currentMillis = 0; // Current time
  unsigned short startMillis = (unsigned short)millis(); // Starting point of the function in ms

  // Execute as long as the duration has not elapsed.
  while (currentMillis - startMillis <= duration) {
    currentMillis = (unsigned short)millis(); // Current time in ms
  }
}
