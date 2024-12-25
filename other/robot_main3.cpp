#include <Arduino.h>
#include <QTRSensors.h>
#include <SPI.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <MFRC522.h>




#define SS_PIN  5  // ESP32 pin GPIO5 
#define RST_PIN 25 // ESP32 pin GPIO27

MFRC522 rfid(SS_PIN, RST_PIN);

// QTR Sensor Setup
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// PID Parameters
float Kp = 0.1;   // Proportional constant
float Ki = 0.0;   // Integral constant
float Kd = 0.0;   // Derivative constant

int lastError = 0;
float integral = 0;

// Updated Motor Pins
int motor1Pin1 = 15; // Motor A pin 1
int motor1Pin2 = 2; // Motor A pin 2
int enable1Pin = 17;  // Motor A enable

int motor2Pin1 = 0; // Motor B pin 1
int motor2Pin2 = 4; // Motor B pin 2
int enable2Pin = 22; // Motor B enable

// PWM properties
const int freq = 30000;
const int pwmChannel1 = 0; // Channel for Motor A
const int pwmChannel2 = 1; // Channel for Motor B
const int resolution = 8;
const int maxSpeed = 240; // Maximum motor speed


const int trigPin = 26;
const int echoPin = 21;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

void driveMotors(int leftSpeed, int rightSpeed);

void setup()
{
  Serial.begin(9600);
  Serial.println("Initializing...");

  // Motor Pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Configure PWM
  ledcAttachPin(enable1Pin, pwmChannel1);
  ledcSetup(pwmChannel1, freq, resolution);

  ledcAttachPin(enable2Pin, pwmChannel2);
  ledcSetup(pwmChannel2, freq, resolution);

  // QTR Sensor Setup
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){36,39,34,35,32,33}, SensorCount);
  qtr.setEmitterPin(2);

  // Calibration Step
  Serial.println("Calibrating sensors. Move the robot over the line...");
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
    if (i % 50 == 0) // Periodically update the user
    {
      Serial.print("Calibration progress: ");
      Serial.print((i / 400.0) * 100);
      Serial.println("%");
    }
  }
  Serial.println("Calibration complete!");

  // Print calibration values
  Serial.println("Calibration minimum values:");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.println("Calibration maximum values:");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();

//----------- HC-SR04 SENSOR CODE --------
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.println("ready...");
//-----------------------------------------

//------------- RFID READER ---------------
  SPI.begin(); // init SPI bus
  rfid.PCD_Init(); // init MFRC522

  Serial.println("Tap an RFID/NFC tag on the RFID-RC522 reader");
//------------------------------------------
  delay(1000); // Small delay before starting the main loop
}

void loop()
{
//---------- RFID READER---------------
  if (rfid.PICC_IsNewCardPresent()) { // new tag is available
    if (rfid.PICC_ReadCardSerial()) { // NUID has been readed
      MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
      Serial.print("RFID/NFC Tag Type: ");
      Serial.println(rfid.PICC_GetTypeName(piccType));

      // print UID in Serial Monitor in the hex format
      Serial.print("UID:");
      for (int i = 0; i < rfid.uid.size; i++) {
        Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
        Serial.print(rfid.uid.uidByte[i], HEX);
      }
      Serial.println();

      rfid.PICC_HaltA(); // halt PICC
      rfid.PCD_StopCrypto1(); // stop encryption on PCD
    }
  }
//---------------------------------
//-------------ULTRASONIC SENSOR -------------
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
//----------------------------------------






  // Read calibrated sensor values
  uint16_t position = qtr.readLineBlack(sensorValues);

/*     if ((sensorValues[0] > 750) && (sensorValues[1] > 750) && (sensorValues[2] > 750)&& (sensorValues[3] > 750)&& (sensorValues[4] > 750)&& (sensorValues[5] > 750))
  {
    driveMotors(0,0);
    delay(2000);
    goDirection(0);
    return;
  } */


  // Calculate error
  int error = position - 2500; // Assuming center is 2500 for a 6-sensor setup

  // Calculate PID terms
  float proportional = error;
  integral += error;
  float derivative = error - lastError;

  // PID correction
  float correction = (Kp * proportional) + (Ki * integral) + (Kd * derivative);

  // Limit correction
  
  correction = constrain(correction, -maxSpeed, maxSpeed);
  int minSpeed = 170;

  // Motor speeds
  int leftMotorSpeed = (maxSpeed + correction);
  int rightMotorSpeed = (maxSpeed - correction);

  // Ensure motor speeds are within range
  float rangeFactor= ((float)maxSpeed-(float)minSpeed+1)/(float)maxSpeed;
  leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);
leftMotorSpeed = minSpeed + (((float)maxSpeed-(float)minSpeed)/((float)maxSpeed-0))*((float)leftMotorSpeed-0);
rightMotorSpeed = minSpeed + (((float)maxSpeed-(float)minSpeed)/((float)maxSpeed-0))*((float)rightMotorSpeed-0);
   // Slow down and stop if there is an obstacle
  if (10 > distanceCm && distanceCm > 5){
    leftMotorSpeed= map(distanceCm, 5, 10, 80, leftMotorSpeed);
    rightMotorSpeed= map(distanceCm, 5, 10, 80, rightMotorSpeed);
  } else if (distanceCm < 5){
    leftMotorSpeed= 0;
    rightMotorSpeed= 0;
  }; 

  // Drive motors
  driveMotors(leftMotorSpeed, rightMotorSpeed);

  // Update last error
  lastError = error;

  // Print debug information
  Serial.print("Position: ");
  Serial.print(position);
  Serial.print("\tError: ");
  Serial.print(error);
  Serial.print("\tCorrection: ");
  Serial.print(correction);
  Serial.print("\tLeft Speed: ");
  Serial.print(leftMotorSpeed);
  Serial.print("\tRight Speed: ");
  Serial.print(rightMotorSpeed);
  Serial.print("\tdistanceCm: ");
  Serial.println(distanceCm);
}

void driveMotors(int leftSpeed, int rightSpeed)
{
  // Left Motor
  if (leftSpeed > 0)
  {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  }
  else
  {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  }
  ledcWrite(pwmChannel1, abs(leftSpeed));

  // Right Motor
  if (rightSpeed > 0)
  {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  }
  else
  {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }
  ledcWrite(pwmChannel2, abs(rightSpeed));
}

void goDirection(int dir){
  switch (dir)
  {
  case 0://right 5s
    driveMotors(200,-200);
    delay(5000);
    break;

  case 1://left
    driveMotors(-200,200);
    delay(5000);
    break;

  default:
    break;
  }
}