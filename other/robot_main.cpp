/*
 * --------------------------------------------------------------------------------------------------------------------
 * Example sketch/program showing how to read data from a PICC to serial.
 * --------------------------------------------------------------------------------------------------------------------
 * This is a MFRC522 library example; for further details and other examples see: https://github.com/miguelbalboa/rfid
 * 
 * Example sketch/program showing how to read data from a PICC (that is: a RFID Tag or Card) using a MFRC522 based RFID
 * Reader on the Arduino SPI interface.
 * 
 * When the Arduino and the MFRC522 module are connected (see the pin layout below), load this sketch into Arduino IDE
 * then verify/compile and upload it. To see the output: use Tools, Serial Monitor of the IDE (hit Ctrl+Shft+M). When
 * you present a PICC (that is: a RFID Tag or Card) at reading distance of the MFRC522 Reader/PCD, the serial output
 * will show the ID/UID, type and any data blocks it can read. Note: you may see "Timeout in communication" messages
 * when removing the PICC from reading distance too early.
 * 
 * If your reader supports it, this sketch/program will read all the PICCs presented (that is: multiple tag reading).
 * So if you stack two or more PICCs on top of each other and present them to the reader, it will first output all
 * details of the first and then the next PICC. Note that this may take some time as all data blocks are dumped, so
 * keep the PICCs at reading distance until complete.
 * 
 * @license Released into the public domain.
 * 
 * Typical pin layout used:
 * -----------------------------------------------------------------------------------------
 *             MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
 *             Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
 * Signal      Pin          Pin           Pin       Pin        Pin              Pin
 * -----------------------------------------------------------------------------------------
 * RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
 * SPI SS      SDA(SS)      10            53        D10        10               10
 * SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
 * SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
 * SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
 *
 * More pin layouts for other boards can be found here: https://github.com/miguelbalboa/rfid#pin-layout
 */

// #include <SPI.h>
// #include <MFRC522.h>

// #define RST_PIN         22          // Configurable, see typical pin layout above
// #define SS_PIN          5         // Configurable, see typical pin layout above

// MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance

// void setup() {
// 	Serial.begin(9600);		// Initialize serial communications with the PC
// 	while (!Serial);		// Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
// 	SPI.begin();			// Init SPI bus
// 	mfrc522.PCD_Init();		// Init MFRC522
// 	delay(4);				// Optional delay. Some board do need more time after init to be ready, see Readme
// 	mfrc522.PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details
// 	Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
// }

// void loop() {
// 	// Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
// 	if ( ! mfrc522.PICC_IsNewCardPresent()) {
// 		return;
// 	}

// 	// Select one of the cards
// 	if ( ! mfrc522.PICC_ReadCardSerial()) {
// 		return;
// 	}

// 	// Dump debug info about the card; PICC_HaltA() is automatically called
// 	mfrc522.PICC_DumpToSerial(&(mfrc522.uid));
// }

#include <Arduino.h>
#include <QTRSensors.h>
#include <SPI.h>

// QTR Sensor Setup
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// PID Parameters
float Kp = 0.2;   // Proportional constant
float Ki = 0.0;   // Integral constant
float Kd = 1.0;   // Derivative constant

int lastError = 0;
float integral = 0;

// Updated Motor Pins
int motor1Pin1 = 16; // Motor A pin 1
int motor1Pin2 = 17; // Motor A pin 2
int enable1Pin = 4;  // Motor A enable

int motor2Pin1 = 15; // Motor B pin 1
int motor2Pin2 = 2; // Motor B pin 2
int enable2Pin = 13; // Motor B enable

// PWM properties
const int freq = 30000;
const int pwmChannel1 = 0; // Channel for Motor A
const int pwmChannel2 = 1; // Channel for Motor B
const int resolution = 8;
const int maxSpeed = 255; // Maximum motor speed


const int trigPin = 12;
const int echoPin = 14;

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
  qtr.setSensorPins((const uint8_t[]){34, 35, 32, 33, 25, 26}, SensorCount);
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

  delay(1000); // Small delay before starting the main loop
}
// if dist 30>10, (dist-10)*5=maxspeed
void loop()
{
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


  // Read calibrated sensor values
  uint16_t position = qtr.readLineBlack(sensorValues);

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

  // Motor speeds
  int leftMotorSpeed = maxSpeed - correction;
  int rightMotorSpeed = maxSpeed + correction;

  // Ensure motor speeds are within range
  leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);

  // Slow down and stop if there is an obstacle
  if (30 > distanceCm && distanceCm > 10){
    leftMotorSpeed= map(distanceCm, 10, 30, 80, leftMotorSpeed);
    rightMotorSpeed= map(distanceCm, 10, 30, 80, rightMotorSpeed);
  } else if (distanceCm < 10){
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