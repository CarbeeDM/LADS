#include <Arduino.h>
#include <QTRSensors.h>

// This example is designed for use with six analog QTR sensors. These
// reflectance sensors should be connected to analog pins A0 to A5. The
// sensors' emitter control pin (CTRL or LEDON) can optionally be connected to
// digital pin 2, or you can leave it disconnected and remove the call to
// setEmitterPin().
//
// The main loop of the example reads the raw sensor values (uncalibrated). You
// can test this by taping a piece of 3/4" black electrical tape to a piece of
// white paper and sliding the sensor across it. It prints the sensor values to
// the serial monitor as numbers from 0 (maximum reflectance) to 1023 (minimum
// reflectance).

QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// Motor A
int motorAPin1 = 4; 
int motorAPin2 = 5; 
int enableAPin = 6; 

// Motor B
int motorBPin1 = 7; 
int motorBPin2 = 8; 
int enableBPin = 9; 

int speed=200;

void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){34, 35, 32, 33, 25, 26}, SensorCount);
  qtr.setEmitterPin(2);

  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);
  pinMode(enableAPin, OUTPUT);

  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);
  pinMode(enableBPin, OUTPUT);

for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtr.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }


  Serial.begin(9600);

  for (int i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  Serial.println("setup is done");
  delay(5000);

}

void forward()
{
  analogWrite(enableAPin, speed);
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, HIGH);

  analogWrite(enableBPin, speed);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, HIGH);
}

void right()
{
  analogWrite(enableAPin, speed);
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, HIGH);
  
  analogWrite(enableBPin, speed);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, LOW);
}

void left()
{
  analogWrite(enableAPin, speed);
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, LOW);
  
  analogWrite(enableBPin, speed);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, HIGH);
}

void stop()
{
  analogWrite(enableAPin, speed);
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, LOW);
  
  analogWrite(enableBPin, speed);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, LOW);
}

int line_see(uint16_t sV[], int limit)
{
  if (sV[0]<limit && sV[0]<limit && sV[0]<limit && sV[0]<limit && sV[0]<limit && sV[0]<limit){

  }
  return 1;
}

void loop()
{

  // read raw sensor values
  qtr.read(sensorValues);

  // print the sensor values as numbers from 0 to 1023, where 0 means maximum
  // reflectance and 1023 means minimum reflectance
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(500);
}