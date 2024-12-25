

  //Rui Santos & Sara Santos - Random Nerd Tutorials
  //Complete project details at https://RandomNerdTutorials.com/esp32-dc-motor-l298n-motor-driver-control-speed-direction/
  //Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.  
  //The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

#include <Arduino.h>
#include <L298N.h>
// Motor A
/* int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int enable1Pin = 14;  */

int motor1Pin1 = 15; // Motor A pin 1
int motor1Pin2 = 2; // Motor A pin 2
int enable1Pin = 1;  // Motor A enable


// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

void setup() {
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  
  // configure LEDC PWM
  ledcAttachPin(enable1Pin, pwmChannel);
ledcSetup(pwmChannel, freq, resolution);
  Serial.begin(9600);

  // testing
  Serial.print("Testing DC Motor...");
  analogWrite(enable1Pin, 200);
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 
}

void loop() {
  // Move the DC motor forward at maximum speed
  dutyCycle = 80;
  Serial.println("Moving Forward");
  analogWrite(enable1Pin, dutyCycle);

    while (dutyCycle <= 255){
        analogWrite(enable1Pin, dutyCycle);   
        Serial.print("Forward with duty cycle: ");
        Serial.println(dutyCycle);
        dutyCycle = dutyCycle + 5;
        delay(200);
    }

  



}