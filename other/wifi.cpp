#include <Arduino.h>
#include <QTRSensors.h>
#include <SPI.h>

#include <MFRC522.h>

#include <WiFi.h>
#include<Firebase_ESP_Client.h>


#define WIFI_SSID "cece"
#define WIFI_PASSWORD "123123aa"
#define API_KEY "AIzaSyD3BE-hRNRFyzfK1d98scXM5zG5w5iX3fw"
#define DATABASE_URL "https://ladsceng483-default-rtdb.europe-west1.firebasedatabase.app/"

#define LED1_PIN 12
#define LED2_PIN 14
#define LDR_PIN 36

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis=0;
bool signupOK = false;
int ldrData =0;
float voltage =0.0;

const int trigPin = 26;
const int echoPin = 21;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;


void setup(){
pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT);

Serial.begin(9600);
WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
Serial.print("connecting to wifi");
while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");delay(300);
}
Serial.println();
Serial.print("connected IP: ");
Serial.println(WiFi.localIP());
Serial.println();

config.api_key=API_KEY;
config.database_url=DATABASE_URL;
if(Firebase.signUp(&config,&auth,"","")){
    Serial.println("signup OK");
    signupOK=true;
} else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
}

//config.token_status_callback = tokenStatusCallback;
Firebase.begin(&config, &auth);
Firebase.reconnectWiFi(true);
}

void loop(){
if(Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 1000 ||  sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    // store sensor data to a rtdb
    ldrData = 25;
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
  
  // Convert to inches
  distanceInch = distanceCm * CM_TO_INCH;
  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);
    if(Firebase.RTDB.setInt(&fbdo, "Sensor/ldr_data", ldrData)){
        Serial.println(); Serial.print(ldrData);
        Serial.print(" - successfully saved to: "); Serial.print(fbdo.dataPath());
        Serial.println(fbdo.dataType());
    }else{
        Serial.println("FAILED");
        Serial.println(fbdo.errorReason());
    }

    if(Firebase.RTDB.setFloat(&fbdo, "Sensor/voltage", distanceCm)){
        Serial.println(); Serial.print(voltage);
        Serial.print(" - successfully saved to: "); Serial.print(fbdo.dataPath());
        Serial.println(fbdo.dataType());
    }else{
        Serial.println("FAILED");
        Serial.println(fbdo.errorReason());
    }
    delay(1000);

}
}