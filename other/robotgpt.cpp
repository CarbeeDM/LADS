/*******************
 * Example code showing:
 *  - PID line-following
 *  - RFID detection as nodes
 *  - Intersection detection (T/L-turn)
 *  - Simple map building with nodes/edges
 *
 * Hardware:
 *  - ESP32 (with WiFi / Firebase capability if needed)
 *  - QTR-6 sensor array
 *  - HC-SR04 ultrasonic sensor (optional obstacle check)
 *  - MFRC522 RFID module
 *  - 2 DC motors w/ L298 or similar driver
 *******************/

#include <Arduino.h>
#include <QTRSensors.h>
#include <SPI.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <MFRC522.h>

// ------------------- RFID PINS ----------------------
#define SS_PIN  5   // ESP32 pin for MFRC522 SS/SDA
#define RST_PIN 25  // ESP32 pin for MFRC522 RST
MFRC522 rfid(SS_PIN, RST_PIN);  // Create MFRC522 instance

// ------------------- QTR SENSOR SETUP ----------------
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// ------------------- PID PARAMETERS ------------------
float Kp = 0.1;
float Ki = 0.000; //TODO
float Kd = 0.;

int lastError = 0;
float integral = 0;

// ------------------- MOTOR PINS ----------------------
int motor1Pin1 = 15; 
int motor1Pin2 = 2;  
int enable1Pin = 17; 

int motor2Pin1 = 0;  
int motor2Pin2 = 4;  
int enable2Pin = 22; 

// ------------------- MOTOR PWM PROPERTIES ------------
const int freq = 30000;
const int pwmChannel1 = 0; 
const int pwmChannel2 = 1; 
const int resolution = 8;
const int maxSpeed = 240;//TODO

// ------------------- ULTRASONIC PINS -----------------
const int trigPin = 26;
const int echoPin = 21;

#define SOUND_SPEED 0.034

// ------------------- MAPPING STRUCTURES --------------
// Each Node can be an intersection or an RFID tag
struct Node {
  String uid;               // e.g. "RFID_XXYY" or "INT_1", "INT_2"
  bool isRFID;
  // adjacency: which other nodes connect to this node?
  // For simplicity, store parallel arrays of neighbors & costs (time in ms)
  // or you could store a vector of structs like {neighbor, cost}.
  std::vector<int> neighbors;
  std::vector<unsigned long> costs;  // travel time in ms
};

// We'll store our nodes in a global vector
std::vector<Node> graph;

// keep track of the index of the current node in the graph
int currentNodeIndex = -1;
int startNodeIndex = -1;

// Used to generate unique intersection names
int intersectionCounter = 0;

// track time for measuring cost between nodes
unsigned long segmentStartTime = 0;

// helper function to find a node by uid
int findNodeIndex(const String& uid) {
  for (size_t i = 0; i < graph.size(); i++) {
    if (graph[i].uid == uid) {
      return i;
    }
  }
  return -1; // not found
}

bool doesEdgeExist(const int from, const int to){
    for (int i=0; i< graph[from].neighbors.size();i++){
        if(graph[from].neighbors[i]==to){
            return true;
        }
    }
    return false;
}

// Add a new node to the graph
int addNode(const String& uid, bool isRFID) {
  Node n;
  n.uid = uid;
  n.isRFID = isRFID;
  graph.push_back(n);
  return graph.size() - 1; // return the index of the newly added node
}

// record edge in both directions
void addEdge(int from, int to, unsigned long cost) {
  if(doesEdgeExist(from,to)){return;}

  graph[from].neighbors.push_back(to);
  graph[from].costs.push_back(cost);

  // if you want an undirected graph, add reverse as well
  graph[to].neighbors.push_back(from);
  graph[to].costs.push_back(cost);
}

// ------------------- FUNCTION DECLARATIONS -----------
void driveMotors(int leftSpeed, int rightSpeed);
void handleLineFollow();
void checkForRFID();
bool detectIntersection();
String createIntersectionID();
void handleIntersectionIfNeeded();
void stopAndMapNewNode(const String& nodeUID, bool isRFID);

// --------------- SETUP -------------------------------
void setup()
{
  Serial.begin(9600);
  Serial.println("Initializing...");

  // ----- MOTOR SETUP -----
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  ledcAttachPin(enable1Pin, pwmChannel1);
  ledcSetup(pwmChannel1, freq, resolution);

  ledcAttachPin(enable2Pin, pwmChannel2);
  ledcSetup(pwmChannel2, freq, resolution);

  // ----- QTR SENSOR SETUP -----
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){36,39,34,35,32,33}, SensorCount);
  qtr.setEmitterPin(2);

  // ----- CALIBRATION -----
  Serial.println("Calibrating sensors. Move the robot over the line...");
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
    if (i % 50 == 0) {
      Serial.print("Calibration progress: ");
      Serial.print((i / 400.0) * 100);
      Serial.println("%");
    }
  }
  Serial.println("Calibration complete!");

  // Print calibration values (optional)
  Serial.println("Calibration minimum values:");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.println("Calibration maximum values:");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // ----- ULTRASONIC SETUP -----
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.println("ready...");

  // ----- RFID READER SETUP -----
  SPI.begin(); 
  rfid.PCD_Init(); 
  Serial.println("Tap an RFID/NFC tag on the RFID-RC522 reader");

  delay(1000); 

  // OPTIONAL: If you know the robot starts on an RFID, read it now:
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    // We'll treat this as the "start node"
    String startUID = "";
    for (int i = 0; i < rfid.uid.size; i++) {
      startUID += String(rfid.uid.uidByte[i], HEX);
    }
    startUID.toUpperCase();
    startNodeIndex = addNode(startUID, true);
    currentNodeIndex = startNodeIndex;
    Serial.print("Starting node: ");
    Serial.println(startUID);
  } else {
    // If not on RFID, create a default "START" node
    startNodeIndex = addNode("START", false);
    currentNodeIndex = startNodeIndex;
  }
  segmentStartTime = millis(); // start measuring travel time from the initial node
}

// --------------- MAIN LOOP ---------------------------
void loop()
{
  // 1. Check for new RFID and create a node if found
  checkForRFID();

  // 2. Check for intersection (T or L)
  handleIntersectionIfNeeded();

  // 3. Perform line following to stay on track
  handleLineFollow();

  // 4. (Optional) You could check if you've returned to the start node with no new paths to explore:
  //    if we've come back to 'startNodeIndex' and all edges visited, we could end mapping.
  //    That logic is omitted here for brevity.
}

// ------------------- HELPER FUNCTIONS ----------------

// Read the QTR line sensors and do PID correction
void handleLineFollow()
{
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 2500; // center ~ 2500 for 6 sensors
  float proportional = error;
  integral += error;
  float derivative = error - lastError;
  float correction = (Kp * proportional) + (Ki * integral) + (Kd * derivative);
  correction = constrain(correction, -maxSpeed, maxSpeed);

  // Example approach: base speed around ~ (maxSpeed - something)
  // then +/- correction
  int minSpeed = 170; 
  int leftMotorSpeed  = maxSpeed + correction;
  int rightMotorSpeed = maxSpeed - correction;

  leftMotorSpeed  = constrain(leftMotorSpeed, 0, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);

float rangeFactor= ((float)maxSpeed-(float)minSpeed+1)/(float)maxSpeed;
  
leftMotorSpeed = minSpeed + (rangeFactor)*((float)leftMotorSpeed);
rightMotorSpeed = minSpeed + (rangeFactor)*((float)rightMotorSpeed);

  // Drive motors
  driveMotors(leftMotorSpeed, rightMotorSpeed);

  lastError = error;
}

void checkForRFID()
{
  if (rfid.PICC_IsNewCardPresent()) {
    if (rfid.PICC_ReadCardSerial()) {
      // Build the UID string
      String newUID = "";
      for (int i = 0; i < rfid.uid.size; i++) {
        newUID += String(rfid.uid.uidByte[i], HEX);
      }
      newUID.toUpperCase();

      // If we haven't seen this RFID before, treat it as a new node
      int idx = findNodeIndex(newUID);
      if (idx == -1) {
        // This is a new RFID node
        stopAndMapNewNode(newUID, true);
      } else {
        // We already know this RFID: optional logic if you want to stop or do something
        // e.g., if it's the start node again and no more branches, you might end mapping
        // For now, do nothing
      }

      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }
  }
}

// Detect an intersection by checking if all sensors read black for some time
bool detectIntersection() {
  // Very simplistic approach: if all sensors above a certain threshold => wide black
  // You might also check sensorValues individually or do more advanced checks
  int threshold = 500; 
  bool allBlack = true;
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < threshold) {
        Serial.print(i + ": " + (sensorValues[i] < threshold));
      allBlack = false;
    }
  }
  Serial.println();
  return allBlack;
}

// If we detect intersection, stop, map it, pick direction (hand-on-wall or other)
void handleIntersectionIfNeeded()
{
  if (detectIntersection()) {
    // We'll label it a new intersection node
    String intersectionUID = createIntersectionID();
    stopAndMapNewNode(intersectionUID, false);

    // Example: do a "hand on left" rule check
    // (In reality, you'd do more robust check of left/straight/right lines.)
    // For simplicity, let's do a small left pivot check
    driveMotors(0,0);
    delay(200); // short stop
    bool turnLeft = false; // e.g. if you'd sense line on left side, set true

    // Pseudo-check: if leftmost sensor is also black => there's a left branch
    if (sensorValues[0] > 600) {
      turnLeft = true;
    }

    if (turnLeft) {
      // Turn left
      driveMotors(-220,220);
      delay(800); // pivot left for ~500ms or until line reacquired
    } else {
      // else go straight
      // or check right
      // For demonstration, assume straight is main path
      driveMotors(220,-220);
      delay(800); // small forward
    }
    // Then resume line follow
  }
}

String createIntersectionID() {
  // create something like "INT_1", "INT_2", etc.
  String id = "INT_";
  id += String(intersectionCounter++);
  return id;
}

// Called whenever we discover a new node (RFID or intersection)
// stops the robot briefly, records an edge from currentNode -> newNode
void stopAndMapNewNode(const String& nodeUID, bool isRFID)
{
  // stop motors
  driveMotors(0,0);
  delay(200);

  unsigned long now = millis();
  unsigned long travelTime = now - segmentStartTime;

  // create the new node if it doesn't exist
  int newNodeIndex = findNodeIndex(nodeUID);
  if (newNodeIndex == -1) {
    newNodeIndex = addNode(nodeUID, isRFID);
    Serial.print("New node discovered: ");
    Serial.println(nodeUID);
  } else {
    Serial.print("Revisiting node: ");
    Serial.println(nodeUID);
  }

  // record the edge from current node to new node
  addEdge(currentNodeIndex, newNodeIndex, travelTime);

  // update current node
  currentNodeIndex = newNodeIndex;

  // reset travel timer
  segmentStartTime = millis();

  // optional: small delay for stability
  delay(300);
}

// Simple motor driver function
void driveMotors(int leftSpeed, int rightSpeed)
{
  // Left Motor
  if (leftSpeed >= 0) {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  } else {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  }
  ledcWrite(pwmChannel1, abs(leftSpeed));

  // Right Motor
  if (rightSpeed >= 0) {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  } else {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }
  ledcWrite(pwmChannel2, abs(rightSpeed));
}