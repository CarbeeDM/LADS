#include <Arduino.h>
#include <QTRSensors.h>
#include <SPI.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <MFRC522.h>
#include <stack>

// ------------------- RFID PINS ----------------------
#define SS_PIN  5
#define RST_PIN 25
MFRC522 rfid(SS_PIN, RST_PIN);  // Create MFRC522 instance

// ------------------- QTR SENSOR SETUP ---------------
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
bool prevSensorFirst=true;
bool prevSensorLast=true;
// ------------------- PID PARAMETERS ------------------
float Kp = 0.1;
float Ki = 0.0;
float Kd = 0.0;

int lastError = 0;
float integral = 0;

// ------------------- MOTOR PINS ---------------------
int motor1Pin1 = 15;
int motor1Pin2 = 2;
int enable1Pin = 17;

int motor2Pin1 = 0;
int motor2Pin2 = 4;
int enable2Pin = 22;

// ------------------- MOTOR PWM ----------------------
const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
const int maxSpeed = 220;

// ------------------- ULTRASONIC ---------------------
const int trigPin = 26;
const int echoPin = 21;
#define SOUND_SPEED 0.034

// ------------------- MAPPING STRUCTURES -------------
struct Node {
  String uid;         // e.g. "RFID_XXYY" or "INT_1", "INT_2"
  bool isRFID;
  std::vector<int> neighbors;
  std::vector<unsigned long> costs;  // travel time
};

std::vector<Node> graph;
int currentNodeIndex    = -1;
int startNodeIndex      = -1;
int intersectionCounter = 0;
unsigned long segmentStartTime = 0;

// We'll keep a stack to track intersections for backtracking
std::stack<int> nodeStack;

// For edges visited, we track usage:
struct EdgeVisit {
  int nodeA;
  int nodeB;
  bool visited;
};
std::vector<EdgeVisit> visitedEdges;

// -------------- RFID & START LOGIC ------------------
bool startNodeSet          = false;     // haven't chosen a start node yet
String startNodeUID        = "";        // store the actual UID of the start node
int timesAtStartNode       = 0;         // how many times we've revisited start node
unsigned long ignoreRFIDUntil = 0;      // ignore RFID reads until this time
unsigned long ignoreIntersectionUntil = 0;
String lastNodeSeen = "";               // the last node seen, to prevent double reads
// --------------- FUNCTION DECLARATIONS ---------------
int  findNodeIndex(const String& uid);
int  addNode(const String& uid, bool isRFID);
void addEdge(int from, int to, unsigned long cost);

bool edgeIsVisited(int a, int b);
void markEdgeVisited(int a, int b);

void driveMotors(int leftSpeed, int rightSpeed);
void handleLineFollow();
void handleIntersectionIfNeeded();
bool detectIntersection();
char detectIntersectionType();
String createIntersectionID(char type);

void checkForRFID(); // always scanning in mapping mode
void stopAndMapNewNode(const String& nodeUID, bool isRFID);

// "hand-on-wall" approach
void turnLeft();
void turnRight();
void turnBack();
void goStraight();

// -----------------------------------------------------
//                     SETUP
// -----------------------------------------------------
void setup()
{
  Serial.begin(9600);
  Serial.println("Initializing...");

  // MOTOR SETUP
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  ledcAttachPin(enable1Pin, pwmChannel1);
  ledcSetup(pwmChannel1, freq, resolution);

  ledcAttachPin(enable2Pin, pwmChannel2);
  ledcSetup(pwmChannel2, freq, resolution);

  // QTR SENSOR SETUP
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){36,39,34,35,32,33}, SensorCount);
  qtr.setEmitterPin(2);

  Serial.println("Calibrating sensors. Move the robot over the line...");
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
    if (i % 50 == 0) {
      Serial.print("Calibration progress: ");
      Serial.print((i / 200.0) * 100);
      Serial.println("%");
    }
  }
  Serial.println("Calibration complete!");

  // ULTRASONIC
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // RFID READER
  SPI.begin();
  rfid.PCD_Init();
  Serial.println("Robot starts moving and exploring. First RFID it sees will be the Start Node.");
  delay(1000);

  // No start node assigned. The robot will immediately drive & do intersection logic.
  // The first time it sees an RFID, that becomes the start node.
  segmentStartTime = millis();
}

// -----------------------------------------------------
//                     LOOP
// -----------------------------------------------------
// Add this global variable at the top of the code:
unsigned long lastStartNodeScanTime = 0;  // Track the last time the start node was scanned

// In the `loop` function, update the logic for handling the start node:
void loop()
{
    // Always do line following + intersection detection
    if(millis() > ignoreIntersectionUntil ){
    handleIntersectionIfNeeded();
    }

    handleLineFollow();

    // Check if RFID reading is allowed
    if (millis() > ignoreRFIDUntil) {
        checkForRFID();
    }

    // If we have a start node set, check if we’re back on it
    if (startNodeSet && currentNodeIndex == startNodeIndex && nodeStack.size() <= 1)
    {
        // Check for delay since the last scan
        if (millis() - lastStartNodeScanTime < 2000) {
            // Ignore if it's been less than 2 seconds since the last scan
            return;
        }

        // Calculate the time elapsed since the last visit to the start node
        unsigned long elapsedTime = millis() - lastStartNodeScanTime;

        // Update the last scan time
        lastStartNodeScanTime = millis();

        // Increment timesAtStartNode
        timesAtStartNode++;
        Serial.print("Arrived back at start node (");
        Serial.print(startNodeUID);
        Serial.print("). timesAtStartNode = ");
        Serial.print(timesAtStartNode);
        Serial.print(", Time since last visit: ");
        Serial.print(elapsedTime);
        Serial.println(" ms");

        // If timesAtStartNode == 2 => done
        if (timesAtStartNode >= 2)
        {
            Serial.println("We have visited the start node a second time. Mapping complete!");
            driveMotors(0, 0);

            // Optionally: check all edges visited if desired
            // For now, we just stop
            while (true) {
                // Robot is done. Wait for commands or do nothing
            }
        }
        else {
            // First time is effectively a no-op
            Serial.println("First time returning to start. Continue exploring...");
        }
    }
}

// In the `checkForRFID` function, ensure the `ignoreRFIDUntil` logic still applies as it does now:
void checkForRFID()
{
  // Try reading an RFID, if present
  if (rfid.PICC_IsNewCardPresent()) {
    // Ignore further RFID scans for 2 seconds
    ignoreRFIDUntil = millis() + 2000;

    if (rfid.PICC_ReadCardSerial()) {
      // Build UID
      String newUID = "";
      for (int i = 0; i < rfid.uid.size; i++) {
        newUID += String(rfid.uid.uidByte[i], HEX);
      }
      newUID.toUpperCase();
      if (newUID == lastNodeSeen) {
        return;
      }
      // Handle first RFID detection as Start Node
      if (!startNodeSet)
      {
        startNodeUID = newUID;
        startNodeSet = true;

        // Clear out old intersections from stack
        while (!nodeStack.empty()) {
          nodeStack.pop();
        }

        // Create the start node
        startNodeIndex = addNode(startNodeUID, true);
        currentNodeIndex = startNodeIndex;

        // Push onto the stack
        nodeStack.push(currentNodeIndex);

        timesAtStartNode = 1;  // First visit
        Serial.print("First RFID found. Setting start node: ");
        Serial.println(startNodeUID);

        // Ignore further RFID scans for 2 seconds
       
      }
      else
      {
        // Existing Start Node Scans
        if (newUID == startNodeUID) {
          stopAndMapNewNode(newUID, true);
          Serial.println("Detected start node RFID again. Handled in loop.");
        }
        else {
          // Handle other RFIDs
            stopAndMapNewNode(newUID, true);
            Serial.print("New RFID discovered: ");
            Serial.println(newUID);
          
        }

      }

      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }
  }
}


void handleLineFollow()
{
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 2500;
  float proportional = error;
  integral += error;
  float derivative = error - lastError;

  float correction = (Kp * proportional) + (Ki * integral) + (Kd * derivative);
  correction = constrain(correction, -maxSpeed, maxSpeed);

  int minSpeed = 170;
  int leftMotorSpeed  = maxSpeed + correction;
  int rightMotorSpeed = maxSpeed - correction;

  leftMotorSpeed  = constrain(leftMotorSpeed, 0, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);

  float rangeFactor = (float)(maxSpeed - minSpeed) / (float)maxSpeed;
  leftMotorSpeed  = minSpeed + rangeFactor * leftMotorSpeed;
  rightMotorSpeed = minSpeed + rangeFactor * rightMotorSpeed;

  driveMotors(leftMotorSpeed, rightMotorSpeed);
  lastError = error;
}

void handleIntersectionIfNeeded()
{
  if (detectIntersection()) {
    ignoreIntersectionUntil = millis() + 2000;
    char type = detectIntersectionType();
    String intersectionUID = createIntersectionID(type);

    Serial.print("Intersection detected: ");
    Serial.println(intersectionUID);

    stopAndMapNewNode(intersectionUID, false);

    nodeStack.push(currentNodeIndex);

    driveMotors(0,0);
    delay(200);

    // "hand on left" approach
    if (type == 'L') {
      turnLeft();
    } else if (type == 'R') {
      turnRight();
    } else if (type == 'T') {
      turnLeft(); // default left
    } else if (type == 'X'){
      turnBack();
    }else {
      goStraight();
    }
  }
}

bool detectIntersection()
{
  int threshold = qtr.calibrationOn.minimum[0] + 100;
  bool leftBranch = (sensorValues[0] > threshold &&
                     sensorValues[1] > threshold &&
                     sensorValues[2] > threshold);
  bool rightBranch = (sensorValues[3] > threshold &&
                      sensorValues[4] > threshold &&
                      sensorValues[5] > threshold);

  return (leftBranch || rightBranch);
}

char detectIntersectionType()
{
  int thresholdLeft  = qtr.calibrationOn.minimum[0] + 300;
  int thresholdRight = qtr.calibrationOn.minimum[5] + 300;
  prevSensorFirst = sensorValues[0] > thresholdLeft;
  prevSensorFirst = sensorValues[5] > thresholdRight;
  bool leftSide = (sensorValues[0] > thresholdLeft &&
                   sensorValues[1] > thresholdLeft &&
                   sensorValues[2] > thresholdLeft);
  bool rightSide = (sensorValues[3] > thresholdRight &&
                    sensorValues[4] > thresholdRight &&
                    sensorValues[5] > thresholdRight);

  bool nothing = (sensorValues[0] < thresholdLeft &&
                  sensorValues[1] < thresholdLeft &&
                  sensorValues[2] < thresholdLeft &&
                  sensorValues[3] < thresholdRight &&
                  sensorValues[4] < thresholdRight &&
                  sensorValues[5] < thresholdRight);

  // Determine intersection type
  if (leftSide && rightSide) {
    return 'T';  // T or + intersection
  } else if (leftSide) {
    return 'L';  // Left intersection
  } else if (rightSide) {
    return 'R';  // Right intersection
  } else if(nothing){
    return 'X';
  } else {
    return 'U';  // Unknown or straight
  }
}

String createIntersectionID(char type)
{
  String id = "INT_";
  id += String(intersectionCounter++);
  id += "_";
  id += type;
  return id;
}

void stopAndMapNewNode(const String& nodeUID, bool isRFID)
{
  driveMotors(0,0);
  delay(200);

  unsigned long now = millis();
  unsigned long travelTime = now - segmentStartTime;

  int newNodeIndex = findNodeIndex(nodeUID);
  if (newNodeIndex == -1) {
    newNodeIndex = addNode(nodeUID, isRFID);
    if (isRFID) {
      Serial.print("New RFID Node: ");
    } else {
      Serial.print("New Intersection Node: ");
    }
    Serial.println(nodeUID);
  } else {
    Serial.print("Revisiting node: ");
    Serial.println(nodeUID);
  }

  // record edge if not visited
  if (!edgeIsVisited(currentNodeIndex, newNodeIndex)) {
    addEdge(currentNodeIndex, newNodeIndex, travelTime);
    markEdgeVisited(currentNodeIndex, newNodeIndex);
  }

  currentNodeIndex = newNodeIndex;
  segmentStartTime = millis();

  delay(300);
}

int findNodeIndex(const String& uid)
{
  for (size_t i = 0; i < graph.size(); i++) {
    if (graph[i].uid == uid) {
      return i;
    }
  }
  return -1;
}

int addNode(const String& uid, bool isRFID)
{
  Node n;
  n.uid = uid;
  n.isRFID = isRFID;
  graph.push_back(n);
  return graph.size() - 1;
}

void addEdge(int from, int to, unsigned long cost)
{
  graph[from].neighbors.push_back(to);
  graph[from].costs.push_back(cost);

  graph[to].neighbors.push_back(from);
  graph[to].costs.push_back(cost);

  EdgeVisit ev;
  ev.nodeA = from;
  ev.nodeB = to;
  ev.visited = false;
  visitedEdges.push_back(ev);
}

bool edgeIsVisited(int a, int b)
{
  for (auto &e : visitedEdges) {
    if ((e.nodeA == a && e.nodeB == b) ||
        (e.nodeA == b && e.nodeB == a)) {
      return e.visited;
    }
  }
  return false;
}

void markEdgeVisited(int a, int b)
{
  for (auto &e : visitedEdges) {
    if ((e.nodeA == a && e.nodeB == b) ||
        (e.nodeA == b && e.nodeB == a)) {
      e.visited = true;
      return;
    }
  }
}

// hand-on-wall turns
void turnLeft(){
  Serial.println("Turning LEFT...");
  driveMotors(-220, 220);
  delay(400);
}
void turnRight(){
  Serial.println("Turning RIGHT...");
  driveMotors(220, -220);
  delay(400);
}
void goStraight(){
  Serial.println("Going STRAIGHT...");
  driveMotors(220, 220);
  delay(400);
}
void turnBack(){
  Serial.println("Turning BACK...");
  driveMotors(-220, 220);
  delay(800);
}

void driveMotors(int leftSpeed, int rightSpeed)
{
  if (leftSpeed >= 0) {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  } else {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  }
  ledcWrite(pwmChannel1, abs(leftSpeed));

  if (rightSpeed >= 0) {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  } else {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }
  ledcWrite(pwmChannel2, abs(rightSpeed));
}