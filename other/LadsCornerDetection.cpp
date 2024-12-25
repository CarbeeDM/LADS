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
float Ki = 0.0;
float Kd = 0.0;

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
const int maxSpeed = 220;

// ------------------- ULTRASONIC PINS -----------------
const int trigPin = 26;
const int echoPin = 21;

#define SOUND_SPEED 0.034

// ------------------- MAPPING STRUCTURES --------------
// Each Node can be an intersection or an RFID tag
struct Node {
  String uid;         // e.g. "RFID_XXYY" or "INT_1", "INT_2"
  bool isRFID;
  // adjacency: which other nodes connect to this node?
  std::vector<int> neighbors;
  std::vector<unsigned long> costs;  // travel time in ms
};

std::vector<Node> graph;            // Our global graph

int currentNodeIndex = -1;          // Index of the current node in the graph
int startNodeIndex = -1;            // Index of the start node in the graph
int intersectionCounter = 0;        // Generate unique intersection IDs
unsigned long segmentStartTime = 0; // measure travel time between nodes

// ------------------- FUNCTION DECLARATIONS -----------
int  findNodeIndex(const String& uid);
int  addNode(const String& uid, bool isRFID);
void addEdge(int from, int to, unsigned long cost);
void driveMotors(int leftSpeed, int rightSpeed);
void handleLineFollow();
void checkForRFID();
bool detectIntersection();
char detectIntersectionType();
String createIntersectionID(char type);
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
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
    if (i % 50 == 0) {
      Serial.print("Calibration progress: ");
      Serial.print((i / 200.0) * 100);
      Serial.println("%");
    }
  }
  Serial.println("Calibration complete!");

  // (Optional) Print calibration values
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
  Serial.println("Ultrasonic ready...");

  // ----- RFID READER SETUP -----
  SPI.begin();
  rfid.PCD_Init();
  Serial.println("Tap an RFID/NFC tag on the RFID-RC522 reader");

  delay(1000);

  // ----- START NODE HANDLING -----
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    // We'll treat this as the "start node"
    String startUID = "";
    for (int i = 0; i < rfid.uid.size; i++) {
      startUID += String(rfid.uid.uidByte[i], HEX);
    }
    startUID.toUpperCase();

    startNodeIndex = addNode(startUID, true);
    currentNodeIndex = startNodeIndex;
    Serial.print("Starting node (RFID): ");
    Serial.println(startUID);

  } else {
    // No RFID at start => create a "START" node
    startNodeIndex = addNode("START", false);
    currentNodeIndex = startNodeIndex;
    Serial.println("No RFID detected at start; using 'START' node.");
  }

  segmentStartTime = millis(); // start measuring travel time from the initial node
}

// --------------- MAIN LOOP ---------------------------
void loop()
{
  // 1. Check for new RFID and create a node if found
  checkForRFID();

  // 2. Check for intersection (T, L, +, etc.)
  handleIntersectionIfNeeded();

  // 3. Perform line following to stay on track
  handleLineFollow();

  // 4. (Optional) If we've returned to start with no new paths, we could end mapping here.
}

// -----------------------------------------------------
//                 HELPER FUNCTIONS
// -----------------------------------------------------

/**
 * @brief Find node index in global 'graph' by its UID
 */
int findNodeIndex(const String& uid) {
  for (size_t i = 0; i < graph.size(); i++) {
    if (graph[i].uid == uid) {
      return i;
    }
  }
  return -1; // not found
}

/**
 * @brief Add a new node to the graph
 */
int addNode(const String& uid, bool isRFID) {
  Node n;
  n.uid = uid;
  n.isRFID = isRFID;
  graph.push_back(n);
  return graph.size() - 1; // index of new node
}

/**
 * @brief Add an undirected edge with a cost (time in ms)
 */
void addEdge(int from, int to, unsigned long cost) {
  graph[from].neighbors.push_back(to);
  graph[from].costs.push_back(cost);

  graph[to].neighbors.push_back(from);
  graph[to].costs.push_back(cost);
}

bool obstacleDetection(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  float distanceCm = duration * SOUND_SPEED/2;

return distanceCm<5.0;
}

/**
 * @brief Basic motor driver
 */
void driveMotors(int leftSpeed, int rightSpeed)
{

  if(obstacleDetection()){
    ledcWrite(pwmChannel1, 0);
    ledcWrite(pwmChannel2, 0);
    return;
  }

  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  } else {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  }
  ledcWrite(pwmChannel1, abs(leftSpeed));

  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  } else {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }
  ledcWrite(pwmChannel2, abs(rightSpeed));
}

/**
 * @brief Reads QTR sensors, does PID line follow
 */
void handleLineFollow()
{
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 2500;  // for 6 sensors
  float proportional = error;
  integral += error;
  float derivative = error - lastError;

  float correction = (Kp * proportional) + (Ki * integral) + (Kd * derivative);
  correction = constrain(correction, -maxSpeed, maxSpeed);

  // We'll do a simple approach: a "base speed" approach, or direct approach.
  int minSpeed = 170;
  int leftMotorSpeed  = maxSpeed + correction;
  int rightMotorSpeed = maxSpeed - correction;

  leftMotorSpeed  = constrain(leftMotorSpeed, 0, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);

  // scale them so they never go below minSpeed
  float rangeFactor = (float)(maxSpeed - minSpeed) / (float)maxSpeed; 
  leftMotorSpeed  = minSpeed + rangeFactor * leftMotorSpeed;
  rightMotorSpeed = minSpeed + rangeFactor * rightMotorSpeed;

  driveMotors(leftMotorSpeed, rightMotorSpeed);
  lastError = error;
}

/**
 * @brief Check if a new RFID card is present; if new, create a node
 */
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

      int idx = findNodeIndex(newUID);
      if (idx == -1) {
        // new RFID
        stopAndMapNewNode(newUID, true);
        Serial.print("New RFID discovered: ");
        Serial.println(newUID);
      } else {
        // existing RFID
        Serial.print("Existing RFID scanned: ");
        Serial.println(newUID);
        // optional: do something if it's the start or end
        stopAndMapNewNode(newUID, true);
      }

      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }
  }
}

/**
 * @brief Quick check if all sensors read black -> potential intersection
 */
/**
 * @brief Detect an intersection based on sensor values.
 * 
 * Checks:
 * - If sensors [0, 1, 2] indicate a left branch (above threshold).
 * - If sensors [3, 4, 5] indicate a right branch (above threshold).
 * - If both indicate a T intersection.
 * - If center sensors indicate a straight path.
 */
bool detectIntersection() {
  qtr.readCalibrated(sensorValues);
  int threshold =400; // General threshold adjustment
  bool leftBranch = (sensorValues[0] > threshold && sensorValues[1] > threshold && sensorValues[2] > threshold && sensorValues[3] > threshold);
  bool rightBranch = (sensorValues[2] > threshold && sensorValues[3] > threshold && sensorValues[4] > threshold && sensorValues[5] > threshold);
  int flag=0;
  for(int i=0; i<SensorCount;i++){
    flag=(int)(sensorValues[i] > threshold);
    Serial.print(flag);
  }
  Serial.println();
  // Intersection detected if there’s at least one branch
  return leftBranch || rightBranch;
}

/**
 * @brief Attempt to guess intersection type (L, T, +, etc.)
 * 
 * Very naive approach: we check the leftmost sensor (0) and rightmost sensor (5).
 *  - If only leftmost is black -> L intersection to the left
 *  - If only rightmost is black -> L intersection to the right
 *  - If both are black -> T or + intersection
 * In a real scenario, you'd do additional checks or scanning.
 */
/**
 * @brief Attempt to guess intersection type (L, T, R, etc.)
 * 
 * - If sensors [0, 1, 2] are black (above threshold) -> L intersection
 * - If sensors [3, 4, 5] are black (above threshold) -> R intersection
 * - If both sides are black -> T intersection
 * 
 * Uses calibrated minimum values for thresholding.
 */

char detectIntersectionType() {
  int thresholdLeft = qtr.calibrationOn.minimum[0] + 300;  // Threshold for left sensors
  int thresholdRight = qtr.calibrationOn.minimum[5] + 300; // Threshold for right sensors

  // Check left side (sensors [0, 1, 2])
  bool leftSide = (sensorValues[0] > thresholdLeft &&
                   sensorValues[1] > thresholdLeft &&
                   sensorValues[2] > thresholdLeft);

  // Check right side (sensors [3, 4, 5])
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

/**
 * @brief Create an intersection ID string, e.g. "INT_0_T", "INT_1_L"
 */
String createIntersectionID(char type)
{
  String id = "INT_";
  id += String(intersectionCounter++);
  id += "_";
  // append the type code
  id += type; 
  return id;
}

/**
 * @brief If intersection is detected, stop, map it, decide a turn
 */
void handleIntersectionIfNeeded() {
  if (detectIntersection()) {
    // Determine intersection type
    char type = detectIntersectionType();

    // Create an intersection node
    String intersectionUID = createIntersectionID(type);
    Serial.print("Intersection detected: ");
    Serial.println(intersectionUID);

    // Map the intersection
    stopAndMapNewNode(intersectionUID, false);

    // Handle turn decisions
    driveMotors(0, 0);
    delay(200);

    if (type == 'L') {
      // Left intersection
      Serial.println("Turning LEFT (L intersection)...");
      driveMotors(-220, 220);  // Pivot left
      delay(400);
    } else if (type == 'R') {
      // Right intersection
      Serial.println("Turning RIGHT (R intersection)...");
      driveMotors(220, -220);  // Pivot right
      delay(400);
    } else if (type == 'T') {
      // T or + intersection
      Serial.println("T/+ intersection detected. Defaulting to LEFT...");
      driveMotors(-220, 220);  // Turn left by default
      delay(500);
    } else if (type == 'X') {
      // T or + intersection
      Serial.println("X deadend detected. reversing...");
      driveMotors(-220, 220);  // Turn left by default
      delay(1000);
    } else {
      // Unknown or straight
      Serial.println("Unknown intersection. Going straight...");
      driveMotors(220, 220);
      delay(400);
    }
  }
}

/**
 * @brief Called whenever we discover a new node (RFID or intersection).
 *        Stops the robot, records an edge from currentNode -> newNode, etc.
 */
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
    // Print whether intersection or RFID
    if (isRFID) {
      Serial.print("New RFID Node: ");
    } else {
      Serial.print("New Intersection Node: ");
    }
    Serial.println(nodeUID);
  } else {
    // already in graph
    Serial.print("Revisiting node: ");
    Serial.println(nodeUID);
  }

  // record the edge from current node to new node
  addEdge(currentNodeIndex, newNodeIndex, travelTime);

  // update current node
  currentNodeIndex = newNodeIndex;

  // reset travel timer
  segmentStartTime = millis();

  // small delay for stability
  delay(300);
}