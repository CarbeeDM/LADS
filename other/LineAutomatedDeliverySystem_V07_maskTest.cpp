#include <Arduino.h>
#include <QTRSensors.h>
#include <SPI.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <MFRC522.h>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <queue>

/* 
===============<[PURPOSE OF THIS CODE]>================
This code will drive the robot along the line while printing the status of the sensors.
It will produce 6 column list of inputs, for example:
0 0 1 1 0 0
0 0 1 1 0 0
1 1 1 1 1 1
1 1 1 1 1 1
0 0 1 1 0 0
0 0 1 1 1 0
1 0 1 1 0 0
0 0 1 1 0 0
(this is hand written dummy data)
it will tell us how wide the robot reads intersections as, and will let us decide on a memory size(how many previous sensor reads to store).
this will be used for hopefully improving the intersection identification.
=======================================================
data acquired:
001000
001000
001000
001000
001000
----
101000
101100
111111
111111
001111
001000
-----
001000
001000
001000
*/
using namespace std;
const vector<vector<int>> weights={ 
  {2,1  ,1,1  ,1,2}, 
  {2,1  ,1,1  ,1,2}, 

  {2,1  ,1,1  ,1,2}, 
  {2,1  ,1,1  ,1,2}, 

  {2,1  ,1,1  ,1,2}, 
  {3,3  ,3,3  ,3,3} };
class BoolMemory { 
  private: 
    deque<std::vector<bool>> memory;
    const size_t maxSize = 6;
   
  public: 
  BoolMemory() { 
    // Initialize the memory with rows of 'false' 
    std::vector<bool> initialRow={false,false,false,false,false,false}; 
    for (size_t i = 0; i < maxSize; ++i) { 
      memory.push_back(initialRow); 
    } 
  }
    void addSequence(const std::vector<bool> sequence) {
       if (sequence.size() != maxSize) {
          Serial.println("error sequence must be size 6");
          return;
           } 
        if (memory.size() == maxSize) {
          memory.pop_front();
           // Remove the oldest sequence 
           } 
           memory.push_back(sequence); // Add the new sequence 
           } 
    bool matchPattern(const std::vector<std::vector<bool>> mask, int errorMargin) {
       if (mask.size() != maxSize || mask[0].size() != maxSize) {
         Serial.println("error: mask must be 6x6");
         return false; 
         } 
      int totalDifferences = 0;
      for (size_t i = 0; i < maxSize; ++i) {
        for (size_t j = 0; j < maxSize; ++j) {
          if (memory[i][j] != mask[i][j]) { 
            totalDifferences++; 
          } 
          if (totalDifferences > errorMargin) { 
            return false; 
          } 
        } 
      } 
      return true; 
      } 
    int distance_to_Pattern(const std::vector<std::vector<bool>> mask) {
       if (mask.size() != maxSize || mask[0].size() != maxSize) {
         Serial.println("error: mask must be 6x6");
         return 99999; 
         } 
      int totalDifferences = 0;
      for (size_t i = 0; i < maxSize; ++i) {
        for (size_t j = 0; j < maxSize; ++j) {
          if (memory[i][j] != mask[i][j]) { 
            totalDifferences+=weights[i][j]; 
          } 
        } 
      } 
      return totalDifferences; 
      } 
    
    void printMemory() const {
      for (const auto sequence : memory) { 
        for (bool val : sequence) { 
          Serial.print(val);
           } 
        Serial.println();
      } 
    } 
  };

int mask_detected=0;
const vector<vector<bool>> mask_4way = { 
  {1,1  ,1,1  ,1,1}, 
  {1,1  ,1,1  ,1,1}, 

  {0,0  ,1,0  ,0,0}, 
  {0,0  ,1,0  ,0,0}, 

  {0,0  ,1,0  ,0,0}, 
  {0,0  ,1,0  ,0,0} };
  const vector<vector<bool>> mask_4way2 = { 
  {1,1  ,1,1  ,1,1}, 
  {1,1  ,1,1  ,1,1}, 

  {0,0  ,0,1  ,0,0}, 
  {0,0  ,0,1  ,0,0}, 

  {0,0  ,0,1  ,0,0}, 
  {0,0  ,0,1  ,0,0} };
const vector<vector<bool>> mask_L = { 
  {1,1  ,1,1  ,0,0}, 
  {1,1  ,1,1  ,0,0}, 

  {0,0  ,0,0  ,0,0}, 
  {0,0  ,0,0  ,0,0}, 

  {0,0  ,0,0  ,0,0}, 
  {0,0  ,0,0  ,0,0} };
const vector<vector<bool>> mask_R = { 
  {0,0  ,1,1  ,1,1}, 
  {0,0  ,1,1  ,1,1}, 

  {0,0  ,0,0  ,0,0}, 
  {0,0  ,0,0  ,0,0}, 

  {0,0  ,0,0  ,0,0}, 
  {0,0  ,0,0  ,0,0} };
const vector<vector<bool>> mask_Tf = { 
  {1,1  ,1,1  ,1,1}, 
  {1,1  ,1,1  ,1,1}, 

  {0,0  ,0,0  ,0,0}, 
  {0,0  ,0,0  ,0,0}, 

  {0,0  ,0,0  ,0,0}, 
  {0,0  ,0,0  ,0,0} };
const vector<vector<bool>> mask_Tr = {  
  {0,0  ,1,1  ,1,1}, 
  {0,0  ,1,1  ,1,1}, 

  {0,0  ,1,0  ,0,0}, 
  {0,0  ,1,0  ,0,0}, 

  {0,0  ,1,0  ,0,0}, 
  {0,0  ,1,0  ,0,0} };
  const vector<vector<bool>> mask_Tr2 = {  
  {0,0  ,0,1  ,1,1}, 
  {0,0  ,0,1  ,1,1}, 

  {0,0  ,0,1  ,0,0}, 
  {0,0  ,0,1  ,0,0}, 

  {0,0  ,0,1  ,0,0}, 
  {0,0  ,0,1  ,0,0} };

const vector<vector<bool>> mask_Tl = {  
  {1,1  ,1,0  ,0,0}, 
  {1,1  ,1,0  ,0,0}, 

  {0,0  ,1,0  ,0,0}, 
  {0,0  ,1,0  ,0,0}, 

  {0,0  ,1,0  ,0,0}, 
  {0,0  ,1,0  ,0,0} };
  const vector<vector<bool>> mask_Tl2 = {  
  {1,1  ,1,1  ,0,0}, 
  {1,1  ,1,1  ,0,0}, 

  {0,0  ,0,1  ,0,0}, 
  {0,0  ,0,1  ,0,0}, 

  {0,0  ,0,1  ,0,0}, 
  {0,0  ,0,1  ,0,0} };
const vector<vector<bool>> mask_N = { 
  {0,0  ,0,0  ,0,0}, 
  {0,0  ,0,0  ,0,0},

  {0,0  ,0,0  ,0,0}, 
  {0,0  ,0,0  ,0,0}, 

  {0,0  ,0,0  ,0,0}, 
  {0,0  ,0,0  ,0,0} };
const vector<vector<bool>> mask_I = { 
  {0,0  ,1,0  ,0,0}, 
  {0,0  ,1,0  ,0,0}, 

  {0,0  ,1,0  ,0,0}, 
  {0,0  ,1,0  ,0,0}, 

  {0,0  ,1,0  ,0,0}, 
  {0,0  ,1,0  ,0,0} };
const vector<vector<bool>> mask_I2 = { 
  {0,0  ,0,1  ,0,0}, 
  {0,0  ,0,1  ,0,0}, 

  {0,0  ,0,1  ,0,0}, 
  {0,0  ,0,1  ,0,0}, 

  {0,0  ,0,1  ,0,0}, 
  {0,0  ,0,1  ,0,0} };



// ------------------- RFID PINS ----------------------
#define SS_PIN  5   // ESP32 pin for MFRC522 SS/SDA
#define RST_PIN 25  // ESP32 pin for MFRC522 RST
MFRC522 rfid(SS_PIN, RST_PIN);  // Create MFRC522 instance

// ------------------- QTR SENSOR SETUP ----------------
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
BoolMemory s_memory;

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
//every left turn increments direction by 1, every right turn decrements it by 1. U turn does so by 2. then take mod4
int direction=0; //0(up) 1(left) 2(down) 3(right)

// Each Node can be an intersection or an RFID tag
struct Node {
  String uid;         // e.g. "RFID_XXYY" or "INT_1", "INT_2"
  bool isRFID;
  // adjacency: which other nodes connect to this node?
  std::vector<int> neighbors={-1,-1,-1,-1};
  vector<Node*> ptrs={nullptr,nullptr,nullptr,nullptr};
  std::vector<unsigned long> costs;  // travel time in ms
};

std::vector<Node> graph;            // Our global graph

Node wall;

vector<Node*> targetpath;

//0= wandering around until it comes across an RFID tag.
//1= mapping the maze
//2= going to a target node(for maze purposes)
//3= awaiting orders
//4= going to a target node(for delivery purposes)
int mode=0;  



int currentNodeIndex = -1;          // Index of the current node in the graph
int startNodeIndex = -1;            // Index of the start node in the graph
int intersectionCounter = 0;        // Generate unique intersection IDs
unsigned long segmentStartTime = 0; // measure travel time between nodes
unsigned long ignoreRFIDUntil = 0;      // ignore RFID reads until this time
unsigned long ignoreIntersectionUntil = 0;
// ------------------- FUNCTION DECLARATIONS -----------
int  findNodeIndex(const String& uid); //reused as is
int  addNode(const String& uid, bool isRFID);//reused as is
int  addNodePtr(const String uid, bool isRFID, Node* prev_n, int mask);//new

bool checkForRFID();//tweaked
bool doesRFIDexist(String new_uid); //new
bool obstacleDetection();//reused as is
bool handleIntersectionIfNeeded();//tweaked
bool knownNode(Node* n);

void driveMotors(int leftSpeed, int rightSpeed); //reused as is
void handleLineFollow(); //reused as is
void MapNode(const String nodeUID, bool isRFID, int mask);//new
void rstVars();
void turnL();//tweaked
void turnR();//tweaked
void handleDuplicate(Node* trueNode,Node* dupeNode);

String detectIntersection2(); //new
String createIntersectionID(char type); //reused as is

Node* newTarget(); //new

vector<Node*> pathToNode(String target_uid); //new

//----mode functions:
void m1(); //technically new
void wander(); //new
void mapMaze(); //new
void readyForOrder(); //new
void goToNode(vector<Node*> path); //new

// --------------- SETUP -------------------------------
void setup()
{
  Serial.begin(9600);
  Serial.println("Initializing...");
  //wall setup-------------
  wall.uid="wall";
  //-----------------------
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

  segmentStartTime = millis(); // start measuring travel time from the initial node
  
}

// --------------- MAIN LOOP ---------------------------
void loop()
{
  handleLineFollow();
detectIntersection2();
//handleIntersectionIfNeeded();
}
// -----------------------------------------------------
//                       MODES
// -----------------------------------------------------
void m1(){
  if (millis() > ignoreRFIDUntil) {
        checkForRFID();
    }

  // 2. Check for intersection (T, L, +, etc.)
  if(millis() > ignoreIntersectionUntil ){
        handleIntersectionIfNeeded();
    }

  // 3. Perform line following to stay on track
  handleLineFollow();
}

void readyForOrder(){
  //open connnection and wait until you get given a task.
  if(false){
  String targetInput;
  targetpath=pathToNode(targetInput);
  mode=4;
  rstVars();
  return;
  } else {
    Serial.println("I have mapped everything I saw");
    delay(10000);
  }
}

void mapMaze(){
  handleLineFollow();
  Node* target;
  if(handleIntersectionIfNeeded() || checkForRFID()){

  if(knownNode(&graph[currentNodeIndex])){
    target=newTarget();
    rstVars();
    if(target==&wall){
      mode=3;
      return;
    }
    targetpath=pathToNode(target->uid);
    mode=2;
    return;
  } else {
    for(int i=0; i< graph[currentNodeIndex].ptrs.size();i++){
      if(graph[currentNodeIndex].ptrs[i]==nullptr){
        switch (direction-i)
        {
        case -1:
        case 3:
          turnL();
          break;
        case 2:
          turnL();
          turnL();
          break;
        case 1: 
        case -3:
          turnR();
          break;
        default:
          break;
        }
        i+=graph[currentNodeIndex].ptrs.size();
      }
    }

  }
  }
}

void goToNode(vector<Node*> path){

    handleLineFollow();

    if(handleIntersectionIfNeeded() || checkForRFID()){
      if(&graph[currentNodeIndex]==*path.begin()){
        path.erase(path.begin());
      }
      if(path.empty()){
        Serial.println("I have arrived");
        rstVars();
        if(mode==2){
          mode=1;
        } else {
          mode=3;
        }
        return;
      }
      for(int i=0; i< graph[currentNodeIndex].ptrs.size();i++){
      if(graph[currentNodeIndex].ptrs[i]==*path.begin()){
        switch (direction-i)
        {
        case -1:
        case 3:
          turnL();
          break;
        case 2:
          turnL();
          turnL();
          break;
        case 1: 
        case -3:
          turnR();
          break;
        default:
          break;
        }
        i+=graph[currentNodeIndex].ptrs.size();
      }
    }
    }
}

void wander(){
  handleLineFollow();
  if(handleIntersectionIfNeeded() || checkForRFID()){
    if(currentNodeIndex>-1){
      mode=1;
      rstVars();
      return;
    }
    for(int i=0; i< graph[currentNodeIndex].ptrs.size();i++){
      if(graph[currentNodeIndex].ptrs[i]!=&wall){
        switch (direction-i)
        {
        case -1:
        case 3:
          turnL();
          break;
        case 2:
          turnL();
          turnL();
          break;
        case 1: 
        case -3:
          turnR();
          break;
        default:
          break;
        }
        i+=graph[currentNodeIndex].ptrs.size();
      }
    }
  }
}



// -----------------------------------------------------
//                 HELPER FUNCTIONS
// -----------------------------------------------------
void rstVars(){
  ignoreIntersectionUntil=0;
  ignoreRFIDUntil=0;
}
bool knownNode(Node* n){
  for(int i=0;i<n->ptrs.size();i++){
    if(n->ptrs[i]==nullptr){
      return false;
    }
  }
  return true;
}

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

// /**
//  * @brief Add a new node to the graph
//  */
int addNode(const String& uid, bool isRFID) {
  Node n;
  n.uid = uid;
  n.isRFID = isRFID;
  graph.push_back(n);
  return graph.size() - 1; // index of new node
}
int addNodePtr(const String uid, bool isRFID, Node* prev_n, int mask){
  Node n;
  n.uid = uid;
  n.isRFID = isRFID;
  int from= (direction+2)%4;
  if(prev_n!=nullptr){
  n.neighbors[from]=findNodeIndex(prev_n->uid);
  n.ptrs[from]=prev_n;
  }
  if(!(mask&(1<<0))){
    n.ptrs[0]=&wall;
  }
  if(!(mask&(1<<1))){
    n.ptrs[1]=&wall;
  }
  if(!(mask&(1<<2))){
    n.ptrs[2]=&wall;
  }
  if(!(mask&(1<<3))){
    n.ptrs[3]=&wall;
  }
  graph.push_back(n);
  return graph.size() - 1;
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
bool checkForRFID()
{
  if (rfid.PICC_IsNewCardPresent() || ignoreRFIDUntil < millis()) {
    ignoreRFIDUntil = millis() + 2000;
    if (rfid.PICC_ReadCardSerial()) {
      // Build the UID string
      String newUID = "";
      for (int i = 0; i < rfid.uid.size; i++) {
        newUID += String(rfid.uid.uidByte[i], HEX);
      }
      newUID.toUpperCase();

      int mask=0;
      int mflag;
      mflag= (direction)%4;
      mask = mask | 1 << mflag;
      mflag= (direction+2)%4;
      mask = mask | 1 << mflag;
      MapNode(newUID,true,mask);

      

      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
      return true;
    }
  }
  return false;
}


String detectIntersection2() {
  qtr.readCalibrated(sensorValues);
  int threshold =400; // General threshold adjustment
int flag=0;
  for(int i=0; i<SensorCount;i++){
    flag=(int)(sensorValues[i] > threshold);
    Serial.print(flag);
  }

  s_memory.addSequence({sensorValues[0] > threshold,
                        sensorValues[1] > threshold,
                        sensorValues[2] > threshold,
                        sensorValues[3] > threshold,
                        sensorValues[4] > threshold,
                        sensorValues[5] > threshold});
  
  int dist_L=s_memory.distance_to_Pattern(mask_L);
  int dist_R=s_memory.distance_to_Pattern(mask_R);
  int dist_Tf=s_memory.distance_to_Pattern(mask_Tf);
  int dist_Tr=min(s_memory.distance_to_Pattern(mask_Tr),s_memory.distance_to_Pattern(mask_Tr2));
  int dist_Tl=min(s_memory.distance_to_Pattern(mask_Tl),s_memory.distance_to_Pattern(mask_Tl2));
  int dist_I=min(s_memory.distance_to_Pattern(mask_I),s_memory.distance_to_Pattern(mask_I2));
  int dist_N=s_memory.distance_to_Pattern(mask_N);
  int dist_X=min(s_memory.distance_to_Pattern(mask_4way),s_memory.distance_to_Pattern(mask_4way2));
  Serial.print(" --- ");
  Serial.print("I:");Serial.print(dist_I);
  Serial.print(" L:");Serial.print(dist_L);
  Serial.print(" R:");Serial.print(dist_R);
  Serial.print(" Tf:");Serial.print(dist_Tf);
  Serial.print(" Tr:");Serial.print(dist_Tr);
  Serial.print(" Tl:");Serial.print(dist_Tl);
  Serial.print(" X:");Serial.print(dist_X);
  Serial.print(" N:");Serial.print(dist_N);
  Serial.print(" ---->");
  if(dist_L==min({dist_L,dist_R,dist_Tf,dist_Tr,dist_Tl,dist_I,dist_N,dist_X})){
    Serial.println("L");return "L";
  } else if(dist_R==min({dist_L,dist_R,dist_Tf,dist_Tr,dist_Tl,dist_I,dist_N,dist_X})){
    Serial.println("R");return "R";
  } else if(dist_Tf==min({dist_L,dist_R,dist_Tf,dist_Tr,dist_Tl,dist_I,dist_N,dist_X})){
    Serial.println("TF");return "Tf";
  } else if(dist_Tr==min({dist_L,dist_R,dist_Tf,dist_Tr,dist_Tl,dist_I,dist_N,dist_X})){
    Serial.println("TR");return "Tr";
  } else if(dist_Tl==min({dist_L,dist_R,dist_Tf,dist_Tr,dist_Tl,dist_I,dist_N,dist_X})){
    Serial.println("TL");return "Tl";
  } else if(dist_I==min({dist_L,dist_R,dist_Tf,dist_Tr,dist_Tl,dist_I,dist_N,dist_X})){
    Serial.println("I");return "I";
  } else if(dist_N==min({dist_L,dist_R,dist_Tf,dist_Tr,dist_Tl,dist_I,dist_N,dist_X})){
    Serial.println("N");return "N";
  } else if(dist_X==min({dist_L,dist_R,dist_Tf,dist_Tr,dist_Tl,dist_I,dist_N,dist_X})){
    Serial.println("X");return "X";
  }

  // Intersection detected if there’s at least one branch
  return "none";
}

/**
 * @brief Create an intersection ID string, e.g. "INT_0_T", "INT_1_L"
 */
String createIntersectionID(String type)
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
bool handleIntersectionIfNeeded() {
    
    // Determine intersection type
    if(ignoreIntersectionUntil>millis()){Serial.println("waiting...");return false;}
    String type = detectIntersection2();
    if(type=="I"){return false;}
    // Create an intersection node
    String intersectionUID = createIntersectionID(type);
    Serial.print("Intersection detected: ");
    Serial.println(intersectionUID);
    ignoreIntersectionUntil=millis()+4000;



    int mask=0;
    int mflag= (direction+2)%4;
    mask = mask | 1 << mflag;
    // Handle turn decisions
    driveMotors(0, 0);


    if (type == "L") {
      // Left intersection
      Serial.println("Turning LEFT (L intersection)...");
      mflag= (direction+1)%4;
      mask = mask | 1 << mflag;
      //turnL();
    } else if (type == "R") {
      // Right intersection
      Serial.println("Turning RIGHT (R intersection)...");
      mflag= (direction-1)%4;
      mask = mask | 1 << mflag;
      //turnR();
    } else if (type == "Tf") {
      // T or + intersection
      Serial.println("Tf intersection detected. Defaulting to LEFT...");
      mflag= (direction+1)%4;
      mask = mask | 1 << mflag;
      mflag= (direction-1)%4;
      mask = mask | 1 << mflag;
      //turnL();
    } else if (type == "Tl") {
      // T or + intersection
      Serial.println("Tl intersection detected. Defaulting to LEFT...");
      mflag= (direction+1)%4;
      mask = mask | 1 << mflag;
      mflag= (direction)%4;
      mask = mask | 1 << mflag;
      //turnL();
    } else if (type == "Tr") {
      // T or + intersection
      Serial.println("Tr intersection detected. Going forward...");
      mflag= (direction-1)%4;
      mask = mask | 1 << mflag;
      mflag= (direction)%4;
      mask = mask | 1 << mflag;

    } else if (type == "X") {
      // T or + intersection
      Serial.println("+ intersection detected. Defaulting to LEFT...");
      mflag= (direction)%4;
      mask = mask | 1 << mflag;
      mflag= (direction+1)%4;
      mask = mask | 1 << mflag;
      mflag= (direction+2)%4;
      mask = mask | 1 << mflag;
      mflag= (direction+3)%4;
      mask = mask | 1 << mflag;
      //turnL();
    }  else if (type == "N") {
      // T or + intersection
      Serial.println("N: deadend detected. reversing...");
      //turnL();
      //turnL();
    } else {
      // Unknown or straight
      Serial.println("Unknown intersection. Going straight...");
    }
  //MapNode(intersectionUID,false,mask);
  return true;
}

void turnL(){direction++;direction=direction%4;
      driveMotors(-220, 220);  // Pivot left
      delay(500);
      }
void turnR(){direction--;direction=direction%4;
      driveMotors(220, -220);  // Pivot left
      delay(500);
      }
/**
 * @brief Called whenever we discover a new node (RFID or intersection).
 *        Stops the robot, records an edge from currentNode -> newNode, etc.
 */
void MapNode(const String nodeUID, bool isRFID, int mask){
  // stop motors
  driveMotors(0,0);
  delay(200);
  Serial.print("mapping new node, mask is: ");
  Serial.println(mask);
  if(currentNodeIndex=-1){
    currentNodeIndex=addNodePtr(nodeUID,isRFID,nullptr,mask);
    return;
  }
  unsigned long now = millis();
  unsigned long travelTime = now - segmentStartTime;
  int newNodeIndex = findNodeIndex(nodeUID);
  int from=(direction+2)%4;
  if(newNodeIndex==-1){
    newNodeIndex=addNodePtr(nodeUID,isRFID,&graph[currentNodeIndex],mask);
  } else if(graph[newNodeIndex].ptrs[from] != nullptr){
      handleDuplicate(graph[newNodeIndex].ptrs[from], &graph[currentNodeIndex]);
      return;
  }
  graph[currentNodeIndex].neighbors[direction]=newNodeIndex;
  graph[currentNodeIndex].costs[direction]=travelTime;
  graph[newNodeIndex].costs[from]=travelTime;
  graph[currentNodeIndex].ptrs[direction]=&(graph[newNodeIndex]);
  currentNodeIndex = newNodeIndex;
}

void handleDuplicate(Node* trueNode,Node* dupeNode){
  for(int i=0; i< trueNode->ptrs.size();i++){
    if(dupeNode->ptrs[i] !=nullptr){
      if(trueNode->ptrs[i]==nullptr){
        trueNode->ptrs[i]=dupeNode->ptrs[i];
        trueNode->neighbors[i]=dupeNode->neighbors[i];
        trueNode->costs[i]=dupeNode->costs[i];
        int j=(i+2)%4;
        dupeNode->ptrs[i]->ptrs[j]=trueNode;
        dupeNode->ptrs[i]->neighbors[j]=findNodeIndex(trueNode->uid);
      } 
      if(trueNode->ptrs[i]!=nullptr && (trueNode->ptrs[i]->uid != dupeNode->ptrs[i]->uid)){
        handleDuplicate(trueNode->ptrs[i],dupeNode->ptrs[i]);
      }
    }
  }
  //add here function to delete the dupe node to free up space
  //will fuck up the neighbour arrays due to reference by index
  //but that can be removed entirely
}

bool doesRFIDexist(String new_uid){
  for(Node i : graph){
    if(i.uid==new_uid){
      return true;
    }
  }
  return false;
}

Node* newTarget(){
  queue<Node*> toVisit;
  toVisit.push(&graph[currentNodeIndex]);
  unordered_set<Node*> visited;
  visited.insert(&graph[currentNodeIndex]);
  while(!toVisit.empty()){
    Node* curr_node=toVisit.front();
    toVisit.pop();
    for(int i=0; i<curr_node->ptrs.size();i++){
      if(curr_node->ptrs[i]==nullptr){
        return curr_node;
      } else if (curr_node->ptrs[i]!=&wall){
        toVisit.push(curr_node->ptrs[i]);
        visited.insert(curr_node->ptrs[i]);
      }
    }
  }
  return &wall;
}


vector<Node*> pathToNode(String target_uid){
  queue<Node*> toVisit; 
  unordered_map<Node*, Node*> parentMap; 
  // To store the parent of each node 
  unordered_set<Node*> visited;
  toVisit.push(&graph[currentNodeIndex]);
  visited.insert(&graph[currentNodeIndex]);
  parentMap[&graph[currentNodeIndex]] = nullptr;
  if(findNodeIndex(target_uid)==-1){vector<Node*> nopath={}; return nopath;}
  ///while
  while(!toVisit.empty()){
    Node* curr_node=toVisit.front();
    toVisit.pop();

    if(curr_node->uid ==target_uid){
      vector<Node*> path;
      for (Node* node = curr_node;node!=nullptr;node=parentMap[node]){
        path.push_back(node);
      }
      reverse(path.begin(), path.end());
      return path;
    }

    for (Node* ptr_neighbor : curr_node->ptrs){
        if (ptr_neighbor!=nullptr && ptr_neighbor!=&wall){
          if (visited.find(ptr_neighbor) == visited.end()){
            toVisit.push(ptr_neighbor);
            visited.insert(ptr_neighbor);
            parentMap[ptr_neighbor] = curr_node;
          }
        }
    }
  }
}