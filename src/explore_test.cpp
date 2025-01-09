#include <algorithm>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <string>
#include <iostream>
#include <memory>


using namespace std;


// ------------------- MAPPING STRUCTURES --------------
//every left turn increments direction by 1, every right turn decrements it by 1. U turn does so by 2. then take mod4
int direction=0; //0(up) 1(left) 2(down) 3(right)
int prevDirection=0;

// Each Node can be an intersection or an RFID tag
struct Node {
  string uid;         // e.g. "RFID_XXYY" or "INT_1", "INT_2"
  bool isRFID;
  // adjacency: which other nodes connect to this node?
  vector<shared_ptr<Node>> ptrs={nullptr,nullptr,nullptr,nullptr};
  std::vector<int> costs={0,0,0,0};  // travel time in ms
  
};

//std::vector<Node> graph;            // Our global graph
std::vector<shared_ptr<Node>> graph;
vector<tuple<string,shared_ptr<Node>>> cmd_stack;
shared_ptr<Node> current_Node_Ptr;

std::shared_ptr<Node> wall = std::make_shared<Node>();

vector<shared_ptr<Node>> targetpath;

//0= wandering around until it comes across an RFID tag.
//1= mapping the maze
//2= going to a target node(for maze purposes)
//3= awaiting orders
//4= going to a target node(for delivery purposes)
int mode=0;  

bool backtrack=false;

int currentNodeIndex = -1;              // Index of the current node in the graph
int startNodeIndex = -1;                // Index of the start node in the graph
int intersectionCounter = 0;            // Generate unique intersection IDs
unsigned long segmentStartTime = 0;     // measure travel time between nodes
unsigned long ignoreRFIDUntil = 0;      // ignore RFID reads until this time
unsigned long ignoreIntersectionUntil = 0;
string last_read_tagUID = "";
// ------------------- FUNCTION DECLARATIONS -----------
int  findNodeIndex(const string& uid); //reused as is

int  addNodePtr(const string uid, bool isRFID, int mask);//new


bool checkForRFID();//tweaked
bool doesRFIDexist(string new_uid); //new
bool obstacleDetection();//reused as is
bool handleIntersectionIfNeeded();//tweaked
bool knownNode(shared_ptr<Node> n);

void driveMotors(int leftSpeed, int rightSpeed); //reused as is
void handleLineFollow(); //reused as is

void rstVars();
void turnL();//tweaked
void turnR();//tweaked
void merge_two_nodes(shared_ptr<Node> trueNode,shared_ptr<Node> oldNode);
void link_via_direction(shared_ptr<Node> n, unsigned long cost);
void process_cmd(string cmd, int cost);

string detectIntersection(); //new
string createIntersectionID(string type, bool rfid); //reused as is

shared_ptr<Node> newTarget(); //new

vector<shared_ptr<Node>> pathToNode(string target_uid); //new

tuple<string, bool, bool> interpret_triple(tuple<string, string, string> triple);

//----mode functions:
void m1(); //technically new
void ExplorationPhase();
void wander(); //new
void mapMaze(); //new
void readyForOrder(); //new
void goToNode(vector<shared_ptr<Node>> path); //new
void printFinalGraphState();

// --------------- SETUP -------------------------------

// --------------- MAIN LOOP ---------------------------
int main()
{
  mode=0;
  cout<<"starting..."<<endl;
  while(mode==0){
    string cmd;
    int cost;
    cout << "backtrace mode: "<< backtrack<<endl;
    cout << "Enter cmd(q to exit): ";
    cin >> cmd; // Waits for user input
    if (cmd=="q"){
      printFinalGraphState(); 
      mode=-99; 
      break;}

    cout << "enter cost: ";
    cin >> cost;
    process_cmd(cmd,cost);
    cout<< "----------CURRENT MAP STATE---------"<<endl;
    for(shared_ptr<Node> n : graph){
      cout<< n->uid<< ":  {";
      for (int i=0;i <n->ptrs.size();i++){
        cout <<"    ";
        if (n->ptrs[i]!=nullptr && n->ptrs[i]!=wall){
          cout << i <<": "<< n->ptrs[i]->uid;
          cout << "/" <<n->costs[i]<<"||";
        } else{
          cout << i << ": NULL ||";
        }
      }
      cout<<"}"<<endl;
    }
    cout<<"--------------------------------------"<<endl;
}
return 0;
}
// -----------------------------------------------------
//                       MODES
// -----------------------------------------------------
void ExplorationPhase(){
  handleLineFollow();


        handleIntersectionIfNeeded();
    

        if(checkForRFID()){
          //process_cmd("P");
        };
    
}
void m1(){

        checkForRFID();
    

  // 2. Check for intersection (T, L, +, etc.)

        handleIntersectionIfNeeded();
    

  // 3. Perform line following to stay on track
  handleLineFollow();
}

void readyForOrder(){
  //open connnection and wait until you get given a task.
  if(false){
  string targetInput;
  targetpath=pathToNode(targetInput);
  mode=4;
  rstVars();
  return;
  } else {
    
  }
}

void mapMaze(){
  handleLineFollow();
  shared_ptr<Node> target;
  if((detectIntersection()!="I") || checkForRFID()){
  current_Node_Ptr=current_Node_Ptr->ptrs[direction];
  if(knownNode(current_Node_Ptr)){
    target=newTarget();
    rstVars();
    if(target==wall){
      mode=3;
      return;
    }
    targetpath=pathToNode(target->uid);
    mode=2;
    return;
  } else {
    for(int i=0; i< graph[currentNodeIndex]->ptrs.size();i++){
      if(graph[currentNodeIndex]->ptrs[i]==nullptr){
        switch (direction-i)
        {
        case -1:
        case 3:
          turnL();
          break;
        case -2:
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
        i+=graph[currentNodeIndex]->ptrs.size();//break from for loop
      }
    }

  }
  }
}

void goToNode(vector<shared_ptr<Node>> path){

    handleLineFollow();

    if((detectIntersection()!="I") || checkForRFID()){
      current_Node_Ptr=current_Node_Ptr->ptrs[direction];
      if(current_Node_Ptr==*path.begin()){
        path.erase(path.begin());
      }
      if(path.empty()){
        
        rstVars();
        if(mode==2){
          mode=1;
        } else {
          mode=3;
        }
        return;
      }
      for(int i=0; i< graph[currentNodeIndex]->ptrs.size();i++){
      if(graph[currentNodeIndex]->ptrs[i]==*path.begin()){
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
        i+=graph[currentNodeIndex]->ptrs.size();//break from for loop
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
    for(int i=0; i< graph[currentNodeIndex]->ptrs.size();i++){
      if(graph[currentNodeIndex]->ptrs[i]!=wall){
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
        i+=graph[currentNodeIndex]->ptrs.size();
      }
    }
  }
}



// -----------------------------------------------------
//                 HELPER FUNCTIONS
// -----------------------------------------------------

/**
 * @brief set ignore vars to 0
 */
void rstVars(){
  ignoreIntersectionUntil=0;
  ignoreRFIDUntil=0;
}

/**
 * @brief check if a given node has all its corners mapped out completely
 */
bool knownNode(shared_ptr<Node> n){
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
int findNodeIndex(const string& uid) {
  for (size_t i = 0; i < graph.size(); i++) {
    if (graph[i]->uid == uid) {
      return i;
    }
  }
  return -1; // not found
}

// /**
//  * @brief Add a new node to the graph
//  */
int addNodePtr(const string uid, bool isRFID, int mask){

  auto n = std::make_shared<Node>();
 
  n->uid=uid;

  n->isRFID = isRFID;

  if(!(mask&(1<<0))){
    n->ptrs[0]=wall;
  }

  if(!(mask&(1<<1))){
    n->ptrs[1]=wall;
  }

  if(!(mask&(1<<2))){
    n->ptrs[2]=wall;
  }

  if(!(mask&(1<<3))){
    n->ptrs[3]=wall;
  }

  graph.push_back(n);

  return graph.size() - 1;
}
/**
 * @brief MUST be called before updating current_node_ptr.
 * take in the new node, create a connection between it and the current_node_ptr
 */
void link_via_direction(shared_ptr<Node> new_node, int cost){
cout<<"[link]";
  if(current_Node_Ptr==nullptr){
    cout<<"[ERROR:link_via_direction]: nullpointer"<< endl;
    return;
  }
cout<<current_Node_Ptr->uid<<"/"<<prevDirection<<"/"<<new_node->uid;
  current_Node_Ptr->ptrs[prevDirection]=new_node;
cout<<"b";
  current_Node_Ptr->costs[prevDirection]=cost;
cout<<"c";
  int opp= (prevDirection+2)%4;

  new_node->ptrs[opp]=current_Node_Ptr;
cout<<"d";
  new_node->costs[opp]=cost;
cout<<"[/link]";

}

bool obstacleDetection(){


return false;
}

/**
 * @brief Basic motor driver
 */
void driveMotors(int leftSpeed, int rightSpeed)
{


}

/**
 * @brief Reads QTR sensors, does PID line follow
 */
void handleLineFollow()
{
  
}

/**
 * @brief Check if a new RFID card is present, update ignoreRFIDuntil
 */
bool checkForRFID()
{
  return false;
}

/**
 * @brief returns a T, L, R, or U based on intersection.
 * Returns I if no intersection.
 * Updates ignoreIntersectionUntil
 */
string detectIntersection() {

  int threshold =400; // General threshold adjustment

  bool leftSide = false;
  bool rightSide = false;

  bool nothing = false;

  // Determine intersection type
  if (leftSide && rightSide) {
    return "T";  // T or + intersection
  } else if (leftSide) {
    return "L";  // Left intersection
  } else if (rightSide) {
    return "R";  // Right intersection
  } else if(nothing){
    return "U";
  } else {
    return "I";  // Unknown or straight
  }

}

/**
 * @brief Create an ID string, e.g. "INT_0_T", "INT_1_L"
 */
string createIntersectionID(string type, bool rfid)
{ 
  if(rfid){
  string id = "rfid_";
  id += to_string(intersectionCounter++);
  id += "_";
  // append the type code
  id += type;
  return id;
  }else{
  string id = "INT_";
  id += to_string(intersectionCounter++);
  id += "_";
  // append the type code
  id += type; 
  return id;
  }
}

/**
 * @brief If intersection is detected process_cmd().
 *  Only for exploration phase
 */
bool handleIntersectionIfNeeded() {
    
    // Determine intersection type
    string type = detectIntersection();
    if(type=="I"){return false;}
    //process_cmd(type);

    return true;
}

void turnL(){direction++;direction=(direction+4)%4;
      
      }
void turnR(){direction--;direction=(direction+4)%4;
      
      }

/**
 * @brief Add a new node or do backtracking. S for startNode.
 */
void process_cmd(string cmd, int cost){
    if(cmd=="S"){
      if(current_Node_Ptr==nullptr){
        cout<< "We are at the start." << endl;
        int mask=0;
        int mflag= (direction+2)%4;
        mask = mask | 1 << mflag;
        mask = mask | 1 << direction;
        int newNodeIndex=addNodePtr("START_NODE", false, mask);

        current_Node_Ptr=graph[newNodeIndex];

        //tuple <string, shared_ptr<Node>> tup=make_tuple(cmd,current_Node_Ptr);
        cmd_stack.push_back(make_tuple(cmd,current_Node_Ptr));
        return;
      } else {
        cout<< "Returned to the start; We are done with exploration mode." << endl;
        
        link_via_direction(graph[findNodeIndex("START_NODE")],cost);
        current_Node_Ptr=graph[findNodeIndex("START_NODE")];
        mode=1;
        //do stuff
        return;
      }
    }

    bool isRFID=false;
    if(cmd=="P"){
      isRFID=true;
    }
    string UID = createIntersectionID(cmd, isRFID);
    prevDirection=direction;
    int newNodeIndex=-1;
    int mask=0;
    int mflag= (direction+2)%4;
    mask = mask | 1 << mflag;
    // Handle turn decisions
    driveMotors(0, 0);

  if(backtrack==false){
    if (cmd == "L") {
      // Left intersection
      cout<< "Turning LEFT (L intersection)..." << endl;
      mflag= (direction+1)%4;
      mask = mask | 1 << mflag;
      turnL();
    } else if (cmd == "R") {
      // Right intersection
      cout<< "Turning RIGHT (R intersection)..." << endl;
      mflag= (direction+3)%4;
      mask = mask | 1 << mflag;
      turnR();
    } else if (cmd == "T") {
      // T or + intersection
      cout<< "T intersection detected" << endl;
      mflag= (direction+1)%4;
      mask = mask | 1 << mflag;
      mflag= (direction+3)%4;
      mask = mask | 1 << mflag;
      cout<< "defaulting LEFT...";
      turnL(); 
    }  else if (cmd == "U") {
      // T or + intersection
      cout<< "U: deadend detected. reversing..." << endl;

      backtrack=true;

      turnL();
      turnL();
    } else if (cmd == "P") {
      cout<< "RFID tag detected, continuing forward" << endl;
    } else {
      // Unknown or straight
      cout<< "Unknown intersection. Going straight..." << endl;
    }
cout<<"1";
      newNodeIndex=addNodePtr(UID, isRFID, mask);
cout<<"2";
      link_via_direction(graph[newNodeIndex], cost);
cout<<"3";
      current_Node_Ptr=graph[newNodeIndex];
cout<<"4";
      //make tie
      //tuple <string, shared_ptr<Node>> tup=make_tuple(cmd,current_Node_Ptr);
      cmd_stack.push_back(make_tuple(cmd,current_Node_Ptr));
cout<<"5";

  } else {//backtrack
    if(cmd_stack.size()<2){
      newNodeIndex=addNodePtr(UID, isRFID, mask);
      current_Node_Ptr=graph[newNodeIndex];
      //make tie
      //tuple <string, shared_ptr<Node>> tup=make_tuple(cmd,current_Node_Ptr);
        cmd_stack.push_back(make_tuple(cmd,current_Node_Ptr));
      cout<< "backtrace, not enough for a triple, pushed cmd" << endl;
      return;
    } else{
    tuple <string, shared_ptr<Node>> u_tuple=cmd_stack[cmd_stack.size()-1];
    if(get<0>(u_tuple)!="U"){
      newNodeIndex=addNodePtr(UID, isRFID, mask);
      current_Node_Ptr=graph[newNodeIndex];
      //make tie
      //tuple <string, shared_ptr<Node>> tup=make_tuple(cmd,current_Node_Ptr);
        cmd_stack.push_back(make_tuple(cmd,current_Node_Ptr));
      cout<< "backtrace, unexpected top, pushed cmd" << endl;
      return;
    }

    tuple <string, shared_ptr<Node>> y_tuple=cmd_stack[cmd_stack.size()-2];
    //tuple <string,bool,bool> triple_decision; //decision, error, remain_backtrace
    string decision;
    bool error;
    bool remain_backtrace;
    tie(decision,error,remain_backtrace)=interpret_triple(tie(get<0>(y_tuple),"U",cmd));
    if(error){
      cout<< "ERROR: map is corrupted for triple" << endl;
      return;
    }
    if (decision!=""){
      cout<< "backtrace decision:";
      cout<< decision<< endl;
      newNodeIndex=addNodePtr(UID, isRFID, mask);
      current_Node_Ptr=graph[newNodeIndex];
      merge_two_nodes(current_Node_Ptr, get<1>(y_tuple));
      //merge in backtrace(y_node,cmd)
      if(decision=="L"){
        turnL();
      } else if (decision=="R"){
        turnR();
      } 
      cout<< "new direction: ";
      cout<< direction << endl;;
    }

    cmd_stack.pop_back();
    cmd_stack.pop_back();
    if (remain_backtrace){
      cout<<"remaining in backtrace"<<endl;
      cmd_stack.push_back(tie("U", get<1>(u_tuple)));
    }else{
      cout<<"exiting backtrace"<<endl;
      backtrack=false;
    }

    }
  }
}

/**
 * @brief During backtracing, handle triplets.
 */
tuple<string, bool, bool> interpret_triple(tuple<string, string, string> triple) {
  string Y, U, X;
  tie(Y, U, X) = triple; 
  if (tie(Y, U, X) == make_tuple("R", "U", "L")) { 
    return make_tuple("L", false, true); 
  } else if (tie(Y, U, X) == make_tuple("L", "U", "T")) {
    return make_tuple("L", false, false);
  }
  else if (tie(Y, U, X) == make_tuple("R", "U", "T")) { 
    return make_tuple("R", false, false);
  } else if (tie(Y, U, X) == make_tuple("L", "U", "R")) {
    return make_tuple("R", false, true);
  } 
  else if (tie(Y, U, X) == make_tuple("T", "U", "L")) {
    return make_tuple("forward", false, false);
  } else if (tie(Y, U, X) == make_tuple("P", "U", "P")) {
    return make_tuple("forward", false, true);
  }
  else if (tie(Y, U, X) == make_tuple("L", "U", "L")) {
    return make_tuple("", true, true);//returning empty string instead of None
  } else if (tie(Y, U, X) == make_tuple("T", "U", "T")) {
    return make_tuple("", true, true);//we dont accept 4 ways
  } else if (tie(Y, U, X) == make_tuple("T", "U", "R")) {
    return make_tuple("", true, true);//we turn left by policy, so we wouldnt see this
  } else {
    return make_tuple("", true, true);
  }
}

/**
 * @brief copy oldNode's info onto trueNode.
 * update the neighbors of oldNode.
 */
void merge_two_nodes(shared_ptr<Node> trueNode,shared_ptr<Node> oldNode){
  for(int i=0; i< trueNode->ptrs.size();i++){

      if(oldNode->ptrs[i]!=nullptr && oldNode->ptrs[i]!=wall){

        trueNode->ptrs[i]=oldNode->ptrs[i];
        trueNode->costs[i]=max(oldNode->costs[i],trueNode->costs[i]);

        int j=(i+2)%4;
        oldNode->ptrs[i]->ptrs[j]=trueNode;
        oldNode->ptrs[i]->costs[j]=trueNode->costs[i];
      } 
    
  }
  graph.erase(graph.begin() + findNodeIndex(oldNode->uid));
  currentNodeIndex=findNodeIndex(trueNode->uid);

}

/**
 * @brief check if a given rfid exists in the graph
 */
bool doesRFIDexist(string new_uid){
  for(shared_ptr<Node> i : graph){
    if(i->uid==new_uid){
      return true;
    }
  }
  return false;
}

/**
 * @brief find a nodePtr that doesnt have all its neighbors mapped.
 * returns &wall if all nodes have been mapped.
 */
shared_ptr<Node> newTarget(){
  queue<shared_ptr<Node>> toVisit;
  toVisit.push(current_Node_Ptr);
  unordered_set<shared_ptr<Node>> visited;
  visited.insert(current_Node_Ptr);
  while(!toVisit.empty()){
    shared_ptr<Node> curr_node=toVisit.front();
    toVisit.pop();
    for(int i=0; i<curr_node->ptrs.size();i++){
      if(curr_node->ptrs[i]==nullptr){
        return curr_node;
      } else if (curr_node->ptrs[i]!=wall){
        toVisit.push(curr_node->ptrs[i]);
        visited.insert(curr_node->ptrs[i]);
      }
    }
  }
  return wall;
}

/**
 * @brief returns a ptr list of nodes that lead to a target_uid.
 * [current_node*, ptr1, ptr2, ptr3, target_uid*]
 */
vector<shared_ptr<Node>> pathToNode(string target_uid){
  queue<shared_ptr<Node>> toVisit;
  unordered_map<shared_ptr<Node>, shared_ptr<Node>> parentMap; 
  // To store the parent of each node 
  unordered_set<shared_ptr<Node>> visited;
  toVisit.push(current_Node_Ptr);
  visited.insert(current_Node_Ptr);
  parentMap[current_Node_Ptr] = nullptr;
  if(findNodeIndex(target_uid)==-1){vector<shared_ptr<Node>> nopath={}; return nopath;}
  ///while
  while(!toVisit.empty()){
    shared_ptr<Node> curr_node=toVisit.front();
    toVisit.pop();

    if(curr_node->uid ==target_uid){
      vector<shared_ptr<Node>> path;
      for (shared_ptr<Node> node = curr_node;node!=nullptr;node=parentMap[node]){
        path.push_back(node);
      }
      reverse(path.begin(), path.end());
      return path;
    }

    for (shared_ptr<Node> ptr_neighbor : curr_node->ptrs){
        if (ptr_neighbor!=nullptr && ptr_neighbor!=wall){
          if (visited.find(ptr_neighbor) == visited.end()){
            toVisit.push(ptr_neighbor);
            visited.insert(ptr_neighbor);
            parentMap[ptr_neighbor] = curr_node;
          }
        }
    }
  }
  vector<shared_ptr<Node>> nopath={}; return nopath;
}

void printFinalGraphState() {
    unordered_set<shared_ptr<Node>> visited;
    queue<shared_ptr<Node>> q;

    if (graph.empty()) {
        cout << "(Empty graph)\n";
        return;
    }

    q.push(graph[0]); // Start from the first node in the graph

    while (!q.empty()) {
        shared_ptr<Node> node = q.front();
        q.pop();

        if (visited.find(node) != visited.end()) continue;
        visited.insert(node);

        if(node==nullptr || node==wall){
          continue;
        }
      
        cout << node->uid << ": { ";
        
        for (int i = 0; i < 4; i++) {
            if (node->ptrs[i]!=nullptr && node->ptrs[i]!=wall) {
                cout << (i == 0 ? "N: " : i == 1 ? "E: " : i == 2 ? "S: " : "W: ");
                cout << "(" << node->ptrs[i]->uid << ", Cost: " << node->costs[i] << ")";
            } else {
                cout << (i == 0 ? "N: None" : i == 1 ? "E: None" : i == 2 ? "S: None" : "W: None");
            }
            if (i < 3) cout << ", ";
        }

        cout << " }\n";

        // Enqueue neighbors for BFS
        for (int i = 0; i < 4; i++) {
            if (node->ptrs[i] && visited.find(node->ptrs[i]) == visited.end()) {
                q.push(node->ptrs[i]);
            }
        }
    }
}