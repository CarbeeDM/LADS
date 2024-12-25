#include <iostream> 
#include <vector> 
#include <unordered_map> 
#include <string>

using namespace std;
//---------- MAP BEGIN
struct nodeCostPair{
  Node* target;
  float cost;

  nodeCostPair(Node* tag, float incost): target(tag), cost(incost) {}
};
class Node{
  public:
  std::string tag;
  std::vector<nodeCostPair*> neighbors;

  Node(const std::string& tagName) : tag(tagName) {}
};

class Map {
  private:
  std::vector<Node*> nodes;
  std::unordered_map<std::string, Node*> nodeMap;

  public:
  void AddNodeToMap(const std::string& existingTagName, const std::string& newTagName) { 
    float pathCost=1.0;
    Node* newNode = new Node(newTagName); nodes.push_back(newNode);
    nodeMap[newTagName] = newNode;
    Node* existingNode = GetNode(existingTagName);
    if (existingNode) { 
      // Connect the new node to the existing node 
      nodeCostPair* newNodeCostPair = new nodeCostPair(newNode, pathCost);
      nodeCostPair* existingNodeCostPair = new nodeCostPair(existingNode, pathCost);
      existingNode->neighbors.push_back(newNodeCostPair); 
      newNode->neighbors.push_back(existingNodeCostPair); } 
      }
    
    Node* GetNode(const std::string& tagName){
      auto it = nodeMap.find(tagName);
      if (it !=nodeMap.end()){
        return it->second;
      }
      return nullptr;
    }

    ~Map() {
      for (Node* node : nodes) {
        delete node;
      }
    }

  vector<vector<int>> generateGraph(int V) {
    vector<vector<int>> graph(nodeMap.size(), vector<int>(nodeMap.size(), 0));

    return graph;
  }


};

