#include <iostream>
#include <string>
#include <regex>
#include <map>
#include <vector>
#include <algorithm>
#include <limits>
#include <sstream>

// Define possible directions
const std::map<char, std::map<char, std::string>> TURN_MAP = {
    {'N', {{'N', "Forward"}, {'E', "Right"}, {'S', "U-Turn"}, {'W', "Left"}}},
    {'E', {{'N', "Left"}, {'E', "Forward"}, {'S', "Right"}, {'W', "U-Turn"}}},
    {'S', {{'N', "U-Turn"}, {'E', "Left"}, {'S', "Forward"}, {'W', "Right"}}},
    {'W', {{'N', "Right"}, {'E', "U-Turn"}, {'S', "Left"}, {'W', "Forward"}}}
};

// Graph data structure
typedef std::map<char, std::pair<std::string, int>> DirectionMap;
typedef std::map<std::string, DirectionMap> Graph;

// Function to parse the graph from the input string
Graph parse_graph(const std::string &input_string) {
    Graph graph;
    std::istringstream iss(input_string);
    std::string line;

    // Regex to match node lines like "S1: { N: (T1, Cost: 6), E: None, S: (R5, Cost: 1), W: None }"
    std::regex node_regex(R"((\w+):\s*\{(.*)\})");

    // Regex to match each neighbor direction/cost pair: "N: (T1, Cost: 6)"
    std::regex neighbor_regex(R"(([NSEW]):\s*\((\w+),\s*Cost:\s*(\d+)\))");

    while (std::getline(iss, line)) {
        std::smatch node_match;
        if (std::regex_search(line, node_match, node_regex)) {
            std::string node = node_match[1];
            std::string neighbors_str = node_match[2];

            // Initialize all directions as empty
            DirectionMap directions;
            directions['N'] = {"", -1};
            directions['E'] = {"", -1};
            directions['S'] = {"", -1};
            directions['W'] = {"", -1};

            // Find all valid neighbors
            auto nb_begin = std::sregex_iterator(neighbors_str.begin(), neighbors_str.end(), neighbor_regex);
            auto nb_end = std::sregex_iterator();

            for (auto it = nb_begin; it != nb_end; ++it) {
                std::smatch m = *it;
                char direction = m[1].str()[0];
                std::string neighbor = m[2];
                int cost = std::stoi(m[3]);
                directions[direction] = {neighbor, cost};
            }

            // Store in graph
            graph[node] = directions;
        }
    }

    return graph;
}

// Dijkstra's Algorithm with turn instructions
std::pair<std::vector<std::string>, int> dijkstra(const Graph &graph,
                                                  const std::string &start,
                                                  const std::string &end,
                                                  std::vector<std::string> &turns,
                                                  char initial_direction)
{
    std::map<std::string, int> distances;
    std::map<std::string, std::string> predecessors;
    std::map<std::string, char> directions_taken;

    for (const auto &kv : graph) {
        distances[kv.first] = std::numeric_limits<int>::max();
    }
    distances[start] = 0;

    std::vector<std::string> unvisited;
    for (const auto &kv : graph) {
        unvisited.push_back(kv.first);
    }

    auto get_smallest_distance_node = [&](const std::vector<std::string> &nodes) {
        std::string best_node;
        int min_dist = std::numeric_limits<int>::max();
        for (const auto &node : nodes) {
            if (distances[node] < min_dist) {
                min_dist = distances[node];
                best_node = node;
            }
        }
        return best_node;
    };

    while (!unvisited.empty()) {
        std::string current_node = get_smallest_distance_node(unvisited);
        if (distances[current_node] == std::numeric_limits<int>::max()) break;

        unvisited.erase(std::remove(unvisited.begin(), unvisited.end(), current_node), unvisited.end());

        for (const auto &dir_pair : graph.at(current_node)) {
            char direction = dir_pair.first;
            std::string neighbor = dir_pair.second.first;
            int cost = dir_pair.second.second;

            if (neighbor.empty() || cost < 0) continue;

            int alt = distances[current_node] + cost;
            if (alt < distances[neighbor]) {
                distances[neighbor] = alt;
                predecessors[neighbor] = current_node;
                directions_taken[neighbor] = direction;
            }
        }

        if (current_node == end) break;
    }

    std::vector<std::string> path;
    if (distances[end] == std::numeric_limits<int>::max()) return {path, -1};

    std::string node = end;
    while (predecessors.find(node) != predecessors.end()) {
        path.push_back(node);
        node = predecessors[node];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());

    // Compute turns based on movement direction
    char current_direction = initial_direction;
    for (size_t i = 1; i < path.size(); ++i) {
        std::string prev = path[i - 1];
        std::string curr = path[i];
        char new_direction = directions_taken[curr];

        if (TURN_MAP.count(current_direction) && TURN_MAP.at(current_direction).count(new_direction)) {
            turns.push_back(TURN_MAP.at(current_direction).at(new_direction));
        }

        current_direction = new_direction;
    }

    return {path, distances[end]};
}

int main() {
    std::string input_string = R"(
START_NODE: { N: (INT_4_T, Cost: 7), E: None, S: None, W: None }
INT_4_T: { N: (INT_5_R, Cost: 2), E: (INT_3_L, Cost: 3), S: (START_NODE, Cost: 7), W: None }
INT_5_R: { N: None, E: None, S: (INT_4_T, Cost: 2), W: None }
INT_3_L: { N: (INT_2_U, Cost: 4), E: None, S: None, W: (INT_4_T, Cost: 3) }
INT_2_U: { N: None, E: None, S: (INT_3_L, Cost: 4), W: None }
)";

    Graph graph = parse_graph(input_string);
    std::string start_node = "START_NODE";
    std::string end_node = "INT_2_U";
    char initial_direction = 'N';
    std::vector<std::string> turns;

    auto result = dijkstra(graph, start_node, end_node, turns, initial_direction);
    const auto &path = result.first;
    int cost = result.second;

    std::cout << "Robot direction: " << initial_direction << "\n";
    std::cout << "Shortest path from " << start_node << " to " << end_node << ": ";
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << path[i];
        if (i + 1 < path.size()) std::cout << " -> ";
    }
    std::cout << "\nNecessary turns: ";
    for (size_t i = 0; i < turns.size(); ++i) {
        std::cout << turns[i];
        if (i + 1 < turns.size()) std::cout << ", ";
    }
    std::cout << "\nTotal cost: " << cost << std::endl;

    return 0;
}