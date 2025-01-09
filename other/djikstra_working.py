import re

def parse_graph(input_string):
    graph = {}

    # Split by lines
    lines = input_string.strip().split("\n")
    for line in lines:
        # Extract node label (e.g., "S1")
        node_match = re.match(r"(\w+):\s\{(.*)\}", line.strip())
        if not node_match:
            continue
        
        node, neighbors_str = node_match.groups()
        graph[node] = {}

        # Extract neighbors and costs
        neighbor_matches = re.findall(r"([NSEW]):\s\((\w+),\sCost:\s(\d+)\)", neighbors_str)
        for direction, neighbor, cost in neighbor_matches:
            graph[node][direction] = (neighbor, int(cost))  # Convert cost to integer

        # Fill in missing directions as None
        for direction in ['N', 'E', 'S', 'W']:
            if direction not in graph[node]:
                graph[node][direction] = None

    return graph


def dijkstra(graph, start, end):
    # Initialize distances to infinity, except the start node
    distances = {node: float('inf') for node in graph}
    distances[start] = 0  # Start node has a distance of 0
    predecessors = {}  # Stores the shortest path tree
    unvisited_nodes = list(graph.keys())  # List of unvisited nodes

    while unvisited_nodes:
        # Step 1: Find the node with the smallest distance (manual min selection)
        current_node = min(unvisited_nodes, key=lambda node: distances[node])

        # If the smallest distance is infinity, stop (unreachable node)
        if distances[current_node] == float('inf'):
            break

        # Step 2: Remove this node from unvisited nodes
        unvisited_nodes.remove(current_node)

        # Step 3: Check neighbors
        for direction, neighbor_info in graph[current_node].items():
            if neighbor_info is None:
                continue  # No connection in this direction

            neighbor, cost = neighbor_info
            new_cost = distances[current_node] + cost

            # Update if a shorter path is found
            if new_cost < distances[neighbor]:
                distances[neighbor] = new_cost
                predecessors[neighbor] = current_node  # Track path

        # Stop early if we reached the end node
        if current_node == end:
            break

    # Step 4: Reconstruct the shortest path
    path = []
    node = end
    while node in predecessors:
        path.append(node)
        node = predecessors[node]
    path.append(start)
    path.reverse()

    return path, distances[end] if distances[end] != float('inf') else None


# Input graph as a string
input_string = """S1: { N: (T1, Cost: 6), E: None, S: (R5, Cost: 1), W: None }
R5: { N: (S1, Cost: 1), E: (R4, Cost: 10), S: None, W: None }
R4: { N: (R3, Cost: 10), E: None, S: None, W: (R5, Cost: 10) }
R3: { N: None, E: None, S: (R4, Cost: 10), W: (R2, Cost: 10) }
R2: { N: None, E: (R3, Cost: 10), S: (T1, Cost: 1), W: None }
T1: { N: (R2, Cost: 1), E: None, S: (S1, Cost: 6), W: (L2, Cost: 2) }
L2: { N: (U1, Cost: 4), E: (T1, Cost: 2), S: None, W: None }
U1: { N: None, E: None, S: (L2, Cost: 4), W: None }"""

# Parse the graph from the string
graph = parse_graph(input_string)

# Example: Find shortest path from "S1" to "U1"
start_node = "S1"
end_node = "R3"
shortest_path, cost = dijkstra(graph, start_node, end_node)

print(f"Shortest path from {start_node} to {end_node}: {shortest_path}")
print(f"Total cost: {cost}")