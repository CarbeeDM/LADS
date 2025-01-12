import re
import networkx as nx
import matplotlib.pyplot as plt
from collections import deque

def parse_graph_description(description: str):
    """
    Parse the input description into an adjacency dictionary of the form:
    {
      'NodeName': {
        'N': ('OtherNode', cost) or None,
        'E': ...,
        'S': ...,
        'W': ...
      },
      ...
    }
    """
    # Split by lines (or semicolons) to get each node's definition
    lines = description.split('\n')
    # Clean empty lines, if any
    lines = [l.strip() for l in lines if l.strip()]

    adjacency = {}

    # Regex to match lines like:
    # S1: { N: (T1, Cost: 6), E: None, S: (R5, Cost: 1), W: None }
    node_pattern = re.compile(
        r'(?P<node>\w+):\s*\{\s*(?P<directions>.*?)\s*\}'
    )

    # Regex to match direction entries like:  N: (T1, Cost: 6)
    # or:  N: None
    direction_pattern = re.compile(
        r'([NESW])\s*:\s*(?P<value>None|\(\s*\w+\s*,\s*Cost:\s*\d+\s*\))'
    )

    # Regex to match "(T1, Cost: 6)"
    tuple_pattern = re.compile(
        r'\(\s*(?P<target>\w+)\s*,\s*Cost:\s*(?P<cost>\d+)\s*\)'
    )

    for line in lines:
        # Extract the node name and the inner directions text
        m = node_pattern.search(line)
        if not m:
            continue
        
        node_name = m.group('node')
        directions_text = m.group('directions')
        
        # Initialize adjacency dict for this node
        adjacency[node_name] = {'N': None, 'E': None, 'S': None, 'W': None}

        # Find all direction-value pairs
        for dmatch in direction_pattern.finditer(directions_text):
            direction = dmatch.group(1)  # N, E, S, or W
            value = dmatch.group('value')
            
            if value != "None":
                # Parse something like "(T1, Cost: 6)"
                tmatch = tuple_pattern.search(value)
                if tmatch:
                    target_node = tmatch.group('target')
                    cost = int(tmatch.group('cost'))
                    adjacency[node_name][direction] = (target_node, cost)
                else:
                    adjacency[node_name][direction] = None
            else:
                adjacency[node_name][direction] = None
    
    return adjacency

def compute_positions(adjacency):
    """
    Given adjacency info with directions (N, E, S, W) and costs,
    compute absolute 2D coordinates for each node. We choose an
    arbitrary node as the 'root' (first in adjacency keys) at (0,0),
    then BFS or DFS outwards, placing each connected node.
    
    Returns:
        pos: dict of node -> (x, y)
    """
    # Offsets for each direction:
    # N: (0, +cost),  E: (+cost, 0),  S: (0, -cost),  W: (-cost, 0)
    direction_offsets = {
        'N': (0, 1),
        'E': (1, 0),
        'S': (0, -1),
        'W': (-1, 0)
    }

    # Pick an arbitrary start node (e.g. the first in adjacency)
    all_nodes = list(adjacency.keys())
    if not all_nodes:
        return {}

    start_node = all_nodes[0]
    pos = {start_node: (0.0, 0.0)}

    # We'll do a BFS from the start node to assign coordinates
    queue = deque([start_node])
    visited = set([start_node])

    while queue:
        current = queue.popleft()
        cx, cy = pos[current]

        # Explore all 4 directions from current
        for direction, info in adjacency[current].items():
            if info is not None:
                target_node, cost = info
                # If we haven't placed target_node yet
                if target_node not in pos:
                    dx, dy = direction_offsets[direction]
                    # new position = current pos + cost * offset
                    nx_ = cx + cost * dx
                    ny_ = cy + cost * dy
                    pos[target_node] = (nx_, ny_)
                # Add to queue if not visited
                if target_node not in visited:
                    visited.add(target_node)
                    queue.append(target_node)

    return pos

def build_networkx_graph(adjacency):
    """
    Build a NetworkX Graph from the adjacency dictionary.
    Each edge is stored once (undirected edge).
    """
    G = nx.Graph()
    
    # Add nodes
    for node in adjacency:
        G.add_node(node)

    # Add edges
    # Each direction can represent the same edge. We'll add only once
    # by requiring an ordering or checking if edge already exists.
    for node, dirs in adjacency.items():
        for direction, info in dirs.items():
            if info is not None:
                target_node, cost = info
                # Add edge if it doesn't exist
                if not G.has_edge(node, target_node):
                    G.add_edge(node, target_node, weight=cost)

    return G

def visualize_graph(description: str):
    """
    Parse the description, compute node positions, build a NetworkX graph,
    and display using matplotlib.
    """
    # 1. Parse
    adjacency = parse_graph_description(description)
    # 2. Compute positions
    pos = compute_positions(adjacency)
    # 3. Build Graph
    G = build_networkx_graph(adjacency)
    # 4. Draw
    plt.figure(figsize=(8, 6))
    nx.draw(G, pos, with_labels=True, node_size=1000, node_color='lightblue')
    # Draw edge labels for the 'cost'
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
    plt.title("Directional Graph Visualization")
    plt.axis("equal")  # so that edges reflect actual distances/directions
    plt.show()


if __name__ == "__main__":
    # The input string
    graph_description = """START_NODE: { N: (INT_0_R, Cost: 6), E: None, S: (INT_5_R, Cost: 1), W: None }
INT_0_R: { N: None, E: None, S: (START_NODE, Cost: 6), W: (INT_1_R, Cost: 7) }
INT_5_R: { N: (START_NODE, Cost: 1), E: None, S: None, W: (rfid_4_P, Cost: 4) }
INT_1_R: { N: None, E: (INT_0_R, Cost: 7), S: (rfid_2_P, Cost: 4), W: None }
rfid_4_P: { N: None, E: (INT_5_R, Cost: 4), S: None, W: (INT_3_R, Cost: 3) }
rfid_2_P: { N: (INT_1_R, Cost: 4), E: None, S: (INT_3_R, Cost: 2), W: None }
INT_3_R: { N: (rfid_2_P, Cost: 2), E: (rfid_4_P, Cost: 3), S: None, W: None }"""

    # Visualize it!
    visualize_graph(graph_description)