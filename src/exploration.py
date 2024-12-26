import matplotlib.pyplot as plt

# Node class to represent intersections
class Node:
    def __init__(self, x, y, name):
        self.x = x
        self.y = y
        self.name = name

# Plotting function for the robot map
def plot_map(nodes, edges, directions_taken, colors):
    """
    Visualize the path using matplotlib.
      - nodes: list of Node objects
      - edges: list of (iA, iB) index pairs
      - directions_taken: list of direction labels for each edge
      - colors: list of color strings for each node
    """
    plt.figure(figsize=(8, 8))
    
    # Draw edges
    for idx, (iA, iB) in enumerate(edges):
        xA, yA = nodes[iA].x, nodes[iA].y
        xB, yB = nodes[iB].x, nodes[iB].y
        plt.plot([xA, xB], [yA, yB], 'k-', linewidth=2)

        # Annotate directions
        mid_x, mid_y = (xA + xB) / 2, (yA + yB) / 2
        direction_str = directions_taken[idx]
        plt.text(mid_x, mid_y, direction_str, fontsize=10, color='blue',
                 ha='center', va='center', 
                 bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="blue", alpha=0.5))

    # Draw nodes
    for i, node in enumerate(nodes):
        x, y = node.x, node.y
        plt.plot(x, y, 'o', color=colors[i], markersize=10)
        plt.text(x, y, node.name, fontsize=10, ha='right', va='bottom')

    plt.title("Robot Logical Path (DFS Exploration)")
    plt.axis('equal')
    plt.grid(True)
    plt.show()

# Orientation helpers
def turn_right(orientation):
    mapping = {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}
    return mapping[orientation]

def turn_left(orientation):
    mapping = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}
    return mapping[orientation]

def turn_u(orientation):
    mapping = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
    return mapping[orientation]

# Main Robot Mapper Class
class DFSRobotMapper:
    def __init__(self, example_reads):
        self.nodes = []  # List of all nodes
        self.edges = []  # List of edges (iA, iB)
        self.edge_directions = []  # Directions for edges
        self.colors = []  # Colors for nodes
        self.visited_edges = set()  # To prevent revisiting edges
        self.example_reads = example_reads  # Sensor data
        self.cost_scale = 1.0 / 1000.0  # Scale for distance
        self.color_map = {'S': 'pink', 'R': 'green', 'T': 'yellow', 'U': 'orange', 'L': 'purple'}
        self.orientation = 'N'  # Start facing North
        self.current_index = None  # Current node index
        self.start_node_index = None  # Start node

    def add_node(self, x, y, label):
        """Add a node to the graph."""
        idx = len(self.nodes)
        self.nodes.append(Node(x, y, label))
        self.colors.append(self.color_map.get(label[0], 'cyan'))  # Assign a color
        return idx

    def add_edge(self, iA, iB, direction):
        """Add an edge to the graph."""
        self.edges.append((iA, iB))
        self.edge_directions.append(direction)
        self.visited_edges.add((min(iA, iB), max(iA, iB)))

    def is_edge_visited(self, iA, iB):
        """Check if an edge is visited."""
        return (min(iA, iB), max(iA, iB)) in self.visited_edges

    def move(self, distance):
        """Move in the current orientation."""
        cur_x, cur_y = self.nodes[self.current_index].x, self.nodes[self.current_index].y
        if self.orientation == 'N':
            return cur_x, cur_y + distance
        elif self.orientation == 'E':
            return cur_x + distance, cur_y
        elif self.orientation == 'S':
            return cur_x, cur_y - distance
        elif self.orientation == 'W':
            return cur_x - distance, cur_y
        return cur_x, cur_y

    def explore(self):
        """Perform DFS-style exploration based on sensor data."""
        for i, (label, cost) in enumerate(self.example_reads):
            distance = cost * self.cost_scale

            if i == 0:  # Starting node
                cur_x, cur_y = 0, 0
                self.start_node_index = self.add_node(cur_x, cur_y, f"{label}{i}")
                self.current_index = self.start_node_index
                continue

            # Update orientation based on the label
            if label == 'R':
                self.orientation = turn_right(self.orientation)
            elif label == 'L':
                self.orientation = turn_left(self.orientation)
            elif label == 'U':
                self.orientation = turn_u(self.orientation)

            # Move forward based on the distance
            new_x, new_y = self.move(distance)
            new_node_label = f"{label}{i}"
            new_node_index = self.add_node(new_x, new_y, new_node_label)

            # Add an edge
            if not self.is_edge_visited(self.current_index, new_node_index):
                self.add_edge(self.current_index, new_node_index, label)

            # If we sense the start node (S), connect back to it
            if label == 'S' and i != 0:
                self.add_edge(new_node_index, self.start_node_index, "BackToStart")

            # Update the current node
            self.current_index = new_node_index

    def run(self):
        """Run the mapping and visualize."""
        self.explore()
        plot_map(self.nodes, self.edges, self.edge_directions, self.colors)

# Example sensor data
example_reads = [
    ('S', 0),    # Start
    ('R', 2000), # Right turn
    ('T', 3000), # T-intersection
    ('U', 1500), # U-turn
    ('L', 2000), # Left turn (from T)
    ('S', 0)     # Revisit start node
]

# Main execution
if __name__ == "__main__":
    mapper = DFSRobotMapper(example_reads)
    mapper.run()