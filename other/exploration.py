#!/usr/bin/env python3

###############################################################################
#  Node class: Each node has up to 4 absolute-direction neighbors: N, E, S, W.
#  We store them in a dict: node.neighbors = {'N': None, 'E': None, 'S': None, 'W': None}
#
#  We'll also store a "label" like 'S1', 'R2', 'L3', 'T4', 'U1', etc.
#  <<< NEW >>> We add a parallel dict node.costs to store edge costs.
###############################################################################
class Node:
    def __init__(self, label):
        self.label = label
        self.neighbors = {'N': None, 'E': None, 'S': None, 'W': None}
        # <<< NEW >>>
        self.costs     = {'N': None, 'E': None, 'S': None, 'W': None}

    def __repr__(self):
        return f"Node({self.label})"


###############################################################################
#  RobotNavigator
#    - We push (command, node) onto self.stack
#    - In backtrace mode, we interpret triples (Y, U, X) and merge the Y-node
#      into a newly created X-node, then forcibly move the robot to the new node.
###############################################################################
class RobotNavigator:
    def __init__(self):
        self.stack = []             # Will store tuples (cmd, node)
        self.backtrace_mode = False

        # Keep numeric counters for each type of node
        self.node_counters = {'S': 0, 'R': 0, 'L': 0, 'T': 0, 'U': 0}

        # Current node and direction
        self.current_node = None
        self.robot_direction = 'N'  # start facing North by default

        # Direction utilities
        self.left_of  = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}
        self.right_of = {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}
        self.back_of  = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}

    def reset(self):
        self.stack.clear()
        self.backtrace_mode = False
        self.current_node = None
        self.robot_direction = 'N'

    # -------------------------------------------------------------------------
    # process_command
    #
    # <<< NEW >>> Added a second parameter "cost" (default 0).
    # -------------------------------------------------------------------------
    def process_command(self, cmd, cost=0):
        if cmd == 'S':
        # CHANGE #1: Instead of resetting, we check if S already exists:
            if self.node_counters['S'] == 0:
                print(f"\n[Robot] Received S (start) -> Creating the first S node.")
                nodeS = self.make_new_node('S')
                self.current_node = nodeS
                self.stack.append(('S', nodeS))
                print(f"[Robot] Decision: Start at new location => {nodeS}, facing North")
            else:
                # Already have an S. Re-encountering S means: link current_node with that old S
                print("\n[Robot] Received S -> Re-encountering S; linking to existing S1.")
                # The first item in self.stack should be ('S', S1) if we ever created an S
                (_, s_node) = self.stack[0]  # s_node is S1
                old_node = self.current_node
                if old_node != s_node:
                    # Instead of a full "merge_two_nodes" (which merges adjacency lists),
                    # you might just want to pick a direction from R5 to S1 (or vice versa).
                    #
                    # For example, if you want to link S1 to the South of R5 with cost=10:
                    d = self.robot_direction
                    print(d)
                    opp = self.back_of[d]  # 'N'
                    old_node.neighbors[d] = s_node
                    old_node.costs[d] = cost
                    s_node.neighbors[opp] = old_node
                    s_node.costs[opp] = cost

                self.current_node = s_node
                print(f"[Robot] Decision: Rejoin existing {s_node}")
            return

        if not self.backtrace_mode:
            # Normal Mode
            if cmd in ('L', 'R', 'T'):
                new_node = self.make_new_node(cmd)
                # <<< NEW >>> Pass cost to link_via_direction
                self.link_via_direction(new_node, cost)
                self.stack.append((cmd, new_node))

                if cmd == 'L':
                    print("[Robot] Decision: Move Left.")
                    self.robot_direction = self.left_of[self.robot_direction]
                elif cmd == 'R':
                    print("[Robot] Decision: Move Right.")
                    self.robot_direction = self.right_of[self.robot_direction]
                elif cmd == 'T':
                    print("[Robot] Decision: T-junction -> Move Left (by policy).")
                    self.robot_direction = self.right_of[self.robot_direction]

                print(f"[Robot] Now facing {self.robot_direction}")
                return

            elif cmd == 'U':
                print(f"\n[Robot] Received U -> Enter backtrace mode.")
                new_node = self.make_new_node('U')
                # <<< NEW >>> Pass cost to link_via_direction
                self.link_via_direction(new_node, cost)
                self.stack.append(('U', new_node))

                self.backtrace_mode = True
                self.robot_direction = self.back_of[self.robot_direction]
                print(f"[Robot] Turned 180°, now facing {self.robot_direction}")
                return

        else:
            # Backtrace Mode => interpret triple (Y, U, X)
            if len(self.stack) < 2:
                # Not enough for a triple
                new_node = self.make_new_node(cmd)
                # <<< NEW >>> We might still link with cost if needed,
                # but typically in this code path there's no old node
                # to link to. So do it only if self.current_node is set:
                if self.current_node:
                    self.link_via_direction(new_node, cost)
                self.stack.append((cmd, new_node))
                print(f"[Robot] (Backtrace) Not enough in stack for triple, pushed {cmd}")
                return
            else:
                (u_cmd, u_node) = self.stack[-1]
                if u_cmd != 'U':
                    new_node = self.make_new_node(cmd)
                    if self.current_node:
                        self.link_via_direction(new_node, cost)
                    self.stack.append((cmd, new_node))
                    print("[Robot] (Backtrace) Unexpected top, pushing:", cmd)
                    return

                (y_cmd, y_node) = self.stack[-2]
                triple = (y_cmd, 'U', cmd)

                decision, error, remain_backtrace = self.interpret_triple(triple)
                if error:
                    print("[Robot] ERROR: Map is corrupted for triple:", triple)
                    return

                if decision is not None:
                    print(f"[Robot] (Backtrace) Triple {triple} -> Decision: {decision}")
                    # Merge y_node => new X-node
                    self.handle_merge_in_backtrace(y_node, cmd, cost)

                    # Turn if needed
                    if decision == 'L':
                        self.robot_direction = self.left_of[self.robot_direction]
                    elif decision == 'R':
                        self.robot_direction = self.right_of[self.robot_direction]
                    elif decision == 'forward':
                        pass
                    print(f"[Robot] Now facing {self.robot_direction}")

                # Pop the (U, u_node) and (y_cmd, y_node)
                self.stack.pop()
                self.stack.pop()

                if remain_backtrace:
                    self.stack.append(('U', u_node))
                else:
                    self.backtrace_mode = False

    # -------------------------------------------------------------------------
    # interpret_triple
    # -------------------------------------------------------------------------
    def interpret_triple(self, triple):
        (Y, U, X) = triple
        if (Y, U, X) == ('R', 'U', 'L'):
            return ('L', False, True)
        elif (Y, U, X) == ('R', 'U', 'T'):
            return ('R', False, False)
        elif (Y, U, X) == ('L', 'U', 'R'):
            return ('R', False, True)
        elif (Y, U, X) == ('L', 'U', 'L'):
            return (None, True, True)
        elif (Y, U, X) == ('L', 'U', 'T'):
            return ('L', False, False)
        elif (Y, U, X) == ('T', 'U', 'T'):
            return (None, True, True)
        elif (Y, U, X) == ('T', 'U', 'R'):
            return (None, True, True)
        elif (Y, U, X) == ('T', 'U', 'L'):
            return ('forward', False, False)
        else:
            return (None, True, True)

    # -------------------------------------------------------------------------
    # handle_merge_in_backtrace
    #
    # IMPORTANT FIX: after merging, we forcibly set current_node = new_node
    # so the robot "stands" on the newly created node, not the old one (like U1).
    #
    # <<< NEW >>> We add "cost" so we can link the newly created node properly.
    # -------------------------------------------------------------------------
    def handle_merge_in_backtrace(self, old_node, cmdX, cost):
        new_node = self.make_new_node(cmdX)
        self.merge_two_nodes(old_node, new_node)
        # Force the robot to stand on new_node after the merge
        self.current_node = new_node
        # Optionally link them with the given cost if that makes sense
        # Typically, "merge_two_nodes" already merges edges, but if you
        # want a direct link cost between old_node and new_node (rarely needed),
        # you could do something similar to link_via_direction. In many maze
        # scenarios, the "merge" is enough.

    # -------------------------------------------------------------------------
    # make_new_node
    # -------------------------------------------------------------------------
    def make_new_node(self, cmd):
        self.node_counters[cmd] += 1
        label = f"{cmd}{self.node_counters[cmd]}"
        return Node(label)

    # -------------------------------------------------------------------------
    # link_via_direction
    #
    # <<< NEW >>> We accept a cost, store it in both directions.
    # If an edge already exists with a cost, we keep the higher cost
    # if the new one is bigger (as requested).
    # -------------------------------------------------------------------------
    def link_via_direction(self, new_node, cost=0):
        if self.current_node is None:
            self.current_node = new_node
            return

        d = self.robot_direction
        # Link in the forward direction
        old_neighbor = self.current_node.neighbors[d]
        self.current_node.neighbors[d] = new_node

        old_cost = self.current_node.costs[d]
        if old_cost is None:
            self.current_node.costs[d] = cost
        else:
            # Use the higher cost if the new cost is greater
            self.current_node.costs[d] = max(old_cost, cost)

        # Link in the opposite direction
        opp = self.back_of[d]
        new_node.neighbors[opp] = self.current_node

        old_cost_new = new_node.costs[opp]
        if old_cost_new is None:
            new_node.costs[opp] = cost
        else:
            new_node.costs[opp] = max(old_cost_new, cost)

        # Move current_node to the new_node
        self.current_node = new_node

    # -------------------------------------------------------------------------
    # merge_two_nodes
    #
    # <<< NEW >>> When merging, if both nodes have a cost for a direction,
    # keep the higher cost.
    # -------------------------------------------------------------------------
    def merge_two_nodes(self, old_node, new_node):
        print(f"[Robot] Merging {old_node.label} into {new_node.label}")
        for d in ('N','E','S','W'):
            # If new_node has no neighbor in direction d, but old_node does,
            # we merge that neighbor into new_node's adjacency.
            if new_node.neighbors[d] is None and old_node.neighbors[d] is not None:
                new_node.neighbors[d] = old_node.neighbors[d]

                # Merge the cost with "max" logic
                cost_old = old_node.costs[d]
                new_node_cost = new_node.costs[d]
                if new_node_cost is None:
                    new_node.costs[d] = cost_old
                else:
                    new_node.costs[d] = max(new_node_cost, cost_old)

                opp = self.back_of[d]
                # Update the old neighbor's opposite link to point to new_node
                old_node.neighbors[d].neighbors[opp] = new_node

                # Also merge cost in that neighbor's opposite direction
                old_neighbor_cost = old_node.neighbors[d].costs[opp]
                if old_neighbor_cost is None:
                    old_node.neighbors[d].costs[opp] = new_node.costs[d]
                else:
                    old_node.neighbors[d].costs[opp] = max(
                        old_neighbor_cost, new_node.costs[d]
                    )

        # If we were on old_node, move to new_node
        if self.current_node == old_node:
            self.current_node = new_node

        # Wipe old_node's connections
        old_node.neighbors = {'N': None, 'E': None, 'S': None, 'W': None}
        old_node.costs     = {'N': None, 'E': None, 'S': None, 'W': None}

    # -------------------------------------------------------------------------
    # debug_print_graph
    #
    # <<< NEW >>> Print the cost next to each neighbor.
    # -------------------------------------------------------------------------
    def debug_print_graph(self, node, visited=None):
        if visited is None:
            visited = set()
        if not node or node in visited:
            return
        visited.add(node)
        print(f"  {node.label} neighbors:")
        for d in ('N','E','S','W'):
            nbr = node.neighbors[d]
            if nbr:
                # Show cost if present (default 0 if None)
                c = node.costs[d] if node.costs[d] is not None else 0
                print(f"    {d} -> {nbr.label}, Cost: {c}")
        for d in ('N','E','S','W'):
            nbr = node.neighbors[d]
            if nbr:
                self.debug_print_graph(nbr, visited)

                
    def print_final_map(self, start_node):
        visited = set()
        stack = [start_node]
        while stack:
            node = stack.pop()
            if node in visited:
                continue
            visited.add(node)

            # Build the neighbor info for printing
            parts = []
            for d in ('N', 'E', 'S', 'W'):
                nbr = node.neighbors[d]
                if nbr:
                    c = node.costs[d] if node.costs[d] is not None else 0
                    parts.append(f"{d}: ({nbr.label}, Cost: {c})")
                else:
                    parts.append(f"{d}: None")

            print(f"{node.label}: " + "{ " + ", ".join(parts) + " }")

            # Push neighbors so we can visit them too
            for d in ('N', 'E', 'S', 'W'):
                nbr = node.neighbors[d]
                if nbr and nbr not in visited:
                    stack.append(nbr)


###############################################################################
# Simple main
#
# <<< NEW >>> We minimally extend the logic so that users can type:
#   S 0
#   L 7
#   R 3
#   U 4
# etc.
# If there's no second token, we assume cost=0.
#
# We keep the same prints and structure, just parse cost beforehand.
###############################################################################
def main():
    nav = RobotNavigator()
    print("Enter commands one at a time: [S, L, R, T, U], or 'Q' to quit.")
    print("The robot always starts facing North when 'S' is read.\n")

    while True:
        raw = input("Command> ").strip()
        if not raw:
            continue
        parts = raw.split()
        c = parts[0].upper()

        if c == 'Q':
            if nav.stack and nav.stack[0][0] == 'S':
                print("\nFinal map after all backtracing:")
                nav.print_final_map(nav.stack[0][1])  # the first 'S' node
            break
        elif c in ['S', 'L', 'R', 'T', 'U']:
            # <<< NEW >>> parse cost if provided
            if len(parts) > 1:
                try:
                    cost = int(parts[1])
                except ValueError:
                    cost = 0
            else:
                cost = 0

            nav.process_command(c, cost)
            print("[Robot] Stack:", nav.stack)
            print("[Robot] Current Node:", nav.current_node)
            print("[Robot] Current Direction:", nav.robot_direction)
            print("Graph from current node:")
            nav.debug_print_graph(nav.current_node)
            print("-----------------------------------------------------------")
        else:
            print("Invalid command. Please enter one of [S, L, R, T, U] or 'Q'.")


if __name__ == "__main__":
    main()