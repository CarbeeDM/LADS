#!/usr/bin/env python3

###############################################################################
#  Node class: Each node has up to 4 absolute-direction neighbors: N, E, S, W.
#  We store them in a dict: node.neighbors = {'N': None, 'E': None, 'S': None, 'W': None}
#
#  We'll also store a "label" like 'S1', 'R2', 'L3', 'T4', 'U1', etc.
###############################################################################
class Node:
    def __init__(self, label):
        self.label = label
        self.neighbors = {'N': None, 'E': None, 'S': None, 'W': None}

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
    # -------------------------------------------------------------------------
    def process_command(self, cmd):
        if cmd == 'S':
            print(f"\n[Robot] Received S (start) -> Resetting stack and navigator state.")
            self.reset()
            nodeS = self.make_new_node('S')
            self.current_node = nodeS
            self.stack.append(('S', nodeS))
            print(f"[Robot] Decision: Start at new location => {nodeS}, facing North")
            return

        if not self.backtrace_mode:
            # Normal Mode
            if cmd in ('L', 'R', 'T'):
                new_node = self.make_new_node(cmd)
                self.link_via_direction(new_node)
                self.stack.append((cmd, new_node))

                if cmd == 'L':
                    print("[Robot] Decision: Move Left.")
                    self.robot_direction = self.left_of[self.robot_direction]
                elif cmd == 'R':
                    print("[Robot] Decision: Move Right.")
                    self.robot_direction = self.right_of[self.robot_direction]
                elif cmd == 'T':
                    print("[Robot] Decision: T-junction -> Move Left (by policy).")
                    self.robot_direction = self.left_of[self.robot_direction]

                print(f"[Robot] Now facing {self.robot_direction}")
                return

            elif cmd == 'U':
                print(f"\n[Robot] Received U -> Enter backtrace mode.")
                new_node = self.make_new_node('U')
                self.link_via_direction(new_node)
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
                self.stack.append((cmd, new_node))
                print(f"[Robot] (Backtrace) Not enough in stack for triple, pushed {cmd}")
                return
            else:
                (u_cmd, u_node) = self.stack[-1]
                if u_cmd != 'U':
                    new_node = self.make_new_node(cmd)
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
                    self.handle_merge_in_backtrace(y_node, cmd)

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
    # -------------------------------------------------------------------------
    def handle_merge_in_backtrace(self, old_node, cmdX):
        new_node = self.make_new_node(cmdX)
        self.merge_two_nodes(old_node, new_node)
        # Force the robot to stand on new_node after the merge
        self.current_node = new_node

    # -------------------------------------------------------------------------
    # make_new_node
    # -------------------------------------------------------------------------
    def make_new_node(self, cmd):
        self.node_counters[cmd] += 1
        label = f"{cmd}{self.node_counters[cmd]}"
        return Node(label)

    # -------------------------------------------------------------------------
    # link_via_direction
    # -------------------------------------------------------------------------
    def link_via_direction(self, new_node):
        if self.current_node is None:
            self.current_node = new_node
            return
        d = self.robot_direction
        self.current_node.neighbors[d] = new_node
        opp = self.back_of[d]
        new_node.neighbors[opp] = self.current_node
        self.current_node = new_node

    # -------------------------------------------------------------------------
    # merge_two_nodes
    # -------------------------------------------------------------------------
    def merge_two_nodes(self, old_node, new_node):
        print(f"[Robot] Merging {old_node.label} into {new_node.label}")
        for d in ('N','E','S','W'):
            if new_node.neighbors[d] is None and old_node.neighbors[d] is not None:
                new_node.neighbors[d] = old_node.neighbors[d]
                opp = self.back_of[d]
                old_node.neighbors[d].neighbors[opp] = new_node

        # If we were on old_node, move to new_node (no longer strictly needed, 
        # because handle_merge_in_backtrace does it unconditionally, but we keep it anyway)
        if self.current_node == old_node:
            self.current_node = new_node

        old_node.neighbors = {'N': None, 'E': None, 'S': None, 'W': None}

    # -------------------------------------------------------------------------
    # debug_print_graph
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
                print(f"    {d} -> {nbr.label}")
        for d in ('N','E','S','W'):
            nbr = node.neighbors[d]
            if nbr:
                self.debug_print_graph(nbr, visited)


###############################################################################
# Simple main
###############################################################################
def main():
    nav = RobotNavigator()
    print("Enter commands one at a time: [S, L, R, T, U], or 'Q' to quit.")
    print("The robot always starts facing North when 'S' is read.\n")

    while True:
        cmd = input("Command> ").strip().upper()
        if cmd == 'Q':
            print("Exiting.")
            break
        elif cmd in ['S', 'L', 'R', 'T', 'U']:
            nav.process_command(cmd)
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