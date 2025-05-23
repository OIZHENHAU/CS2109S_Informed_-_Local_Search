import heapq

class Node:
    r"""Node class for search tree
    Args:
        parent (Node): the parent node of this node in the tree
        act (Action): the action taken from parent to reach this node
        state (State): the state of this node
        g_n (float): the path cost of reaching this state
        h_n (float): the heuristic value of this state
    """
    
    def __init__(
            self, 
            parent: "Node", 
            act, 
            state, 
            g_n: float = 0.0, 
            h_n: float = 0.0):

        self.parent = parent # where am I from
        self.act = act # how to get here
        self.state = state # who am I
        self.g_n = g_n # what it costs to be here g(n)
        self.h_n = h_n # heuristic function h(n)
    
    def get_fn(self):
        r"""
        Returns the sum of heuristic and cost of the node
        """
        return self.g_n + self.h_n

    def __str__(self):
        return str(self.state)

    def __lt__(self, node):
        """Compare the path cost between states"""
        return self.g_n < node.g_n

    def __eq__(self, node):
        """Compare whether two nodes have the same state"""
        return isinstance(node, Node) and self.state == node.state

    def __hash__(self):
        """Node can be used as a KeyValue"""
        return hash(self.state)


class PriorityQueue:
    def __init__(self):
        self.heap = []

    def __contains__(self, node):
        """Decide whether the node (state) is in the queue"""
        return any([item == node for _, item in self.heap])

    def __delitem__(self, node):
        """Delete the an existing node in the queue"""
        try: 
            del self.heap[[item == node for _, item in self.heap].index(True)]
        except ValueError:
            raise KeyError(str(node) + " is not in the queue")
        heapq.heapify(self.heap) # O(n)

    def __getitem__(self, node):
        """Return the priority of the given node in the queue"""
        for value, item in self.heap:
            if item == node:
                return value
        raise KeyError(str(node) + " is not in the queue")

    def __len__(self):
        return len(self.heap)

    def __repr__(self):
        string = '['
        for priority, node in self.heap:
            string += f"({priority}, {node}), "
        string += ']'
        return string

    def push(self, priority, node):
        """Enqueue node with priority"""
        heapq.heappush(self.heap, (priority, node))

    def pop(self):
        """Dequeue node with highest priority (the minimum one)"""
        if self.heap:
            return heapq.heappop(self.heap)[1]
        else:
            raise Exception("Empty priority queue")

    def get_priority(self, node):
        return self.__getitem__(node)
