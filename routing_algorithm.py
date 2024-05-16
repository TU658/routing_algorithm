from helpers import Map, load_map_10, load_map_40, show_map
import math

map_10 = load_map_10()
# show_map(map_10)

# map_10.intersections

# map_10.roads[0] 

# map_10.roads

map_40 = load_map_40()
# show_map(map_40)

# show_map(map_40, start=5, goal=34, path=[5,16,37,12,34])

class PathPlanner():
    """Construct a PathPlanner Object"""
    def __init__(self, M, start=None, goal=None):
        """ """
        self.map = M
        self.start= start
        self.goal = goal
        self.closedSet = self.create_closedSet() if goal != None and start != None else None
        self.openSet = self.create_openSet() if goal != None and start != None else None
        self.cameFrom = self.create_cameFrom() if goal != None and start != None else None
        self.gScore = self.create_gScore() if goal != None and start != None else None
        self.fScore = self.create_fScore() if goal != None and start != None else None
        self.path = self.run_search() if self.map and self.start != None and self.goal != None else None
    
    def reconstruct_path(self, current):
        """ Reconstructs path after search """
        total_path = [current]
        while current in self.cameFrom.keys():
            current = self.cameFrom[current]
            total_path.append(current)
        return total_path
    
    def _reset(self):
        """Private method used to reset the closedSet, openSet, cameFrom, gScore, fScore, and path attributes"""
        self.closedSet = None
        self.openSet = None
        self.cameFrom = None
        self.gScore = None
        self.fScore = None
        self.path = self.run_search() if self.map and self.start and self.goal else None

    def run_search(self):
        """ """
        if self.map == None:
            raise(ValueError, "Must create map before running search. Try running PathPlanner.set_map(start_node)")
        if self.goal == None:
            raise(ValueError, "Must create goal node before running search. Try running PathPlanner.set_goal(start_node)")
        if self.start == None:
            raise(ValueError, "Must create start node before running search. Try running PathPlanner.set_start(start_node)")

        self.openSet = self.openSet if self.openSet != None else  self.create_openSet()
        self.cameFrom = self.cameFrom if self.cameFrom != None else  self.create_cameFrom()
        self.gScore = self.gScore if self.gScore != None else  self.create_gScore()
        self.fScore = self.fScore if self.fScore != None else  self.create_fScore()

        while not self.is_open_empty():
            current = self.get_current_node()
#             print('current node: \t\t\t\t\t\t', current)
#             print(self.closedSet)
            if current == self.goal:
                self.path = [x for x in reversed(self.reconstruct_path(current))]
                return self.path
            else:
                self.openSet.remove(current)
                self.closedSet.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closedSet:
                    continue    # Ignore the neighbor which is already evaluated.

                if not neighbor in self.openSet:    # Discover a new node
#                     self.get_gScore(neighbor)
#                     print('get_gScore neighbor', self.gScore.get(neighbor))
                    self.openSet.add(neighbor)
                    
#                 print('openSet: ', self.openSet)
#                 print('gScore of neighbor: ', neighbor, self.get_gScore(neighbor))
#                 print('heuristic cost estimate: ', neighbor, self.heuristic_cost_estimate(neighbor))
#                 print('fScore of neighbor: ', neighbor, self.calculate_fscore(neighbor))
                # The distance from start to a neighbor
                #the "dist_between" function may vary as per the solution requirements.
#                 print('get tentative gScore: ', self.get_tentative_gScore(current, neighbor), 'get gScore: ', self.get_gScore(neighbor))
                if self.get_tentative_gScore(current, neighbor) >= self.get_gScore(neighbor):
                    continue        # This is not a better path.

                # This path is the best until now. Record it!
                self.record_best_path_to(current, neighbor)
        
                
        print("No Path Found")
        return False
    
def create_closedSet(self):
    """ Creates and returns a data structure suitable to hold the set of nodes already evaluated"""
    # EXAMPLE: return a data structure suitable to hold the set of nodes already evaluated
    return set()

def create_openSet(self):
    """ Creates and returns a data structure suitable to hold the set of currently discovered nodes 
    that are not evaluated yet. Initially, only the start node is known."""
    if self.start != None:
        #  return a data structure suitable to hold the set of currently discovered nodes 
        # that are not evaluated yet. Make sure to include the start node.
        return {self.start}
    
    raise(ValueError, "Must create start node before creating an open set. Try running PathPlanner.set_start(start_node)")

def create_cameFrom(self):
    """Creates and returns a data structure that shows which node can most efficiently be reached from another,
    for each node."""
    #  return a data structure that shows which node can most efficiently be reached from another,
    # for each node.
    return {}
    
def create_gScore(self):
    """Creates and returns a data structure that holds the cost of getting from the start node to that node, 
    for each node. The cost of going from start to start is zero."""
    #   return a data structure that holds the cost of getting from the start node to that node, for each node.
    # for each node. The cost of going from start to start is zero. The rest of the node's values should 
    # be set to infinity.
    gScore = {}
    for node in self.map.intersections:
        if node == self.start:
            gScore[node] = 0
        else:   
            gScore[node] = float('inf')
        
    return gScore

def create_fScore(self):
    """Creates and returns a data structure that holds the total cost of getting from the start node to the goal
    by passing by that node, for each node. That value is partly known, partly heuristic.
    For the first node, that value is completely heuristic."""
    #  return a data structure that holds the total cost of getting from the start node to the goal
    # by passing by that node, for each node. That value is partly known, partly heuristic.
    # For the first node, that value is completely heuristic. The rest of the node's value should be 
    # set to infinity.
    fScore = {}
    for node in self.map.intersections:
        if node == self.goal:
            fScore[node] = 0
        else:
            fScore[node] = float('inf')
    return fScore

def set_map(self, M):
    """Method used to set map attribute """
    self._reset(self)
    self.start = None
    self.goal = None
    #  Set map to new value.
    self.map = M

def set_start(self, start):
    """Method used to set start attribute """
    self._reset(self)
    #  Set start value. Remember to remove goal, closedSet, openSet, cameFrom, gScore, fScore, 
    # and path attributes' values.
    self.start = start
    
def set_goal(self, goal):
    """Method used to set goal attribute """
    self._reset(self)
    #  Set goal value. 
    self.goal = goal

def is_open_empty(self):
    """returns True if the open set is empty. False otherwise. """
    #  Return True if the open set is empty. False otherwise.
    return len(self.openSet) == 0

def get_current_node(self):
    """ Returns the node in the open set with the lowest value of f(node)."""
    #  Return the node in the open set with the lowest value of f(node).
    min_fscore_node = None
    min_fscore = float('inf')
    for node in self.openSet:
        fscore = self.calculate_fscore(node)
#         print('fscore of node: ', node, fscore)
        if fscore < min_fscore:
            min_fscore = fscore
            min_fscore_node = node
    return min_fscore_node

def get_neighbors(self, node):
    """Returns the neighbors of a node"""
    #  Return the neighbors of a node
#     print(self.map.roads[node])
    return self.map.roads[node]

def get_gScore(self, node):
    """Returns the g Score of a node"""
#     print('get_gScore node 24 of 10: ', self.gScore.get(24) + self.distance(self.start, 24))
#     print('heuristic cosst node 24 of goal: ', self.heuristic_cost_estimate(24))
#     if node == self.goal:
#         g_score = self.distance(self.start, node)
#     else:
#         g_score = self.gScore.get(node) + self.distance(self.start, node)
    if node == self.goal:
        return self.gScore.get(node) + self.distance(node, self.goal)
    return self.gScore.get(node) + self.distance(self.start, node)

def distance(self, node_1, node_2):
    """ Computes the Euclidean L2 Distance"""
    #  Compute and return the Euclidean L2 Distance
    x1, y1 = self.map.intersections[node_1]
    x2, y2 = self.map.intersections[node_2]
#     print('distance used', 'node_1: ', node_1, 'node_2: ', node_2, 'distance: ', math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def get_tentative_gScore(self, current, neighbor):
    """Returns the tentative g Score of a node"""
    #  Return the g Score of the current node 
    # plus distance from the current node to it's neighbors
#     print('get tentative gScore used', 'current: ', current, 'neighbor: ', neighbor, 'distance: ', 
#          self.get_gScore(current) + self.distance(current, neighbor))
    return self.get_gScore(current) + self.distance(current, neighbor)

def heuristic_cost_estimate(self, node):
    """ Returns the heuristic cost estimate of a node """
    #  Return the heuristic cost estimate of a node
#     x1, y1 = self.map.intersections[node]
#     x2, y2 = self.map.intersections[self.goal]
# #     print('heuristic cost estimate used', node, 'to', self.goal, '=', math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))
#     return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return self.distance(node, self.goal)

def calculate_fscore(self, node):
    """Calculate the f score of a node. """
    #  Calculate and returns the f score of a node. 
    # REMEMBER F = G + H
#     print('intersection of node: ', node, self.map.intersections[node])
#     print('calculate fscore used ','gScore: ', self.gScore.get(node), 'fScore: ', self.heuristic_cost_estimate(node), 'total: ',
#          self.gScore.get(node) + self.heuristic_cost_estimate(node))
#     print('gscore 10 - 24: ', self.get_gScore(10) + self.distance(10, 24))
#     print('gscore 18 - 24: ', self.get_gScore(18) + self.distance(18, 24))
    return self.gScore.get(node) + self.heuristic_cost_estimate(node)

def record_best_path_to(self, current, neighbor):
    """Record the best path to a node """
    #  Record the best path to a node, by updating cameFrom, gScore, and fScore
    self.cameFrom[neighbor] = current
    self.gScore[neighbor] = self.get_gScore(current)
    self.fScore[neighbor] = self.calculate_fscore(neighbor)

PathPlanner.create_closedSet = create_closedSet
PathPlanner.create_openSet = create_openSet
PathPlanner.create_cameFrom = create_cameFrom
PathPlanner.create_gScore = create_gScore
PathPlanner.create_fScore = create_fScore
PathPlanner.set_map = set_map
PathPlanner.set_start = set_start
PathPlanner.set_goal = set_goal
PathPlanner.is_open_empty = is_open_empty
PathPlanner.get_current_node = get_current_node
PathPlanner.get_neighbors = get_neighbors
PathPlanner.get_gScore = get_gScore
PathPlanner.distance = distance
PathPlanner.get_tentative_gScore = get_tentative_gScore
PathPlanner.heuristic_cost_estimate = heuristic_cost_estimate
PathPlanner.calculate_fscore = calculate_fscore
PathPlanner.record_best_path_to = record_best_path_to

planner = PathPlanner(map_40, 5, 34)
path = planner.path
if path == [5, 16, 37, 12, 34]:
    print("great! Your code works for these inputs!")
    print(path)
else:
    print("something is off, your code produced the following:")
    print(path)

#show test
start = 38
goal = 11
show_map(map_40, start=start, goal=goal, path=PathPlanner(map_40, start, goal).path)