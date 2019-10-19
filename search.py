# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    discovered = set() #a set to keep track of discovered nodes
    fringe = util.Stack() 
    fringe.push(problem.getStartState()) #push the start state onto the stack
    directions = {} #a directions dictionary to efficiently keep track of each node's directions
    goal_node = None
    
    #while the stack is not empty, we iterate through each node on the stack
    #we add the current node to the discovered set so we know that is has been searched
    #we then push the successors onto the stack(if they are not yet discovered)  and add their directions into our dictionary
    #then go back through the while loop and do it all again
    while not fringe.isEmpty(): 
        current_node = fringe.pop()
        discovered.add(current_node)
        if (problem.isGoalState(current_node)): #checking if end has been reached and exit
            goal_node = current_node
            break
        successors = problem.getSuccessors(current_node)
        for neighbor, action, cost in successors:
            if neighbor not in discovered:
                fringe.push(neighbor)
                directions[neighbor] = current_node, action
    
    #if no goal_node is found after traversing the graph, no solution is returned
    if goal_node is None:
        return []
    path = []
    current_node = goal_node
    #moving backwards from the goal node, 
    #appending all the directions that we took to get to it into a list
    while current_node is not problem.getStartState():
        parent, direction = directions[current_node]
        path.append(direction)
        current_node = parent
    path.reverse() #since the directions are added in reverse order, we reverse the list to get the final path

    return path
                
def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    discovered = set() #discovered set to keep track of discovered nodes
    fringe = util.Queue()
    discovered.add(problem.getStartState()) #adding the start state into the discovered set
    fringe.push(problem.getStartState())  #pushing the start state into the queue
    directions = {} #directions dictionary
    goal_node = None
    
    #while the queue is not empty, we look at the first node in the queue
    #we then add successors to discovered set(if they are not) and push them onto the queue
    #we then keep track of the successors' directions in the direction dictionary
    #do this again until the queue is empty
    while not fringe.isEmpty():
        current_node = fringe.pop()
        if (problem.isGoalState(current_node)): #if it is the goal, we break out of the loop
            goal_node = current_node
            break
        successors = problem.getSuccessors(current_node)
        for neighbor, action, cost in successors:
            if neighbor not in discovered:
                discovered.add(neighbor)
                fringe.push(neighbor)
                directions[neighbor] = current_node, action
    
    #if no goal node, return no solution
    if goal_node is None:
        return []
    path = []
    current_node = goal_node
    #going through the directions from goal_node to start node, 
    #adding each direction into a list
    while current_node != problem.getStartState():
        print(f"current node bfs {current_node}")
        parent, direction = directions[current_node]
        path.append(direction)
        current_node = parent
    path.reverse() #reversing the list to get the correct order
    return path

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    #just setting some variables, similar to above functions
    discovered = set()
    fringe = util.PriorityQueue() 
    heap = set() #keeping track of what's in the heap
    discovered.add(problem.getStartState())
    fringe.push(problem.getStartState(), 0)
    heap.add(problem.getStartState()) #adding our start state to the heap (^keeping track)
    directions = {}
    cost_from_start = {problem.getStartState(): 0}
    goal_node = None
    
    #while the priority queue is not empty,
    #we observe the current node (with lowest priority), and add is to our discovered set
    #we iterate through the successors, keeping track of total cost
    #if the neighbor is not in discovered nor in the heap, we add this successor into the queue (and heap set)
    #we add our cost for this particular node into a cost dictionary, then we add our directions to a direction dictionary
    #if the neighbor is in the heap and current documented cost is larger than total cost, we update it to the new lower cost
    #loop and do it all again
    while not fringe.isEmpty():
        current_node = fringe.pop()
        heap.remove(current_node) #remove the node from the heap set
        if (problem.isGoalState(current_node)): #once goal state is reached, leave loop
            goal_node = current_node
            break
        discovered.add(current_node)
        successors = problem.getSuccessors(current_node)
        for neighbor, action, cost in successors:
            total_cost = cost_from_start[current_node] + cost
            print(current_node, total_cost, neighbor)
            if neighbor not in discovered and neighbor not in heap:
                fringe.update(neighbor, total_cost)
                heap.add(neighbor)
                cost_from_start[neighbor] = total_cost
                directions[neighbor] = current_node, action
            elif neighbor in heap and cost_from_start[neighbor] > total_cost: #update neighbor if better option is found
                fringe.update(neighbor, total_cost)
                heap.add(neighbor) 
                cost_from_start[neighbor] = total_cost
                directions[neighbor] = current_node, action
    
    #is no goal node, return no solution
    if goal_node is None:
        return []
    path = []
    current_node = goal_node
    #creating path from dictionary
    while current_node is not problem.getStartState():
        parent, direction = directions[current_node]
        path.append(direction)
        current_node = parent
    path.reverse()

    return path

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    #setsetting variables like in algorithms above (same idea)
    discovered = set()
    fringe = util.PriorityQueue() 
    heap = set() #keeping track of what's in the heap
    discovered.add(problem.getStartState())
    fringe.push(problem.getStartState(), 0)
    heap.add(problem.getStartState())
    directions = {}
    cost_from_start = {problem.getStartState(): 0}
    cost_from_nodes = {problem.getStartState(): 0}
    goal_node = None
    
    #while the priority queue is not empty,
    #we look at the current node (pop it), remove it from our heap set as well
    #add it to discovered set and then iterate through successors
    #same idea as ufs, but we need to keep track of the cost between nodes rather 
    #than simply the total cost so we can keep track of which path is the best option 
    #with the heuristic taken into consideration
    while not fringe.isEmpty():
        current_node = fringe.pop()
        heap.remove(current_node)
        if (problem.isGoalState(current_node)):
            goal_node = current_node
            break
        discovered.add(current_node)
        successors = problem.getSuccessors(current_node)
        for neighbor, action, cost in successors: 
            cost_between_nodes = cost_from_nodes[current_node] + cost #keeping track of the cost between nodes (without heuristic)
            total_cost = cost_between_nodes + heuristic(neighbor, problem)
            if neighbor not in discovered and neighbor not in heap:
                fringe.update(neighbor, total_cost) 
                heap.add(neighbor)
                cost_from_nodes[neighbor] = cost_between_nodes #the cost from initial node to current node
                cost_from_start[neighbor] = total_cost #the cost from the initial node to the current node + th the heuristitc value
                directions[neighbor] = current_node, action #updating directions
            elif neighbor in heap and cost_from_start[neighbor] > total_cost: #updating successor if a better option is found
                fringe.update(neighbor, total_cost)
                heap.add(neighbor) 
                cost_from_nodes[neighbor] = cost_between_nodes
                cost_from_start[neighbor] = total_cost
                directions[neighbor] = current_node, action
    
    #return nothing if goal node not found
    if goal_node is None:
        return []
    path = []
    current_node = goal_node
    #creating the path from our direction dictionary
    while current_node != problem.getStartState():
        parent, direction = directions[current_node]
        path.append(direction)
        current_node = parent
    path.reverse()

    return path

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
