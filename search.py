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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):          #similar to Graph-Search
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """

    frontier = util.Stack()                     # frontier is a Stack
    parents = {}                                # dictionary to store states' parents and action to get to them, format(parent, action)
    explored = []                               # explored list

    current_state = problem.getStartState()
    frontier.push(current_state)
    parents[current_state] = (0, 0)             # Start State's parent is nonexistent

    while not frontier.isEmpty():

        current_state = frontier.pop()

        if (problem.isGoalState(current_state)):    # if Goal State is reached

            temp = []                               # temporary list
            actions = []                            # list of actions needed

            parent = parents[current_state]             # start from Goal State's Parent
            temp.append(parent[1])                      # append to the temp list

            while (parents[parent[0]] != (0, 0)):       # until we reach Start State's parent
                temp.append(parents[parent[0]][1])      # append the action to get to this state
                parent = parents[parent[0]]             # new parent = parent of the current state

            for i in reversed(temp):                # reverse temporary list to start from the first state
                actions.append(i)                   # append actions to action list

            return actions

        else:                                       # if Current State isn't the Goal State
            explored.append(current_state)
            for child_state in problem.getSuccessors(current_state):   # add its successors to the frontier and save current state as their parent
                if child_state[0] not in explored and child_state[0] not in frontier.list:
                    frontier.push(child_state[0])
                    parents[child_state[0]] = (current_state, child_state[1])

    return "Failure"        # if no Goal State is found


def breadthFirstSearch(problem):            # as implemented in lecture notes
    """Search the shallowest nodes in the search tree first."""

    frontier = util.Queue()                 # frontier is a Queue
    parents = {}                            # dictionary to store states' parents and action to get to them, format(parent, action)
    explored = []                           # list of explored states

    current_state = problem.getStartState()
    parents[current_state] = (0, 0)         # parent of Start State is nonexistent

    if(problem.isGoalState(current_state)): # check if Start State is Goal State
        return []

    frontier.push(current_state)

    while not frontier.isEmpty():

        current_state = frontier.pop()
        explored.append(current_state)

        for child_state in problem.getSuccessors(current_state):

            if(child_state[0] not in explored and child_state[0] not in frontier.list):

                parents[child_state[0]] = (current_state, child_state[1])       #store its parent

                if(problem.isGoalState(child_state[0])):            #if it's the Goal State
                    temp = []                                       #same steps as in DFS to find the actions needed
                    actions = []

                    parent = parents[child_state[0]]
                    temp.append(parent[1])

                    while (parents[parent[0]] != (0, 0)):
                        temp.append(parents[parent[0]][1])
                        parent = parents[parent[0]]

                    for i in reversed(temp):
                        actions.append(i)

                    return actions

                else:
                    frontier.push(child_state[0])

    return "Failure"        #if no Goal State is Found

    # util.raiseNotDefined()


def uniformCostSearch(problem):         # as implemented in lecture notes
    """Search the node of least total cost first."""

    pq = util.PriorityQueue()           # frontier is a Priority Queue
    explored = []                       # list of explored states
    info = {}                           # dictionary to store information about states

    start = problem.getStartState()
    pq.push(start, 0)
    info[start] = [0, 0, 0, True]   # store information about a state in a tuple with this format
                                     # (parent, action to get from parent to state, cost of path, isInFrontier)

    while not pq.isEmpty():

        current_state = pq.pop()

        if(problem.isGoalState(current_state)):     #if it is the Goal State

            temp = []
            actions = []

            temp.append(info[current_state][1])     # append action to temp list 
            parent = info[current_state][0]         #parent = parent of current state

            while (info[parent][0] != 0):           # while the parent isn't the parent of the Start State
                temp.append(info[parent][1])        # append action
                parent = info[parent][0]            # parent = parent of parent

            for i in reversed(temp):                # reverse temp
                actions.append(i)                   # copy actions in reverse order

            return actions

        else:                                       # if it is not the Goal State

            explored.append(current_state)
            info[current_state][3] = False          # state is no longer in Froniter

            for child_state in problem.getSuccessors(current_state):

                # cost = cost of edge between parent and child + cost of path until parent
                cost = child_state[2] + info[current_state][2]

                if child_state[0] not in info:                             # if child not in explored list or frontier
                    pq.push(child_state[0], cost)

                    info[child_state[0]] = [current_state, child_state[1], cost, True]  #store info for state

                elif (child_state[0] in info and info[child_state[0]][3] is True    # if child in frontier with a greater cost of path
                        and info[child_state[0]][2] > cost):

                        pq.update(child_state[0], cost)                             # update in frontier

                        info[child_state[0]] = [current_state, child_state[1], cost, True]  # update info

    return "Failure"        # if no Goal State is found


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):              # as implemented in lecture notes
    """Search the node that has the lowest combined cost and heuristic first."""

    pq = util.PriorityQueue()        # frontier is a Priority Queue
    explored = []                    # list of explored states
    info = {}                        # dictionary to store information about states

    start = problem.getStartState()
    pq.push(start, 0)
    info[start] = [0, 0, 0, True]   # store information about a state in a tuple with this format
                                     # (parent, action to get from parent to state, cost of path, isInFrontier) 

    while not pq.isEmpty():

        current_state = pq.pop()

        if(problem.isGoalState(current_state)):         # if current state is Goal State
                                                        # same steps as in UCS
            temp = []
            actions = []

            temp.append(info[current_state][1])
            parent = info[current_state][0]

            while (info[parent][0] != 0):
                temp.append(info[parent][1])
                parent = info[parent][0]

            for i in reversed(temp):
                actions.append(i)

            return actions

        else:                                       # if current state isn't Goal State

            explored.append(current_state)
            info[current_state][3] = False          # state is no longer in Frontier

            for child_state in problem.getSuccessors(current_state):

                # cost of path until child = cost of edge between parent and child + cost of path until parent
                cost = child_state[2] + info[current_state][2]
                # f = cost + heuristic of child
                f = cost + heuristic(child_state[0], problem)

                if child_state[0] not in info:      # if child not in explored or frontier

                    pq.push(child_state[0], f)

                    info[child_state[0]] = [current_state, child_state[1], cost, True]      # store info

                elif (child_state[0] in info and info[child_state[0]][3] is True  # if child state is in frontier with a greater cost of path
                        and info[child_state[0]][2] > cost):

                        pq.update(child_state[0], f)            # update in frontier

                        info[child_state[0]] = [current_state, child_state[1], cost, True]      #update info

    return "Failure"        # if no Goal State is found


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
