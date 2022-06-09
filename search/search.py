# search.py

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem
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


        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    """

    from util import Stack

    # Creates an empty Stack.
    open_list = Stack()

    visited_list = []
    path = []
    action_cost = 0  # Cost of each movement.

    start_position = problem.getStartState()

    # Pushes the start position to the stack.
    open_list.push((start_position, path, action_cost))

    while not open_list.isEmpty():

        current_node = open_list.pop()
        position = current_node[0]
        path = current_node[1]

        # Pushes the current position to the visited list if it is not visited.
        if position not in visited_list:
            visited_list.append(position)

        # Returns the final path if the current position is goal.
        if problem.isGoalState(position):
            return path

        # Gets successors of the current node.
        successors = problem.getSuccessors(position)

        # Pushes the current node's successors to the stack if they are not visited.
        for item in successors:
            if item[0] not in visited_list:

                new_position = item[0]
                new_path = path + [item[1]]
                open_list.push((new_position, new_path, item[2]))

    util.raiseNotDefined()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    from util import Queue

    # Creates an empty Queue.
    open_list = Queue()

    visited_list = []
    path = []
    action_cost = 0  # Cost of each movement.

    start_position = problem.getStartState()

    # Pushes the start position to the Queue.
    open_list.push((start_position, path, action_cost))

    while not open_list.isEmpty():

        current_node = open_list.pop()
        position = current_node[0]
        path = current_node[1]

        # Pushes the current position to the visited list if it is not visited.
        if position not in visited_list:
            visited_list.append(position)

        # Returns the final path if the current position is goal.
        if problem.isGoalState(position):
            return path

        # Gets successors of the current node.
        successors = problem.getSuccessors(position)

        # Pushes the current node's successors to the Queue if they are not visited.
        # We check both visited and open list.
        for item in successors:
            if item[0] not in visited_list and item[0] not in (node[0] for node in open_list.list):

                new_position = item[0]
                new_path = path + [item[1]]
                open_list.push((new_position, new_path, item[2]))

    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    from util import PriorityQueue

    # Creates an empty PriorityQueue.
    open_list = PriorityQueue()

    visited_list = []
    path = []
    priority = 0    # Initializes the priority to 0.

    start_position = problem.getStartState()

    # Pushes the start position to the PriorityQueue.
    open_list.push((start_position, path), priority)

    while not open_list.isEmpty():

        current_node = open_list.pop()
        position = current_node[0]
        path = current_node[1]

        # Returns the final path if the current position is goal.
        if problem.isGoalState(position):
            return path

        # Pushes the current position to the visited list if it is not visited.
        if position not in visited_list:
            visited_list.append(position)

            # Gets successors of the current node.
            successors = problem.getSuccessors(position)

            # Pushes the current node's successors to the PriorityQueue if they are not visited.
            for item in successors:
                if item[0] not in visited_list:
                    new_position = item[0]
                    new_path = path + [item[1]]

                    # Updates priority of the successor using f(n) function.

                    """ g(n): Current cost from start state to the current position. """
                    g = problem.getCostOfActions(new_path)

                    """ h(n): Estimate of the lowest cost from the current position to the goal state. """
                    h = heuristic(new_position, problem)

                    """ f(n): Estimate of the lowest cost of the solution path
                              from start state to the goal state passing through the current position """
                    f = g + h

                    new_priority = f
                    open_list.push((new_position, new_path), new_priority)

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
