# multiAgents.py

from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
	"""
	A reflex agent chooses an action at each choice point by examining
	its alternatives via a state evaluation function.
	"""


	def getAction(self, gameState):
		"""
		getAction takes a GameState and returns
		some Directions.X for some X in the set {NORTH, SOUTH, WEST, EAST, STOP}
		"""
		# Collect legal moves and successor states
		legalMoves = gameState.getLegalActions()

		# Choose one of the best actions
		scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
		bestScore = max(scores)
		bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
		chosenIndex = random.choice(bestIndices) # Pick randomly among the best

		return legalMoves[chosenIndex]

	def evaluationFunction(self, currentGameState, action): # basic
		"""
		The evaluation function takes in the current and proposed successor
		GameStates (pacman.py) and returns a number, where higher numbers are better.

		The code below extracts some useful information from the state, like the
		remaining food (newFood) and Pacman position after moving (newPos).
		newScaredTimes holds the number of moves that each ghost will remain
		scared because of Pacman having eaten a power pellet.
		"""
		# Useful information you can extract from a GameState (pacman.py)
		successorGameState = currentGameState.generatePacmanSuccessor(action)
		newPos = successorGameState.getPacmanPosition()
		newFood = successorGameState.getFood()
		newGhostStates = successorGameState.getGhostStates()
		newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]
		GhostPos = successorGameState.getGhostPositions()

		score = 0
		distFood = []
		distGhost = []
		if (len(distFood) == 0):
			score += 1 # eating food pellets give +1
		else:
			for i in newFood.asList():
				dist = util.manhattanDistance(newPos, i)
				distFood.append(dist)
			ClosestFood = min(distFood)
			if (ClosestFood == 0):
				score += 1
			score += 1/ClosestFood # score increases more for closer food

		for i in GhostPos:
			dist = util.manhattanDistance(newPos, i)
			distGhost.append(dist)
		ClosestGhost = min(distGhost) 
		if (ClosestGhost < 2):
			score -= 10
		return score + successorGameState.getScore()

def scoreEvaluationFunction(currentGameState):
	"""
	This default evaluation function just returns the score of the state.
	The score is the same one displayed in the Pacman GUI.
	"""
	return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
	"""
	This class provides some common elements to all of your
	multi-agent searchers.  Any methods defined here will be available
	to the MinimaxPacmanAgent & AlphaBetaPacmanAgent
	"""

	def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
		self.index = 0 # Pacman is always agent index 0
		self.evaluationFunction = util.lookup(evalFn, globals())
		self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
	"""
	Your minimax agent
	"""

	def getAction(self, gameState):
		"""
		Returns the minimax action from the current gameState using self.depth
		and self.evaluationFunction.

		gameState.getLegalActions(agentIndex):
		Returns a list of legal actions for an agent
		agentIndex=0 means Pacman, ghosts are >= 1

		gameState.generateSuccessor(agentIndex, action):
		Returns the successor game state after an agent takes an action

		gameState.getNumAgents():
		Returns the total number of agents in the game

		gameState.isWin():
		Returns whether or not the game state is a winning state

		gameState.isLose():
		Returns whether or not the game state is a losing state
		"""
		

		def value(self, gameState, depth, agentIndex):

			# If the gameState is a terminating state, evaluate the score
			if ((gameState.isWin()) or (gameState.isLose()) or (depth == self.depth)):
				return self.evaluationFunction(gameState)

			# If the agentIndex == 0, i.e. PacMan, maximize the score
			if (agentIndex==0):
				return max_value(self, gameState, depth, agentIndex)[0]

			# If the agentIndex != 0, i.e. ghost, minimize the score
			else:
				return min_value(self, gameState, depth, agentIndex)[0]

		def max_value(self, gameState, depth, agentIndex):
			v = float('-inf')
			bestAction = None

			# For each action PacMan can take, we will be calculating a new value for
			# the same depth, but increased agent index (ghosts)
			for action in gameState.getLegalActions(agentIndex):
				successor = gameState.generateSuccessor(agentIndex, action)
				NewV = value(self, successor, depth, agentIndex + 1)

				# If the new value is greater than the previous greatest value, set new
				# best value and action and return
				if(NewV > v):
					v = NewV
					bestAction = action
			return v, bestAction

		def min_value(self, gameState, depth, agentIndex):
			v = float('inf')
			bestAction = None

			# For each action a ghost can take, we will be calculating a new value for
			# the same depth, but increased agent index (ghosts again)
			for action in gameState.getLegalActions(agentIndex):
				successor = gameState.generateSuccessor(agentIndex, action)
				nextAgentIndex = agentIndex + 1
				nextDepth = depth
				# ONLY change agent index to 0 (PacMan) and increase depth if we have 
				# finished making decisions for all ghosts at that depth
				if (gameState.getNumAgents() == agentIndex + 1):
					nextAgentIndex = 0
					nextDepth = depth + 1
				NewV = value(self, successor, nextDepth, nextAgentIndex)

				# If the new value is lesser than the previous least value, set new
				# best value and action and return
				if(NewV < v):
					v = NewV
					bestAction = action
			return v, bestAction

		v, action = (max_value(self, gameState, 0, 0))
		return action

class AlphaBetaAgent(MultiAgentSearchAgent):
	"""
	Your minimax agent with alpha-beta pruning
	"""

	def getAction(self, gameState):
		"""
		Returns the minimax action using self.depth and self.evaluationFunction
		"""

		def value(self, gameState, depth, agentIndex, alpha, beta):

			# If the gameState is a terminating state, evaluate the score
			if ((gameState.isWin()) or (gameState.isLose()) or (depth == self.depth)):
				return self.evaluationFunction(gameState)

			# If the agentIndex == 0, i.e. PacMan, maximize the score
			if (agentIndex==0):
				return max_value(self, gameState, depth, agentIndex, alpha, beta)[0]

			# If the agentIndex != 0, i.e. ghost, minimize the score
			else:
				return min_value(self, gameState, depth, agentIndex, alpha, beta)[0]

		def max_value(self, gameState, depth, agentIndex, alpha, beta):
			v = float('-inf')
			bestAction = None

			# For each action PacMan can take, we will be calculating a new value for
			# the same depth, but increased agent index (ghosts)
			for action in gameState.getLegalActions(agentIndex):
				successor = gameState.generateSuccessor(agentIndex, action)
				nextAgentIndex = agentIndex + 1
				if (gameState.getNumAgents() == agentIndex + 1):
					nextAgentIndex = 0
				NewV = value(self, successor, depth, nextAgentIndex, alpha, beta)

				# If the new value is greater than the previous greatest value, set new
				# best value and action and return
				if(NewV > v):
					v = NewV
					bestAction = action
				if (v>beta):
					return v, action
				alpha = max(alpha, v)
			return v, bestAction

		def min_value(self, gameState, depth, agentIndex, alpha, beta):
			v = float('inf')
			bestAction = None

			# For each action a ghost can take, we will be calculating a new value for
			# the same depth, but increased agent index (ghosts again)
			for action in gameState.getLegalActions(agentIndex):
				successor = gameState.generateSuccessor(agentIndex, action)
				nextAgentIndex = agentIndex + 1
				nextDepth = depth

				# ONLY change agent index to 0 (PacMan) and increase depth if we have 
				# finished making decisions for all ghosts at that depth
				if (gameState.getNumAgents() == agentIndex + 1):
					nextAgentIndex = 0
					nextDepth = depth + 1
				NewV = value(self, successor, nextDepth, nextAgentIndex, alpha, beta)

				# If the new value is lesser than the previous least value, set new
				# best value and action and return
				if(NewV < v):
					v = NewV
					bestAction = action
				if (v<alpha):
					return v, action
				beta = min(beta, v)
			return v, bestAction

		v, action = (max_value(self, gameState, 0, 0, float('-inf'), float('inf')))
		return action

def betterEvaluationFunction(currentGameState):

    newPos = currentGameState.getPacmanPosition()
    newFood = currentGameState.getFood()
    newGhostStates = currentGameState.getGhostStates()
    ghost_scared_times_list = [ghostState.scaredTimer for ghostState in newGhostStates]

    food_list = newFood.asList()

    distance_to_food_list = [0]
    for pos in food_list:
        distance_to_food_list.append(manhattanDistance(newPos, pos))

    ghost_pos = []
    for ghost in newGhostStates:
        ghost_pos.append(ghost.getPosition())

    distance_to_ghost_list = [0]
    for pos in ghost_pos:
        distance_to_ghost_list.append(manhattanDistance(newPos, pos))

    relative_food_distance = 0
    if sum(distance_to_food_list) > 0:
        relative_food_distance = 1.0 / sum(distance_to_food_list)

    score = currentGameState.getScore() + relative_food_distance + len(newFood.asList(False))

    if sum(ghost_scared_times_list) > 0:
        score += sum(ghost_scared_times_list) - len(currentGameState.getCapsules()) - sum(distance_to_ghost_list) # if ghost is scared
    else:
        score += sum(distance_to_ghost_list) + len(currentGameState.getCapsules())

    return score

# Abbreviation
better = betterEvaluationFunction
