# PACMAN-BOT

Problem At Hand (Part 1)
Our project has a pacman who must make his way through a maze efficiently to a particular location and collect food.
We built general search algorithms and applied it to the pacman maze.
Search algorithms used:
•	 Breadth First search (BFS) which uses the queue data structure to find the target object. BFS starts at the starting point and goes breadthwise searching every cell till it reaches the food
•	Depth first search (DFS) which uses the stack data structure to find the target object. DFS is a recursive algorithm which utilizes backtracking to find the food.
•	A * search which uses a priority queue so that it can associate each cell with a priority number to efficiently reach its target. A * is the algorithm which usually performs the best

Problem At hand (Part 2)
Now, our pacman must also avoid ghosts while collecting food and traversing the maze.
We implemented the minimax search to try get pacman to win.
•	Minimax search is also a backtracking algorithm which is used to find the optimal move for a player (in our case pacman), assuming that the opponent also plays optimally.
•	 Pacman and the ghosts are the maximizer and the minimizer respectively. The maximizer intends to increase the score while the minimizer tries to lower it as much as possible
•	If pacman has the upper hand then the score is positive while if in a game state, the ghost has an upper hand, the score is negative.
