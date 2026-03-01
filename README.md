# FleetNav-Multi-Agent-Pathfinding-Engine
A pure Python algorithmic routing engine that calculates collision-free, energy-efficient paths for multiple autonomous agents navigating a constrained 2D grid.

FleetNav is a custom-built pathfinding planner designed to navigate multiple robots or agents across a complex 2D coordinate grid. The engine computes optimal routes from starting positions to designated goals, ensuring each agent successfully visits an ordered sequence of required checkpoints along the way. It acts as a robust exploration of state-space search algorithms in constrained environments.


# Key Features

Priority-Based Collision Avoidance: Routes agents sequentially based on an assigned priority tier . Lower-priority agents factor in the time-space reservations of higher-priority agents, actively waiting or rerouting to guarantee no two agents occupy the same cell at the same time .



Complex Environmental Constraints: Safely navigates around static obstacles and strictly adheres to directional movement rules imposed by simulated one-way passages .



Resource Management: Tracks and enforces strict individual energy limits for each agent, capping the total number of moves or wait actions they can execute during their route.



Search-Based Core: Powered by foundational search algorithms (such as Breadth-First Search or Uniform-Cost Search). The state representation tracks current position, time steps, and checkpoint progression to find valid paths .


Automated File I/O: Processes environment layouts, grid bounds, and agent parameters from a standardized configuration file (input.txt) and outputs detailed step-by-step routing logs (output.txt) .

# Tech Stack

Language: Python 3 


Dependencies: Built purely with the Python standard library (Zero external dependencies).
