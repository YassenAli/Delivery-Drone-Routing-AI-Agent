# Delivery Drone Routing AI Agent

[![Prolog](https://img.shields.io/badge/Prolog-FF6F00?style=flat&logo=swi-prolog&logoColor=white)](https://www.swi-prolog.org/)
![AI Assignment](https://img.shields.io/badge/FCAT_AI_Assignment-4B32C3?style=flat)

An implementation of search algorithms for drone package delivery in Prolog, developed as part of the Artificial Intelligence course at the Faculty of Computers & Artificial Intelligence.

## Problem Description

### Problem 1: Uninformed Search (BFS)
**Objective:** Navigate a grid to maximize package delivery while avoiding obstacles  
**Grid Elements:**
- `D`: Drone starting position
- `P`: Delivery points
- `O`: Obstacles
- `-`: Empty cells

**Requirements:**
- Implement BFS/DFS to visit maximum `P`s
- Avoid all `O`s
- Bonus: Visualize grid states

### Problem 2: Informed Search (A* with Energy Constraints)
**Objective:** Optimize package delivery with energy management  
**Additional Elements:**
- `R`: Recharge station
- Energy units (user-defined)

**Requirements:**
- Implement A* for shortest path
- Energy consumption (1 unit/move)
- Recharging at `R`
- Bonus: Energy constraints implementation

## Features

**Problem 1**
- Grid visualization at each step
- Optimal path finding using BFS
- Obstacle avoidance logic
- Path cost calculation

**Problem 2**
- A* with admissible heuristic
- Energy management system
- Recharge station integration
- Adaptive path planning
- Combined cost (distance + energy) optimization

## Installation & Usage

### Requirements
- [SWI-Prolog](https://www.swi-prolog.org/) (v8.4+ recommended)

### Running the Solutions

1. Clone repository:
```bash
git clone https://github.com/yourusername/drone-routing-ai.git
cd drone-routing-ai
```

2. Run Problem 1 (Uninformed Search):
```prolog
swipl -s uninformed_search.pl
?- find_best_path(Path, Collected), print_steps(Path).
```

3. Run Problem 2 (Informed Search with Energy):
```prolog
swipl -s informed_search.pl
?- find_best_path_astar(Path, Collected, Cost, 10), print_steps(Path).
```

### Key Predicates

#### Problem 1

`find_best_path/2`: Main solution predicate
`generate_step_grid/4`: Grid visualization
`bfs/4`: BFS implementation

#### Problem 2

`find_best_path_astar/4`: A* solution with energy
`next_state_astar/3`: State generation with energy
`heuristic/3`: Custom heuristic function

## Example Usage
### Problem 1 Output
```prolog
Drone Route:
D - P - O
- O - - P
- - O P -
P O - - -
- - P O -

Steps:
* D P - O
- O - - P
- - O P -
P O - - -
- - P O -
...
Final:
* * * - O
- O * * *
- - O * *
D O * * -
* * * O -
```

### Problem 2 Output (Energy=10)
```prolog
Optimal Path with 10 Energy:
Total Collected: 5
Total Cost: 12
Path: [(1,1), (1,2), (2,2), ..., (5,5)]
```

### Bonus Features Implemented
- ✅ Full grid visualization for both problems
- ✅ Energy management system with recharge
- ✅ Dynamic path regeneration based on energy
- ✅ Obstacle-aware heuristic function
- ✅ Step-by-step execution tracing
