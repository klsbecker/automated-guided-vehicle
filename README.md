# Automated Guided Vehicle (AGV)

## About the Project
This project was developed as part of the **Microprocessor Systems** course in the Computer Engineering program at Universidade do Vale do Rio dos Sinos (UNISINOS). It consists of building an **automated guided vehicle** (AGV) capable of following marked lines, avoiding obstacles, and calculating the best route to reach a predefined destination.

## Motivation
AGVs are widely used in industrial environments and large-scale warehouses, as demonstrated by the Amazon warehouse robots ([video](https://www.youtube.com/watch?v=1-KS0-xICks)). This project aims to explore similar technologies on a smaller scale, using robotics kits available in the laboratory.

## Features
- Navigation on a predefined grid, moving along the lines.
- Collision avoidance system using infrared sensors.
- Route calculation and decision-making based on Dijkstra's algorithm.
- Ability to follow predefined routes assigned during tests.
- Autonomous execution of tasks without human intervention.

## Technical Specifications
### Hardware
- Robotic base with three wheels (two motorized and one free).
- Infrared sensors for obstacle and line detection.

### Software
- Dijkstra's algorithm for obstacle detection and optimal route calculation.
- Embedded control system for sensor and motor integration.

### Navigation Grid
The robot operates on a grid similar to the one shown below:

```
  E   F   G   H
1 +---+---+---+ 5
  |   |   |   | 
2 +---+---+---+ 6
  |   |   |   |
3 +---+---+---+ 7
  |   |   |   |
4 +---+---+---+ 8
  A   B   C   D
```

This grid represents the robot's operational area, where each cell can contain lines, obstacles, or be a waypoint. The `START_POSITION` is set to **H**, and the `END_POSITION` is set to **3**. Obstacles can be dynamically marked with `'x'`.

## Key Functions in the Code
- **`#define START_POSITION` and `#define END_POSITION`**: Predefined constants specify the grid's starting and ending points, ensuring consistent initialization and pathfinding.
- **`setup()`**: Initializes sensors, motors, and communication interfaces.
- **`loop()`**: Main control loop where the robot scans for lines and obstacles, calculates the optimal path using Dijkstra's algorithm and adjusts motor speeds accordingly.
- **`navigateGrid()`**: Handles movement along the grid based on the calculated path.
- **`detectObstacles()`**: This function uses infrared sensors to identify obstacles and update the path dynamically.
- **`isValidMove()`**: Validates potential movements on the grid by checking boundaries and obstacles.

## How to Execute
1. Assemble the robot using the provided base and sensors.
2. Upload the code to the microcontroller using the Arduino IDE or compatible tool.
3. Place the robot on the test grid and start execution.

## Expected Results
The robot should be able to autonomously navigate the grid, detect and avoid obstacles, and reach the designated destination following the calculated optimal route.

## Future Improvements
- Enhancing the obstacle detection system for higher accuracy.
- Implementing a graphical interface to visualize the robot's path in real time.

---
**Authors:**
- Klaus Becker
- Carlos Souza
