# Maze-Solver

## Hardware:
2 DC motors 🏎️
2 wheels and a castor wheel 🚙
7 IR sensors for precise navigation 📡
H-bridge for motor control ⚙️
Arduino Mega for robust processing 🖥️
## Key Features:
### Maze Exploration:
Watch as our bot navigates through the maze using the right-hand algorithm, effectively mapping out the paths and dead ends.
### Optimized Path Solving:
Once the maze is explored, the bot calculates the shortest path to the exit and efficiently navigates to the finish line.
EEPROM Storage: The bot stores the explored path in EEPROM and then optimizes it to solve the maze in the shortest time.

This project showcases our dedication to embedded systems and problem-solving, combining hardware ingenuity with advanced programming. Big thanks to my amazing team Mustafa Tarek and Daniel Atallah for their hard work and innovative thinking!

## Video
https://github.com/MoYasser1777/Maze-Solver/assets/119734442/0f6e979d-0a19-4079-bd9d-cdb829823e4b

## How It Works
Right-Hand Algorithm
The robot uses the right-hand algorithm to explore the maze. This algorithm ensures that the robot covers all possible paths by keeping its right hand (or sensor) on the wall.

## Path Optimization
Once the exploration is complete, the robot retrieves the stored path from EEPROM. It then processes this path to eliminate loops and redundant movements, ensuring that the final path to the exit is the shortest possible route.
