# HW4 - Leslie Liu
## Part 0: First Steps
This HW extend the feature from previous homework and embedding a decision-tree controller into existing movement and pathfinding system so that the agent can autonomously switch between wandering, wall‑avoidance, and direct or cross-room seeking. 



## Part 1: Decision Tree
Leaf actions are: 
- wander (pick a random interior point and arrive), 
- avoid‑wall (set the room center as a new A* waypoint), or continue (no change). 

1. Move to Part 1 dictionary
2. Run the makefile in directory Part 1

```
make
```
3. Run the command-line arguments specifying the algorithm, which graph file to read, source node, and destination node:
```
./main
```
When the window is opened, you can use the mouse to click random room, and the boid will find a path to the target via A*.

4. Clean up the executable and object files (if needed)
```
make clean
```

## Part 2: Behavior Tree
Key features for this BT are: 
- If the player is detected within a certain radius, the monster should A*-pathfind and chase. 
- If the player is out of range, the monster should either patrol to a random point or perform a “dance” in place.
On collision, both monster and player teleport back to their start positions.


1. Move to Part 2 dictionary
2. Run the makefile in directory Part 3

```
make
```
3. Run the command-line
```
./main
```

4. Clean up the executable and object files (if needed)
```
make clean
```

## Part 3: Decision Tree Learning
1. Move to Part 3 dictionary
2. Run the makefile in directory Part 3

```
make
```
3. Run the commend line
```
./main
```
4. When window open, click the mouse in the room and the character will move toward the position you click. 
5. Clean up the executable and object files (if needed)
```
make clean
```