# HW3 - Leslie Liu
## Part 1: First Steps
### There are two adjacency matrix files and two heuristic files in this folder
1. The small graph was reference to Shapefile of U.S. states, each state’s geometry is accessed:
- Nodes: Each U.S. state (identified by its postal code, like CA, NY, TX) is treated as a node.
- Edges: If two states share a boundary (i.e., their geometries “touch”), the script calculates the distance between their centroids and writes that distance as the edge weight. Otherwise, the weight remains zero (or no direct connection).
- 51 nodes and edges' number is around three times of nodes

2. The large graph was the flights route downloaded from airports.dat and routes.dat from OpenFlights GitHub repository
- Nodes: Each airport is assigned an integer ID. 
- Edges: For every route in routes.dat, the script calculates the haversine distance (in kilometers) between the source and destination airports’ coordinates. This distance becomes the weight of the edge in the adjacency matrix.
- There are 7678 nodes and 66444 edges



## Part 2: Dijkstra’s Algorithm and A*
1. Move to Part 2 dictionary
2. Two algorithms (Dijkstra and A*) is implemented in thsi folder
3. Run the makefile in directory Part 2

```
make
```
4. Run the command-line arguments specifying the algorithm, which graph file to read, source node, and destination node:
```
./<algorithm_file graph.csv> <src> <dest>
```
- 0 => small graph
- 1 => large graph

For example:
```
./dijkstra 0 CA NY
```
```
./dijkstra 1 98 105
```
```
./a_star 0 CA NY
```
```
./a_star 1 98 105
```
5. Clean up the executable and object files (if needed)
```
make clean
```

## Part 3: Heuristics
1. Move to Part 3 dictionary
2. Run the makefile in directory Part 3

```
make
```
3. Run the command-line arguments specifying the algorithm, which graph file to read, source node, destination node and heuristic to use:
```
./astar <0 or 1> <src> <dest> <heurMode>
```
- 0 => small graph
- 1 => large graph
- If heuristicMode == 1, returns round(dist) (admissible).
- If heuristicMode == 2, returns round(dist * 2) (inadmissible).

For example:
```
# Admissible heuristic (mode=1) from "CA" to "NY"
./astar 0 CA NY 1

# Inadmissible heuristic (mode=2)
./astar 0 CA NY 2
```
4. Clean up the executable and object files (if needed)
```
make clean
```

## Part 4: Putting it All Together 
1. Move to Part 4 dictionary
2. Run the makefile in directory Part 4

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
