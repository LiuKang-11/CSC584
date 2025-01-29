#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <vector>
#include <utility> // for std::pair

class Environment {
public:
    Environment(int rows, int cols);
    void reset(); // Reset the environment to its initial state
    std::pair<int, int> getGoalPosition() const;
    bool isObstacle(int x, int y) const;
    bool isGoal(int x, int y) const;
    double getReward(int x, int y) const;
    void render() const; // Debugging purposes, prints the environment state
    int getRows() const; // Get the number of rows
    int getCols() const; // Get the number of columns

private:
    int rows, cols;
    std::pair<int, int> goal;
    std::vector<std::pair<int, int>> obstacles;

    void generateObstacles(); // Randomly generate obstacles
};

#endif
