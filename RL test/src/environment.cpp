#include "environment.h"
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <algorithm> // For std::remove

// Constructor: Initialize grid dimensions
Environment::Environment(int rows, int cols) : rows(rows), cols(cols) {
    std::srand(std::time(0)); // Seed random generator
    reset();
}

// Reset the environment to its initial state
void Environment::reset() {
    obstacles.clear();
    goal = {std::rand() % rows, std::rand() % cols};
    generateObstacles();

    // Ensure starting position (0, 0) is not an obstacle
    obstacles.erase(std::remove(obstacles.begin(), obstacles.end(), std::make_pair(0, 0)), obstacles.end());

    // Ensure the goal is not at (0, 0)
    if (goal == std::make_pair(0, 0)) {
        goal = {std::rand() % rows, std::rand() % cols};
    }
}


// Generate random obstacles in the grid
void Environment::generateObstacles() {
    int obstacleCount = (rows * cols) / 10; // Adjust obstacle density
    for (int i = 0; i < obstacleCount; ++i) {
        int x = std::rand() % rows;
        int y = std::rand() % cols;
        // Avoid placing obstacles on the goal
        if (std::make_pair(x, y) != goal) {
            obstacles.push_back({x, y});
        }
    }
}

// Check if a cell contains an obstacle
bool Environment::isObstacle(int x, int y) const {
    for (const auto& obstacle : obstacles) {
        if (obstacle.first == x && obstacle.second == y) {
            return true;
        }
    }
    return false;
}

// Check if a cell is the goal
bool Environment::isGoal(int x, int y) const {
    return (x == goal.first && y == goal.second);
}

// Get the reward for a cell
double Environment::getReward(int x, int y) const {
    if (isGoal(x, y)) return 10.0;     // High reward for goal
    if (isObstacle(x, y)) return -1.0; // Penalty for hitting obstacles
    return -0.1; // Small penalty for each step
}

// Print the environment (for debugging)
void Environment::render() const {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (isGoal(i, j)) {
                std::cout << "G ";
            } else if (isObstacle(i, j)) {
                std::cout << "X ";
            } else {
                std::cout << ". ";
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

// Get the number of rows
int Environment::getRows() const { return rows; }

// Get the number of columns
int Environment::getCols() const { return cols; }

std::pair<int, int> Environment::getGoalPosition() const {
    return goal;
}
