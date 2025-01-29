#include "agent.h"
#include <iostream>

// Constructor: Initialize agent's starting position
Agent::Agent(int startX, int startY) : x(startX), y(startY) {}

// Move the agent based on the action
void Agent::move(const std::string& direction, const Environment& env) {
    int newX = x, newY = y;

    if (direction == "up") newX--;
    else if (direction == "down") newX++;
    else if (direction == "left") newY--;
    else if (direction == "right") newY++;

    std::cout << "Agent attempting move: " << direction 
              << " from (" << x << ", " << y << ") to (" 
              << newX << ", " << newY << ")\n";

    // Check bounds and obstacles
    if (newX >= 0 && newX < env.getRows() &&
        newY >= 0 && newY < env.getCols() &&
        !env.isObstacle(newX, newY)) {
        x = newX;
        y = newY;
        std::cout << "Move successful to (" << x << ", " << y << ")\n";
    } else {
        std::cout << "Move blocked by obstacle or boundary!\n";
    }
}


// Get the agent's current position
std::pair<int, int> Agent::getPosition() const {
    return {x, y};
}
