#include "q_learning.h"
#include <cmath>       // For abs
#include <cstdlib>     // For random number generation
#include <algorithm>   // For std::max_element
#include <iostream>    // For debugging
#include <fstream>     // For file I/O
#include <sstream>     // For std::stringstream

// Constructor: Initialize learning parameters
QLearning::QLearning(double alpha, double gamma, double epsilon)
    : alpha(alpha), gamma(gamma), epsilon(epsilon) {}

// Generate a key for the Q-table using relative positions
std::string QLearning::getQKey(std::pair<int, int> state, const std::string& action, const std::pair<int, int>& goal) {
    // Compute the relative position to the goal
    int deltaRow = state.first - goal.first;
    int deltaCol = state.second - goal.second;

    // Create a key using the relative position and action
    return std::to_string(deltaRow) + "," + std::to_string(deltaCol) + "," + action;
}

// Get the maximum Q-value for a state
double QLearning::getMaxQValue(std::pair<int, int> state, const std::pair<int, int>& goal) {
    std::vector<std::string> actions = {"up", "down", "left", "right"};
    double maxQ = -1e9; // Initialize with a very low value

    for (const auto& action : actions) {
        std::string key = getQKey(state, action, goal);
        if (Q.find(key) != Q.end()) {
            maxQ = std::max(maxQ, Q[key]);
        } else {
            Q[key] = 0; // Initialize unseen states with 0
        }
    }
    return maxQ;
}

// Update Q-table using relative positions
void QLearning::updateQTable(std::pair<int, int> state, const std::string& action, double reward, std::pair<int, int> nextState, const std::pair<int, int>& goal) {
    std::string key = getQKey(state, action, goal);
    double maxQ = getMaxQValue(nextState, goal); // Use relative position
    Q[key] = (1 - alpha) * Q[key] + alpha * (reward + gamma * maxQ);
}

// Choose an action based on the epsilon-greedy policy
std::string QLearning::chooseAction(std::pair<int, int> state, const Environment& env, const std::pair<int, int>& goal) {
    std::vector<std::string> actions = {"up", "down", "left", "right"};
    std::vector<std::string> validActions;

    // Filter valid actions
    for (const auto& action : actions) {
        int newX = state.first;
        int newY = state.second;

        if (action == "up") newX--;
        else if (action == "down") newX++;
        else if (action == "left") newY--;
        else if (action == "right") newY++;

        // Check boundaries and obstacles
        if (newX >= 0 && newX < env.getRows() &&
            newY >= 0 && newY < env.getCols() &&
            !env.isObstacle(newX, newY)) {
            validActions.push_back(action);
        }
    }

    // Exploration: Choose a random valid action
    if (!validActions.empty() && (double)rand() / RAND_MAX < epsilon) {
        return validActions[rand() % validActions.size()];
    }

    // Exploitation: Choose the action with the highest Q-value
    double maxQ = -1e9;
    std::string bestAction;
    for (const auto& action : validActions) {
        std::string key = getQKey(state, action, goal);
        if (Q.find(key) != Q.end() && Q[key] > maxQ) {
            maxQ = Q[key];
            bestAction = action;
        }
    }

    // If no valid action is found, choose a random action
    return bestAction.empty() ? validActions[rand() % validActions.size()] : bestAction;
}

// Save Q-table to a file
void QLearning::saveQTable(const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        for (const auto& entry : Q) {
            file << entry.first << "," << entry.second << "\n";
        }
        file.close();
    }
}

// Load Q-table from a file
void QLearning::loadQTable(const std::string& filename) {
    std::ifstream file(filename);
    if (file.is_open()) {
        Q.clear();
        std::string line;
        while (getline(file, line)) {
            std::stringstream ss(line); // Fix: Ensure <sstream> is included
            std::string key;
            double value;
            getline(ss, key, ',');
            ss >> value;
            Q[key] = value;
        }
        file.close();
    }
}
