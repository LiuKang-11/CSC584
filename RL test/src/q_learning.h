#ifndef Q_LEARNING_H
#define Q_LEARNING_H

#include <string>
#include <unordered_map>
#include <vector>
#include <utility>
#include "environment.h"

class QLearning {
public:
    QLearning(double alpha, double gamma, double epsilon);
    std::string getQKey(std::pair<int, int> state, const std::string& action, const std::pair<int, int>& goal);
    double getMaxQValue(std::pair<int, int> state, const std::pair<int, int>& goal);
    void updateQTable(std::pair<int, int> state, const std::string& action, double reward, std::pair<int, int> nextState, const std::pair<int, int>& goal);
    std::string chooseAction(std::pair<int, int> state, const Environment& env, const std::pair<int, int>& goal);
    void saveQTable(const std::string& filename);
    void loadQTable(const std::string& filename);

private:
    double alpha; // Learning rate
    double gamma; // Discount factor
    double epsilon; // Exploration rate
    std::unordered_map<std::string, double> Q; // Q-table
};

#endif
