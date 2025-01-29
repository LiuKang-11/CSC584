#ifndef AGENT_H
#define AGENT_H

#include "environment.h"

class Agent {
public:
    Agent(int startX, int startY);
    void move(const std::string& direction, const Environment& env);
    std::pair<int, int> getPosition() const;

private:
    int x, y; // Current position of the agent
};

#endif
